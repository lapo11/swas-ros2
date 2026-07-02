import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from swas_msgs.msg import BxuAlignmentOutput


class BxuDockingControllerNode(Node):
    def __init__(self):
        super().__init__('bxu_docking_controller_node')

        self.declare_parameter('alignment_topic', '/bxu/docking_vision/alignment')
        self.declare_parameter('cmd_vel_topic', '/bxu/cmd_vel')
        self.declare_parameter('status_topic', '/bxu/docking/controller_status')

        self.declare_parameter('kp_xy', 0.0040)
        self.declare_parameter('kd_xy', 0.0004)


        # self.declare_parameter('kp_xy', 0.0045)
        # self.declare_parameter('kd_xy', 0.0004)
        self.declare_parameter('kp_theta', 10.0)
        self.declare_parameter('kd_theta', 0.25)

        self.declare_parameter('max_vxy', 0.04)
        self.declare_parameter('max_omega', 0.12)

        self.declare_parameter('d_filter_alpha', 0.9)
        self.declare_parameter('dock_dt', 0.1)

        self.declare_parameter('xy_threshold_mm', 1.0)
        self.declare_parameter('theta_threshold_deg', 2.0)
        self.declare_parameter('stable_frames', 10)

        self.declare_parameter('lost_tolerance_frames', 3)
        self.declare_parameter('publish_status', True)

        alignment_topic = self.get_parameter('alignment_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.kd_xy = float(self.get_parameter('kd_xy').value)
        self.kp_theta = float(self.get_parameter('kp_theta').value)
        self.kd_theta = float(self.get_parameter('kd_theta').value)

        self.max_vxy = float(self.get_parameter('max_vxy').value)
        self.max_omega = float(self.get_parameter('max_omega').value)

        self.d_filter_alpha = float(self.get_parameter('d_filter_alpha').value)
        self.dock_dt = float(self.get_parameter('dock_dt').value)

        self.xy_threshold_mm = float(self.get_parameter('xy_threshold_mm').value)
        self.theta_threshold_deg = float(self.get_parameter('theta_threshold_deg').value)
        self.stable_frames = int(self.get_parameter('stable_frames').value)

        self.lost_tolerance_frames = int(self.get_parameter('lost_tolerance_frames').value)
        self.publish_status_enabled = bool(self.get_parameter('publish_status').value)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = None
        if self.publish_status_enabled:
            self.status_pub = self.create_publisher(String, status_topic, 10)

        self._prev_ex = None
        self._prev_ey = None
        self._prev_theta = None
        self._dex = 0.0
        self._dey = 0.0
        self._dtheta = 0.0

        self._stable_count = 0
        self._lost_count = 0

        self.subscription = self.create_subscription(
            BxuAlignmentOutput,
            alignment_topic,
            self.alignment_callback,
            10
        )

        self.get_logger().info('BXU docking controller node started.')

    def _publish_status(self, text: str):
        if self.status_pub is not None:
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    def _reset_pd(self):
        self._prev_ex = None
        self._prev_ey = None
        self._prev_theta = None
        self._dex = 0.0
        self._dey = 0.0
        self._dtheta = 0.0

    def alignment_callback(self, msg: BxuAlignmentOutput):
        if not msg.markers_detected:
            self._lost_count += 1
            if self._lost_count >= self.lost_tolerance_frames:
                self.cmd_vel_pub.publish(Twist())
                self._publish_status('LOST')
            return

        self._lost_count = 0

        ex_r = msg.forward_error_m * 1000.0
        ey_r = msg.lateral_error_m * 1000.0
        theta_err = msg.yaw_error_rad

        dist_mm = float(np.sqrt(ex_r ** 2 + ey_r ** 2))
        theta_deg = float(np.degrees(abs(theta_err)))
        converged = dist_mm < self.xy_threshold_mm and theta_deg < self.theta_threshold_deg

        if self._prev_ex is None:
            raw_dex = 0.0
            raw_dey = 0.0
            raw_dtheta = 0.0
        else:
            raw_dex = (ex_r - self._prev_ex) / self.dock_dt
            raw_dey = (ey_r - self._prev_ey) / self.dock_dt
            raw_dtheta = (theta_err - self._prev_theta) / self.dock_dt

        self._dex = self.d_filter_alpha * self._dex + (1.0 - self.d_filter_alpha) * raw_dex
        self._dey = self.d_filter_alpha * self._dey + (1.0 - self.d_filter_alpha) * raw_dey
        self._dtheta = self.d_filter_alpha * self._dtheta + (1.0 - self.d_filter_alpha) * raw_dtheta

        self._prev_ex = ex_r
        self._prev_ey = ey_r
        self._prev_theta = theta_err

        vx = float(np.clip(-(self.kp_xy * ex_r + self.kd_xy * self._dex), -self.max_vxy, self.max_vxy))
        vy = float(np.clip(-(self.kp_xy * ey_r + self.kd_xy * self._dey), -self.max_vxy, self.max_vxy))
        omega = float(np.clip(-(self.kp_theta * theta_err + self.kd_theta * self._dtheta), -self.max_omega, self.max_omega))

        if converged:
            vx, vy, omega = 0.0, 0.0, 0.0
            self._stable_count += 1
        else:
            self._stable_count = 0

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

        if self._stable_count >= self.stable_frames:
            self._publish_status('DOCKED')
        else:
            self._publish_status(
                f'ALIGNING dist_mm={dist_mm:.2f} theta_deg={theta_deg:.2f} '
                f'stable={self._stable_count}/{self.stable_frames}'
            )

    def destroy_node(self):
        self.get_logger().info('BXU docking controller node shutting down — publishing zero velocity.')
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BxuDockingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()