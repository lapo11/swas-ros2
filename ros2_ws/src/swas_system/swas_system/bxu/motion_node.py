import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# When running on the Pi later, you'll import and use serial and global_config
# import serial, time
# from swas_system.common.global_config import MOTION_MAX_V, MOTION_ACCEL, MOTION_DT, ...


class BxuMotionNode(Node):
    def __init__(self):
        super().__init__('bxu_motion_node')

        # TODO: on the Pi, open the real serial port here, using your move_port
        # For now on the laptop, we just log the command.
        self.get_logger().info('BXU motion node started (PRINT-ONLY VERSION, no serial).')

        self.subscription = self.create_subscription(
            Twist,
            '/bxu/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # avoid unused warning

    def cmd_vel_callback(self, msg: Twist):
        # Map ROS Twist to your Pico's expected command:
        # V vx vy omega
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        command = f"V {vx:.3f} {vy:.3f} {omega:.3f}"

        # On the Pi later:
        # self.move_ser.write((command + "\n").encode())
        # self.move_ser.flush()

        # For now, just print/log
        self.get_logger().info(f"Would send to Motion Pico: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = BxuMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()