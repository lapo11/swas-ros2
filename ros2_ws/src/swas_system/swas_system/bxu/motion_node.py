# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# # When running on the Pi later, you'll import and use serial and global_config
# # import serial, time
# # from swas_system.common.global_config import MOTION_MAX_V, MOTION_ACCEL, MOTION_DT, ...


# class BxuMotionNode(Node):
#     def __init__(self):
#         super().__init__('bxu_motion_node')

#         # TODO: on the Pi, open the real serial port here, using your move_port
#         # For now on the laptop, we just log the command.
#         self.get_logger().info('BXU motion node started (PRINT-ONLY VERSION, no serial).')

#         self.subscription = self.create_subscription(
#             Twist,
#             '/bxu/cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )
#         self.subscription  # avoid unused warning

#     def cmd_vel_callback(self, msg: Twist):
#         # Map ROS Twist to your Pico's expected command:
#         # V vx vy omega
#         vx = msg.linear.x
#         vy = msg.linear.y
#         omega = msg.angular.z

#         command = f"V {vx:.3f} {vy:.3f} {omega:.3f}"

#         # On the Pi later:
#         # self.move_ser.write((command + "\n").encode())
#         # self.move_ser.flush()

#         # For now, just print/log
#         self.get_logger().info(f"Would send to Motion Pico: {command}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = BxuMotionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import time


HEARTBEAT_INTERVAL_S = 0.2  # matches your global_config value — move to bxu_params.yaml later


class BxuMotionNode(Node):
    def __init__(self):
        super().__init__('bxu_motion_node')

        # Declare ROS 2 parameter so port can be set from bxu_params.yaml without editing code
        self.declare_parameter(
            'move_port',
            '/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_31c5dd4d04cc2c72-if00'
        )
        self.declare_parameter('baud_rate', 115200)

        move_port = self.get_parameter('move_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Open serial connection to Motion Pico
        try:
            self.move_ser = serial.Serial(move_port, baud_rate, timeout=0.05)
            time.sleep(2)  # wait for Pico to finish reset
            self.get_logger().info(f'BXU motion node started. Motion Pico on {move_port}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Could not open Motion Pico serial port: {e}')
            raise SystemExit(1)

        # Heartbeat: sends V 0 0 0 periodically so the Pico watchdog knows Pi is alive.
        # If the node dies or shuts down cleanly, silence triggers the Pico watchdog
        # and the motors relax automatically — same behaviour as MecanumController.
        self._heartbeat_active = True
        self._holding = True
        self._hb_thread = threading.Thread(target=self._heartbeat, daemon=True)
        self._hb_thread.start()

        self.subscription = self.create_subscription(
            Twist,
            '/bxu/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    # ── Heartbeat ──────────────────────────────────────────────────────

    def _heartbeat(self):
        """
        Sends V 0 0 0 every HEARTBEAT_INTERVAL_S while _holding is True.
        When the node receives a real cmd_vel, _holding is set False and
        the heartbeat idles — the subscriber is in control.
        When cmd_vel goes silent (no new message), the last received
        velocity stays in effect until the next heartbeat cycle picks up
        again after _holding is restored via stop().
        """
        while self._heartbeat_active:
            if self._holding:
                try:
                    self.move_ser.write(b'V 0 0 0\n')
                    self.move_ser.flush()
                except Exception:
                    pass
            time.sleep(HEARTBEAT_INTERVAL_S)

    # ── Subscription callback ──────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        vx    = msg.linear.x
        vy    = msg.linear.y
        omega = msg.angular.z

        if vx == 0.0 and vy == 0.0 and omega == 0.0:
            # Zero velocity: re-enable heartbeat so Pico watchdog stays fed
            self._holding = True
            try:
                self.move_ser.write(b'V 0 0 0\n')
                self.move_ser.flush()
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')
        else:
            # Active motion: disable heartbeat, send command directly
            self._holding = False
            command = f'V {vx:.3f} {vy:.3f} {omega:.3f}\n'
            try:
                self.move_ser.write(command.encode())
                self.move_ser.flush()
                self.get_logger().debug(f'Sent: {command.strip()}')
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    # ── Clean shutdown ─────────────────────────────────────────────────

    def destroy_node(self):
        """
        Stops heartbeat on shutdown. Silence → Pico watchdog fires after ~500ms
        → motors relax. Same behaviour as MecanumController.shutdown().
        Do NOT send V 0 0 0 here.
        """
        self.get_logger().info('BXU motion node shutting down — motors will relax via watchdog.')
        self._holding = False
        self._heartbeat_active = False
        if hasattr(self, 'move_ser') and self.move_ser.is_open:
            self.move_ser.close()
        super().destroy_node()


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