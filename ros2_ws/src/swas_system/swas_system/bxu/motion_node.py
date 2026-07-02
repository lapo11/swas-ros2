import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial


class BxuMotionNode(Node):
    def __init__(self):
        super().__init__('bxu_motion_node')

        self.declare_parameter(
            'move_port',
            '/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_31c5dd4d04cc2c72-if00'
        )
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout_s', 0.05)
        self.declare_parameter('startup_delay_s', 2.0)

        self.declare_parameter('cmd_vel_topic', '/bxu/cmd_vel')
        self.declare_parameter('status_topic', '/bxu/motion/status')
        self.declare_parameter('heartbeat_interval_s', 0.2)

        self.declare_parameter('enable_heartbeat', True)
        self.declare_parameter('heartbeat_command', 'V 0 0 0')
        self.declare_parameter('zero_command', 'V 0 0 0')

        self.declare_parameter('max_vx', 0.30)
        self.declare_parameter('max_vy', 0.30)
        self.declare_parameter('max_omega', 0.80)

        self.declare_parameter('min_nonzero_vx', 0.0)
        self.declare_parameter('min_nonzero_vy', 0.0)
        self.declare_parameter('min_nonzero_omega', 0.0)

        self.declare_parameter('invert_vx', False)
        self.declare_parameter('invert_vy', False)
        self.declare_parameter('invert_omega', False)

        self.declare_parameter('command_prefix', 'V')
        self.declare_parameter('command_decimal_places', 3)
        self.declare_parameter('flush_each_write', True)

        self.declare_parameter('log_commands', False)
        self.declare_parameter('publish_status', True)

        self.move_port = self.get_parameter('move_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self.startup_delay_s = float(self.get_parameter('startup_delay_s').value)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.heartbeat_interval_s = float(self.get_parameter('heartbeat_interval_s').value)

        self.enable_heartbeat = bool(self.get_parameter('enable_heartbeat').value)
        self.heartbeat_command = self.get_parameter('heartbeat_command').value.strip()
        self.zero_command = self.get_parameter('zero_command').value.strip()

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_omega = float(self.get_parameter('max_omega').value)

        self.min_nonzero_vx = float(self.get_parameter('min_nonzero_vx').value)
        self.min_nonzero_vy = float(self.get_parameter('min_nonzero_vy').value)
        self.min_nonzero_omega = float(self.get_parameter('min_nonzero_omega').value)

        self.invert_vx = bool(self.get_parameter('invert_vx').value)
        self.invert_vy = bool(self.get_parameter('invert_vy').value)
        self.invert_omega = bool(self.get_parameter('invert_omega').value)

        self.command_prefix = self.get_parameter('command_prefix').value
        self.command_decimal_places = int(self.get_parameter('command_decimal_places').value)
        self.flush_each_write = bool(self.get_parameter('flush_each_write').value)

        self.log_commands = bool(self.get_parameter('log_commands').value)
        self.publish_status_enabled = bool(self.get_parameter('publish_status').value)

        self.status_pub = None
        if self.publish_status_enabled:
            self.status_pub = self.create_publisher(String, self.status_topic, 10)

        try:
            self.move_ser = serial.Serial(
                self.move_port,
                self.baud_rate,
                timeout=self.serial_timeout_s
            )
            time.sleep(self.startup_delay_s)
            self.get_logger().info(
                f'BXU motion node started. Motion Pico on {self.move_port}, baud={self.baud_rate}'
            )
            self._publish_status(f'CONNECTED:{self.move_port}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Could not open Motion Pico serial port: {e}')
            self._publish_status(f'ERROR:OPEN_FAILED:{e}')
            raise SystemExit(1)

        self._heartbeat_active = True
        self._holding = True
        self._serial_lock = threading.Lock()

        if self.enable_heartbeat:
            self._hb_thread = threading.Thread(target=self._heartbeat, daemon=True)
            self._hb_thread.start()

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

    def _publish_status(self, text: str):
        if self.status_pub is not None:
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    def _clamp(self, value: float, limit: float) -> float:
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def _apply_min_nonzero(self, value: float, min_nonzero: float) -> float:
        if value == 0.0:
            return 0.0
        if abs(value) < min_nonzero:
            return min_nonzero if value > 0.0 else -min_nonzero
        return value

    def _format_command(self, vx: float, vy: float, omega: float) -> str:
        fmt = f'{{:.{self.command_decimal_places}f}}'
        return (
            f'{self.command_prefix} '
            f'{fmt.format(vx)} '
            f'{fmt.format(vy)} '
            f'{fmt.format(omega)}'
        )

    def _write_command(self, command: str):
        try:
            with self._serial_lock:
                self.move_ser.write((command + '\n').encode())
                if self.flush_each_write:
                    self.move_ser.flush()
            if self.log_commands:
                self.get_logger().info(f'Sent: {command}')
            self._publish_status(f'CMD:{command}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
            self._publish_status(f'ERROR:WRITE_FAILED:{e}')

    def _heartbeat(self):
        while self._heartbeat_active:
            if self._holding:
                self._write_command(self.heartbeat_command)
            time.sleep(self.heartbeat_interval_s)

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        if self.invert_vx:
            vx = -vx
        if self.invert_vy:
            vy = -vy
        if self.invert_omega:
            omega = -omega

        vx = self._clamp(vx, self.max_vx)
        vy = self._clamp(vy, self.max_vy)
        omega = self._clamp(omega, self.max_omega)

        vx = self._apply_min_nonzero(vx, self.min_nonzero_vx)
        vy = self._apply_min_nonzero(vy, self.min_nonzero_vy)
        omega = self._apply_min_nonzero(omega, self.min_nonzero_omega)

        if vx == 0.0 and vy == 0.0 and omega == 0.0:
            self._holding = True
            self._write_command(self.zero_command)
            return

        self._holding = False
        command = self._format_command(vx, vy, omega)
        self._write_command(command)

    def destroy_node(self):
        self.get_logger().info(
            'BXU motion node shutting down — motors will relax via watchdog.'
        )
        self._holding = False
        self._heartbeat_active = False

        if hasattr(self, 'move_ser') and self.move_ser.is_open:
            try:
                self.move_ser.close()
            except Exception:
                pass

        self._publish_status('DISCONNECTED')
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