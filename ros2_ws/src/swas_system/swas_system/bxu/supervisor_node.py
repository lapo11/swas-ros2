# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Bool
# from geometry_msgs.msg import Twist


# WAITING = 'WAITING'
# FRONT_APPROACH = 'FRONT_APPROACH'
# DOCKING_ALIGN = 'DOCKING_ALIGN'
# READY_TO_LIFT = 'READY_TO_LIFT'
# FAULT = 'FAULT'


# class BxuSupervisorNode(Node):
#     def __init__(self):
#         super().__init__('bxu_supervisor_node')

#         self.declare_parameter('front_status_topic', '/bxu/front_approach/controller_status')
#         self.declare_parameter('dock_status_topic', '/bxu/docking/controller_status')
#         self.declare_parameter('supervisor_state_topic', '/bxu/supervisor/state')
#         self.declare_parameter('front_enable_topic', '/bxu/front_controller/enabled')
#         self.declare_parameter('dock_enable_topic', '/bxu/docking_controller/enabled')
#         self.declare_parameter('cmd_vel_topic', '/bxu/cmd_vel')
#         self.declare_parameter('auto_start', True)

#         front_status_topic = self.get_parameter('front_status_topic').value
#         dock_status_topic = self.get_parameter('dock_status_topic').value
#         supervisor_state_topic = self.get_parameter('supervisor_state_topic').value
#         front_enable_topic = self.get_parameter('front_enable_topic').value
#         dock_enable_topic = self.get_parameter('dock_enable_topic').value
#         cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
#         auto_start = bool(self.get_parameter('auto_start').value)

#         self.state_pub = self.create_publisher(String, supervisor_state_topic, 10)
#         self.front_enable_pub = self.create_publisher(Bool, front_enable_topic, 10)
#         self.dock_enable_pub = self.create_publisher(Bool, dock_enable_topic, 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

#         self.front_status_sub = self.create_subscription(String, front_status_topic, self._front_status_cb, 10)
#         self.dock_status_sub = self.create_subscription(String, dock_status_topic, self._dock_status_cb, 10)

#         self.front_status = ''
#         self.dock_status = ''
#         self.state = WAITING

#         self.timer = self.create_timer(0.2, self._tick)

#         if auto_start:
#             self._set_state(FRONT_APPROACH)
#         else:
#             self._set_state(WAITING)

#         self.get_logger().info('BXU supervisor node started.')

#     def _publish_bool(self, pub, value: bool):
#         msg = Bool()
#         msg.data = value
#         pub.publish(msg)

#     def _publish_state(self):
#         msg = String()
#         msg.data = self.state
#         self.state_pub.publish(msg)

#     def _publish_zero(self):
#         self.cmd_vel_pub.publish(Twist())

#     def _set_state(self, new_state: str):
#         if new_state != self.state:
#             self.get_logger().info(f'{self.state} -> {new_state}')
#             self.state = new_state
#         self._publish_state()

#     def _front_status_cb(self, msg: String):
#         self.front_status = msg.data

#     def _dock_status_cb(self, msg: String):
#         self.dock_status = msg.data

#     def _tick(self):
#         if self.state == WAITING:
#             self._publish_bool(self.front_enable_pub, False)
#             self._publish_bool(self.dock_enable_pub, False)
#             self._publish_zero()
#             self._publish_state()
#             return

#         if self.state == FRONT_APPROACH:
#             self._publish_bool(self.front_enable_pub, True)
#             self._publish_bool(self.dock_enable_pub, False)
#             self._publish_state()

#             if self.front_status == 'DONE':
#                 self._publish_zero()
#                 self._set_state(DOCKING_ALIGN)
#                 return

#             if self.front_status == 'FAULT':
#                 self._publish_zero()
#                 self._set_state(FAULT)
#                 return

#             return

#         if self.state == DOCKING_ALIGN:
#             self._publish_bool(self.front_enable_pub, False)
#             self._publish_bool(self.dock_enable_pub, True)
#             self._publish_state()

#             if self.dock_status == 'DOCKED':
#                 self._publish_zero()
#                 self._set_state(READY_TO_LIFT)
#                 return

#             if self.dock_status == 'LOST':
#                 self._publish_zero()
#                 self._set_state(FAULT)
#                 return

#             return

#         if self.state == READY_TO_LIFT:
#             self._publish_bool(self.front_enable_pub, False)
#             self._publish_bool(self.dock_enable_pub, False)
#             self._publish_zero()
#             self._publish_state()
#             return

#         if self.state == FAULT:
#             self._publish_bool(self.front_enable_pub, False)
#             self._publish_bool(self.dock_enable_pub, False)
#             self._publish_zero()
#             self._publish_state()
#             return


# def main(args=None):
#     rclpy.init(args=args)
#     node = BxuSupervisorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


WAITING = 'WAITING'
FRONT_APPROACH = 'FRONT_APPROACH'
DOCKING_ALIGN = 'DOCKING_ALIGN'
LIFTING = 'LIFTING'
WAIT_USER_AFTER_LIFT = 'WAIT_USER_AFTER_LIFT'
LOWERING = 'LOWERING'
WAIT_USER_AFTER_LOWER = 'WAIT_USER_AFTER_LOWER'
DRIVE_AWAY = 'DRIVE_AWAY'
DONE = 'DONE'
FAULT = 'FAULT'


class BxuSupervisorNode(Node):
    def __init__(self):
        super().__init__('bxu_supervisor_node')

        self.declare_parameter('front_status_topic', '/bxu/front_approach/controller_status')
        self.declare_parameter('dock_status_topic', '/bxu/docking/controller_status')
        self.declare_parameter('supervisor_state_topic', '/bxu/supervisor/state')
        self.declare_parameter('front_enable_topic', '/bxu/front_controller/enabled')
        self.declare_parameter('dock_enable_topic', '/bxu/docking_controller/enabled')
        self.declare_parameter('cmd_vel_topic', '/bxu/cmd_vel')
        self.declare_parameter('auto_start', True)

        self.declare_parameter('lift_command_topic', '/bxu_battery_lift/command')
        self.declare_parameter('lift_state_topic', '/bxu_battery_lift/state')

        self.declare_parameter('drive_away_speed', -0.15)   # m/s, negative x = backward
        self.declare_parameter('drive_away_duration', 2.0)  # seconds

        front_status_topic = self.get_parameter('front_status_topic').value
        dock_status_topic = self.get_parameter('dock_status_topic').value
        supervisor_state_topic = self.get_parameter('supervisor_state_topic').value
        front_enable_topic = self.get_parameter('front_enable_topic').value
        dock_enable_topic = self.get_parameter('dock_enable_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        auto_start = bool(self.get_parameter('auto_start').value)

        lift_command_topic = self.get_parameter('lift_command_topic').value
        lift_state_topic = self.get_parameter('lift_state_topic').value

        self.drive_away_speed = float(self.get_parameter('drive_away_speed').value)
        self.drive_away_duration = float(self.get_parameter('drive_away_duration').value)

        # Publishers
        self.state_pub = self.create_publisher(String, supervisor_state_topic, 10)
        self.front_enable_pub = self.create_publisher(Bool, front_enable_topic, 10)
        self.dock_enable_pub = self.create_publisher(Bool, dock_enable_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.lift_cmd_pub = self.create_publisher(String, lift_command_topic, 10)

        # Subscribers
        self.front_status_sub = self.create_subscription(String, front_status_topic, self._front_status_cb, 10)
        self.dock_status_sub = self.create_subscription(String, dock_status_topic, self._dock_status_cb, 10)
        self.lift_state_sub = self.create_subscription(String, lift_state_topic, self._lift_state_cb, 10)

        self.front_status = ''
        self.dock_status = ''
        self.lift_state = ''
        self.state = WAITING

        # Flags used by the state machine
        self._lift_cmd_sent = False
        self._lower_cmd_sent = False
        self._user_confirmed = threading.Event()
        self._input_thread = None
        self._drive_away_start_time = None

        self.timer = self.create_timer(0.2, self._tick)

        if auto_start:
            self._set_state(FRONT_APPROACH)
        else:
            self._set_state(WAITING)

        self.get_logger().info('BXU supervisor node started.')

    # --- Publish helpers ------------------------------------------------

    def _publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_zero(self):
        self.cmd_vel_pub.publish(Twist())

    def _set_state(self, new_state: str):
        if new_state != self.state:
            self.get_logger().info(f'{self.state} -> {new_state}')
            self.state = new_state
            # Reset per-state one-shot flags whenever we enter a new state
            self._lift_cmd_sent = False
            self._lower_cmd_sent = False
            self._drive_away_start_time = None
        self._publish_state()

    # --- Subscriptions ---------------------------------------------------

    def _front_status_cb(self, msg: String):
        self.front_status = msg.data

    def _dock_status_cb(self, msg: String):
        self.dock_status = msg.data

    def _lift_state_cb(self, msg: String):
        self.lift_state = msg.data

    # --- User input handling (non-blocking for the ROS executor) --------

    def _start_user_wait(self, prompt: str):
        """Spawn a background thread that blocks on input() and sets an Event when done."""
        self._user_confirmed.clear()

        def _wait():
            input(prompt)
            self._user_confirmed.set()

        self._input_thread = threading.Thread(target=_wait, daemon=True)
        self._input_thread.start()

    # --- Main state machine ----------------------------------------------

    def _tick(self):
        if self.state == WAITING:
            self._publish_bool(self.front_enable_pub, False)
            self._publish_bool(self.dock_enable_pub, False)
            self._publish_zero()
            self._publish_state()
            return

        if self.state == FRONT_APPROACH:
            self._publish_bool(self.front_enable_pub, True)
            self._publish_bool(self.dock_enable_pub, False)
            self._publish_state()

            if self.front_status == 'DONE':
                self._publish_zero()
                self._set_state(DOCKING_ALIGN)
                return

            if self.front_status == 'FAULT':
                self._publish_zero()
                self._set_state(FAULT)
                return

            return

        if self.state == DOCKING_ALIGN:
            self._publish_bool(self.front_enable_pub, False)
            self._publish_bool(self.dock_enable_pub, True)
            self._publish_state()

            if self.dock_status == 'DOCKED':
                self._publish_zero()
                self._set_state(LIFTING)
                return

            if self.dock_status == 'LOST':
                self._publish_zero()
                self._set_state(FAULT)
                return

            return

        if self.state == LIFTING:
            self._publish_bool(self.front_enable_pub, False)
            self._publish_bool(self.dock_enable_pub, False)
            self._publish_zero()
            self._publish_state()

            if not self._lift_cmd_sent:
                self.get_logger().info('Sending LIFT command to battery lift node.')
                self.lift_cmd_pub.publish(String(data='LIFT'))
                self._lift_cmd_sent = True

            if self.lift_state == 'LIFTED':
                self._set_state(WAIT_USER_AFTER_LIFT)
                return

            return

        if self.state == WAIT_USER_AFTER_LIFT:
            self._publish_zero()
            self._publish_state()

            if self._input_thread is None:
                self._start_user_wait('\n✅ Battery LIFTED. Press Enter to lower it... ')

            if self._user_confirmed.is_set():
                self._input_thread = None
                self._set_state(LOWERING)
                return

            return

        if self.state == LOWERING:
            self._publish_zero()
            self._publish_state()

            if not self._lower_cmd_sent:
                self.get_logger().info('Sending LOWER command to battery lift node.')
                self.lift_cmd_pub.publish(String(data='LOWER'))
                self._lower_cmd_sent = True

            if self.lift_state == 'LOWERED':
                self._set_state(WAIT_USER_AFTER_LOWER)
                return

            return

        if self.state == WAIT_USER_AFTER_LOWER:
            self._publish_zero()
            self._publish_state()

            if self._input_thread is None:
                self._start_user_wait('\n✅ Battery LOWERED. Press Enter to drive away... ')

            if self._user_confirmed.is_set():
                self._input_thread = None
                self._set_state(DRIVE_AWAY)
                return

            return

        if self.state == DRIVE_AWAY:
            self._publish_state()

            now = self.get_clock().now().nanoseconds / 1e9
            if self._drive_away_start_time is None:
                self._drive_away_start_time = now
                self.get_logger().info(
                    f'Driving away at vx={self.drive_away_speed} m/s for '
                    f'{self.drive_away_duration}s'
                )

            elapsed = now - self._drive_away_start_time
            if elapsed < self.drive_away_duration:
                twist = Twist()
                twist.linear.x = self.drive_away_speed  # negative = backward
                self.cmd_vel_pub.publish(twist)
            else:
                self._publish_zero()
                self._set_state(DONE)
            return

        if self.state == DONE:
            self._publish_bool(self.front_enable_pub, False)
            self._publish_bool(self.dock_enable_pub, False)
            self._publish_zero()
            self._publish_state()
            return

        if self.state == FAULT:
            self._publish_bool(self.front_enable_pub, False)
            self._publish_bool(self.dock_enable_pub, False)
            self._publish_zero()
            self._publish_state()
            return


def main(args=None):
    rclpy.init(args=args)
    node = BxuSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()