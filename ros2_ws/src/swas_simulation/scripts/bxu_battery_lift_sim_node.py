#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BxuBatteryLiftSimNode(Node):
    """Acknowledge lift commands without moving anything."""

    def __init__(self):
        super().__init__('bxu_battery_lift_sim_node')

        self.declare_parameter(
            'command_topic',
            '/bxu_battery_lift/command',
        )
        self.declare_parameter(
            'state_topic',
            '/bxu_battery_lift/state',
        )

        command_topic = str(
            self.get_parameter('command_topic').value
        )
        state_topic = str(
            self.get_parameter('state_topic').value
        )

        self.state = 'IDLE'

        self.state_pub = self.create_publisher(
            String,
            state_topic,
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            command_topic,
            self._command_cb,
            10,
        )

        self.state_timer = self.create_timer(
            0.5,
            self._publish_state,
        )

        self.get_logger().info(
            'Simulation lift ready: no serial connection, '
            'no movement, immediate command acknowledgement.'
        )

    def _publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def _command_cb(self, msg):
        command = msg.data.strip().upper()

        if command == 'LIFT':
            self.state = 'LIFTED'
        elif command == 'LOWER':
            self.state = 'LOWERED'
        else:
            self.get_logger().warning(
                f'Ignoring unknown lift command: {msg.data!r}'
            )
            return

        self.get_logger().info(
            f'Simulated {command}: reporting {self.state}; '
            'nothing physically moves.'
        )
        self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = BxuBatteryLiftSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()