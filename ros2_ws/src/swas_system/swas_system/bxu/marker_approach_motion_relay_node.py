"""Authorized marker-approach motion relay.

Forwards the isolated controller velocity to the gate AUTO input only while
the marker-approach operation is ACTIVE. It is not a final-motion publisher.
"""
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MarkerApproachMotionRelayNode(Node):
    def __init__(self):
        super().__init__('bxu_marker_approach_motion_relay_node')
        self.declare_parameter(
            'controller_output_topic',
            '/bxu/controllers/front_markers/cmd_vel',
        )
        self.declare_parameter(
            'operation_state_topic',
            '/bxu/operations/marker_approach/state',
        )
        self.declare_parameter(
            'gate_auto_input_topic',
            '/bxu/operations/marker_approach/authorized_cmd_vel',
        )
        self.declare_parameter('operation_fresh_timeout_s', 1.0)

        self.controller_output_topic = self.get_parameter('controller_output_topic').value
        self.operation_state_topic = self.get_parameter('operation_state_topic').value
        self.gate_auto_input_topic = self.get_parameter('gate_auto_input_topic').value
        self.fresh_timeout = float(self.get_parameter('operation_fresh_timeout_s').value)

        self.operation = None
        self.operation_time = 0.0
        self.forwarding = False

        self.create_subscription(
            Twist, self.controller_output_topic, self.controller_cb, 10)
        self.create_subscription(
            String, self.operation_state_topic, self.operation_cb, 10)
        self.gate_pub = self.create_publisher(Twist, self.gate_auto_input_topic, 1)
        self.get_logger().info(
            'Marker-approach motion relay started; forwarding only while operation is ACTIVE.')

    def operation_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warn('Ignoring malformed operation-state JSON.')
            return
        if payload.get('operation') != 'marker_approach':
            return
        self.operation = payload
        self.operation_time = time.monotonic()
        self._update_forwarding()

    def active(self):
        return (
            self.operation is not None
            and self.operation.get('state') == 'ACTIVE'
            and self.operation.get('motion_authorized') is False
            and (time.monotonic() - self.operation_time) <= self.fresh_timeout
        )

    def _update_forwarding(self):
        active = self.active()
        if self.forwarding and not active:
            self.gate_pub.publish(Twist())
            self.get_logger().info('Operation no longer active; zero sent to gate AUTO input.')
        self.forwarding = active

    def controller_cb(self, msg):
        if not self.active():
            if self.forwarding:
                self.gate_pub.publish(Twist())
                self.forwarding = False
            return
        self.forwarding = True
        self.gate_pub.publish(msg)

    def destroy_node(self):
        self.gate_pub.publish(Twist())
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerApproachMotionRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
