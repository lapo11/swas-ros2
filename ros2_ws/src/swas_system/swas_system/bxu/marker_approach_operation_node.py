"""Additive marker-approach operation coordinator: isolated dry-run control.

Start/Stop enable and disable the already-isolated marker-approach controller.
The controller output remains on its private topic and is never forwarded to
the robot command topics. This node grants no motion authority.

Designed for simulation validation now; the same service contract is intended
for later hardware integration after an authorized gate path exists.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class MarkerApproachOperationNode(Node):
    def __init__(self):
        super().__init__('bxu_marker_approach_operation_node')
        self.declare_parameter(
            'preparation_status_topic',
            '/bxu/operations/marker_approach/status',
        )
        self.declare_parameter(
            'controller_enable_topic',
            '/bxu/controllers/front_markers/enabled',
        )
        self.declare_parameter(
            'operation_state_topic',
            '/bxu/operations/marker_approach/state',
        )
        self.declare_parameter(
            'prepare_service',
            '/bxu/operations/marker_approach/prepare',
        )
        self.declare_parameter(
            'start_service',
            '/bxu/operations/marker_approach/start',
        )
        self.declare_parameter(
            'stop_service',
            '/bxu/operations/marker_approach/stop',
        )
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('preparation_fresh_timeout_s', 3.0)
        self.declare_parameter('max_run_duration_s', 120.0)

        self.preparation_status_topic = self.get_parameter('preparation_status_topic').value
        self.controller_enable_topic = self.get_parameter('controller_enable_topic').value
        self.operation_state_topic = self.get_parameter('operation_state_topic').value
        self.prepare_service = self.get_parameter('prepare_service').value
        self.start_service = self.get_parameter('start_service').value
        self.stop_service = self.get_parameter('stop_service').value
        self.status_period = 1.0 / max(float(self.get_parameter('status_rate_hz').value), 0.1)
        self.preparation_fresh_timeout = float(
            self.get_parameter('preparation_fresh_timeout_s').value)
        self.max_run_duration = float(self.get_parameter('max_run_duration_s').value)

        self.preparation = None
        self.preparation_time = 0.0
        self.state = 'IDLE'
        self.reason = 'Marker approach is idle.'
        self.run_started_time = None

        self.create_subscription(
            String, self.preparation_status_topic, self._preparation_cb, 10)
        self.enable_pub = self.create_publisher(Bool, self.controller_enable_topic, 10)
        self.state_pub = self.create_publisher(String, self.operation_state_topic, 10)
        self.create_service(Trigger, self.prepare_service, self._prepare_cb)
        self.create_service(Trigger, self.start_service, self._start_cb)
        self.create_service(Trigger, self.stop_service, self._stop_cb)
        self.timer = self.create_timer(self.status_period, self._publish_state)
        self.get_logger().info(
            'Marker-approach operation coordinator started; isolated dry-run control only.')

    def _preparation_cb(self, msg):
        try:
            self.preparation = json.loads(msg.data)
            self.preparation_time = time.monotonic()
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warn('Ignoring malformed preparation-status JSON.')

    def _preparation_ready(self):
        recent = (
            self.preparation is not None
            and (time.monotonic() - self.preparation_time) <= self.preparation_fresh_timeout
        )
        return (
            recent
            and self.preparation.get('state') == 'PREPARED'
            and self.preparation.get('motion_authorized') is False
            and self.preparation.get('controllers_started') is False
        )

    def _set_state(self, state, reason):
        if self.state != state or self.reason != reason:
            self.get_logger().info(f'Marker approach: {state} — {reason}')
        self.state = state
        self.reason = reason

    def _prepare_cb(self, request, response):
        if not self._preparation_ready():
            response.success = False
            response.message = (
                'Marker approach is not prepared or preparation status is stale. '
                f'Current state: {self.state}.'
            )
            return response
        response.success = True
        response.message = (
            'Marker approach prepared. Controller remains inactive and disconnected '
            'from robot motion; no automatic operation has started.'
        )
        return response

    def _start_cb(self, request, response):
        if self.state == 'ACTIVE':
            response.success = False
            response.message = 'Marker approach is already active. Stop it before starting again.'
            return response

        if not self._preparation_ready():
            response.success = False
            response.message = (
                'Refusing start: preparation status is absent, stale, not PREPARED, '
                'or reports unexpected authority.'
            )
            return response

        self.run_started_time = time.monotonic()
        self.enable_pub.publish(Bool(data=True))
        self._set_state(
            'ACTIVE',
            'Isolated marker-approach controller enabled. Its output remains '
            'disconnected from robot motion.'
        )
        response.success = True
        response.message = (
            'Marker approach started as an isolated dry run. Controller output is not '
            'connected to a robot command topic.'
        )
        return response

    def _stop_cb(self, request, response):
        self.enable_pub.publish(Bool(data=False))
        self.run_started_time = None
        self._set_state('IDLE', 'Marker approach stopped; isolated controller disabled.')
        response.success = True
        response.message = (
            'Marker approach stopped. The isolated controller was disabled and its '
            'disable handler publishes zero on its private output topic.'
        )
        return response

    def _publish_state(self):
        if self.state == 'ACTIVE':
            elapsed = time.monotonic() - self.run_started_time
            if elapsed > self.max_run_duration:
                self.enable_pub.publish(Bool(data=False))
                self.run_started_time = None
                self._set_state(
                    'FAULT',
                    f'Isolated dry run exceeded {self.max_run_duration:.1f} s; controller disabled.'
                )

        payload = {
            'operation': 'marker_approach',
            'state': self.state,
            'reason': self.reason,
            'controller_enable_topic': self.controller_enable_topic,
            'controller_enabled': self.state == 'ACTIVE',
            'motion_authorized': False,
            'motion_output_connected': False,
            'max_run_duration_s': self.max_run_duration,
            'run_elapsed_s': (
                round(time.monotonic() - self.run_started_time, 3)
                if self.run_started_time is not None else None
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerApproachOperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.enable_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()
