"""Additive marker-approach preparation status monitor.

MARKER_PREPARATION_OUTPUT_RECOVERY_V2

Reads existing registry and isolated controller status. Publishes one operation
status. It never starts, enables, commands or configures any process.

Freshness rules:
- Registry component report must be recent.
- Isolated controller heartbeat must be recent.
- Controller must be inactive.
- No nonzero isolated controller output may have been observed.
"""
import json
import time


import rclpy
from rclpy.node import Node
from std_msgs.msg import String


from geometry_msgs.msg import Twist



class MarkerApproachPreparationNode(Node):
    def __init__(self):
        super().__init__('bxu_marker_approach_preparation_node')
        self.declare_parameter('required_components', ['front_markers', 'docking'])
        self.declare_parameter('registry_status_topic', '/bxu/runtime/components/status')
        self.declare_parameter('controller_status_topic', '/bxu/controllers/front_markers/status')
        self.declare_parameter('controller_output_topic', '/bxu/controllers/front_markers/cmd_vel')
        self.declare_parameter('status_topic', '/bxu/operations/marker_approach/status')
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('registry_fresh_timeout_s', 2.0)
        self.declare_parameter('controller_fresh_timeout_s', 3.0)
        self.declare_parameter('controller_output_zero_settle_s', 1.0)


        self.required_components = list(self.get_parameter('required_components').value)
        self.registry_status_topic = self.get_parameter('registry_status_topic').value
        self.controller_status_topic = self.get_parameter('controller_status_topic').value
        self.controller_output_topic = self.get_parameter('controller_output_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.status_period = 1.0 / max(float(self.get_parameter('status_rate_hz').value), 0.1)
        self.registry_fresh_timeout = float(self.get_parameter('registry_fresh_timeout_s').value)
        self.controller_fresh_timeout = float(self.get_parameter('controller_fresh_timeout_s').value)
        self.output_zero_settle = float(
            self.get_parameter('controller_output_zero_settle_s').value)


        self.components = {}
        self.components_time = 0.0
        self.controller_status = None
        self.controller_status_time = 0.0
        self.last_nonzero_output_time = None
        self.last_zero_output_time = None


        self.create_subscription(
            String, self.registry_status_topic, self._components_cb, 10)
        self.create_subscription(
            String, self.controller_status_topic, self._controller_status_cb, 10)
        self.create_subscription(
            Twist, self.controller_output_topic, self._controller_output_cb, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(self.status_period, self._publish_status)
        self.get_logger().info(
            'Marker-approach preparation monitor started; observation only, no control.')


    def _components_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warn('Ignoring malformed component-status JSON.')
            return
        components = payload.get('components', {})
        if not isinstance(components, dict):
            self.get_logger().warn('Ignoring component-status payload without a components map.')
            return
        self.components = {
            name: {
                'state': item.get('state'),
                'process_running': bool(item.get('process_running', False)),
                'load_allowed': bool(item.get('load_allowed', False)),
                'unload_allowed': bool(item.get('unload_allowed', False)),
            }
            for name, item in components.items()
            if isinstance(item, dict)
        }
        self.components_time = time.monotonic()


    def _controller_status_cb(self, msg):
        self.controller_status = msg.data
        self.controller_status_time = time.monotonic()


    def _controller_output_cb(self, msg):
        nonzero = any(abs(value) > 1e-9 for value in (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z))
        now = time.monotonic()
        if nonzero:
            self.last_nonzero_output_time = now
        else:
            self.last_zero_output_time = now


    def _publish_status(self):
        now = time.monotonic()
        registry_recent = (
            bool(self.components)
            and (now - self.components_time) <= self.registry_fresh_timeout
        )
        controller_recent = (
            self.controller_status is not None
            and (now - self.controller_status_time) <= self.controller_fresh_timeout
        )
        controller_inactive = self.controller_status == 'INACTIVE'

        missing = [name for name in self.required_components if name not in self.components]
        not_ready = [
            name for name in self.required_components
            if self.components.get(name, {}).get('state') != 'READY'
        ]

        if not registry_recent:
            if not self.components:
                state, reason = 'WAITING_PERCEPTION', 'No registry component report received.'
            else:
                state, reason = 'STALE_REGISTRY', 'Registry component report is stale or absent.'
        elif missing:
            state, reason = 'UNAVAILABLE', 'Required registry components absent: ' + ', '.join(missing)
        elif not_ready:
            state, reason = 'WAITING_PERCEPTION', 'Components not READY: ' + ', '.join(not_ready)
        elif not controller_recent:
            if self.controller_status is None:
                state, reason = 'WAITING_CONTROLLER', 'Isolated controller status not received.'
            else:
                state, reason = 'STALE_CONTROLLER', 'Isolated controller status is stale.'
        elif not controller_inactive:
            state, reason = 'BLOCKED', 'Controller reports ' + str(self.controller_status) + '; preparation must remain inactive.'
        elif self.last_nonzero_output_time is not None:
            # A disabled controller sends a one-time zero acknowledgement;
            # do not require that acknowledgement to keep arriving.
            zero_acknowledged = (
                self.last_zero_output_time is not None
                and self.last_zero_output_time >= self.last_nonzero_output_time
            )
            zero_settled = (
                self.last_zero_output_time is not None
                and (now - self.last_nonzero_output_time) >= self.output_zero_settle
            )
            if controller_inactive and zero_acknowledged and zero_settled:
                state, reason = 'PREPARED', (
                    'Required perception READY; isolated controller inactive and '
                    f'zero output settled for {self.output_zero_settle:.1f} s.'
                )
            else:
                state, reason = 'BLOCKED', (
                    'Nonzero isolated controller output was observed; '
                    'inactive controller and settled zero acknowledgement are required.'
                )
        else:
            state, reason = 'PREPARED', 'Required perception READY and isolated controller present but inactive.'


        payload = {
            'operation': 'marker_approach',
            'state': state,
            'reason': reason,
            'required_components': self.required_components,
            'component_states': {
                name: self.components.get(name, {}).get('state')
                for name in self.required_components
            },
            'registry_status_age_s': round(max(0.0, now - self.components_time), 3) if self.components_time else None,
            'controller_status': self.controller_status,
            'controller_status_age_s': round(max(0.0, now - self.controller_status_time), 3) if self.controller_status_time else None,
            'controller_output_topic': self.controller_output_topic,
            'controller_output_observed': self.last_nonzero_output_time is not None,
            'motion_authorized': False,
            'controllers_started': False,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MarkerApproachPreparationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
