#!/usr/bin/env python3
"""Opt-in manual-only SIMULATION gate with an explicit bounded operation mode."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swas_system.bxu.platform_gate_core import PlatformGateCore


class PlatformGate(Node):
    def __init__(self):
        super().__init__('bxu_command_gate')
        self.declare_parameter('expected_sink_node', 'ros_gz_bridge')
        self.sink_name = str(self.get_parameter('expected_sink_node').value)
        self.core = PlatformGateCore()
        self.out = self.create_publisher(Twist, '/bxu/cmd_vel', 1)
        self.mode_pub = self.create_publisher(String, '/bxu/control/mode', 1)
        self.reason_pub = self.create_publisher(String, '/bxu/control/status', 1)
        self.platform_pub = self.create_publisher(String, '/bxu/platform/status', 1)
        self.create_subscription(Twist, '/bxu/teleop/cmd_vel', self.teleop, 1)
        self.create_subscription(
            Twist,
            '/bxu/operations/marker_approach/authorized_cmd_vel',
            self.operation_velocity,
            1,
        )
        self.create_subscription(ClockMsg, '/clock', self.clock_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/bxu/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/bxu/lift/state', lambda m: self.core.observe_lift(m.data, time.monotonic()), 1)
        self.create_subscription(
            String,
            '/bxu/operations/marker_approach/state',
            self.operation_state_cb,
            1,
        )
        for name in ('/bxu/control/manual', '/bxu/platform/manual'):
            self.create_service(Trigger, name, self.manual)
        self.create_service(Trigger, '/bxu/control/stop', self.stop_cb)
        self.create_service(Trigger, '/bxu/control/auto', self.auto_cb)
        self.create_service(
            Trigger,
            '/bxu/operations/marker_approach/authorize',
            self.authorize_cb,
        )
        self.last_graph, self.last_status = -float('inf'), -float('inf')
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(
            'MANUAL-ONLY SIMULATION PLATFORM with explicit bounded OPERATION mode. '
            'Legacy AUTO and lift actuation remain unavailable.')

    def clock_cb(self, msg):
        self.core.observe_clock(msg.clock.sec+msg.clock.nanosec/1e9, time.monotonic())

    def odom_cb(self, msg):
        t = msg.twist.twist
        self.core.observe_odom(t.linear.x,t.linear.y,t.angular.z,time.monotonic())

    def operation_state_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warn('Ignoring malformed marker-approach operation state.')
            return
        if payload.get('operation') != 'marker_approach':
            return
        self.core.observe_operation(payload, time.monotonic())

    def teleop(self, msg):
        if any(v != 0.0 for v in (msg.linear.z,msg.angular.x,msg.angular.y)):
            self.core.stop('Unsupported teleop axes; only x/y/yaw are allowed')
            return
        self.core.command((msg.linear.x,msg.linear.y,msg.angular.z),time.monotonic())

    def operation_velocity(self, msg):
        if any(v != 0.0 for v in (msg.linear.z,msg.angular.x,msg.angular.y)):
            self.core.stop('Unsupported operation velocity axes; only x/y/yaw are allowed')
            return
        self.core.command_operation(
            (msg.linear.x,msg.linear.y,msg.angular.z), time.monotonic())

    def graph_check(self):
        now = time.monotonic()
        final = self.get_publishers_info_by_topic('/bxu/cmd_vel')
        sinks = self.get_subscriptions_info_by_topic('/bxu/cmd_vel')
        graph = {
            'final_publishers':len(final),
            'own_final_publisher':len(final)==1 and final[0].node_name==self.get_name() and final[0].node_namespace==self.get_namespace(),
            'expected_sink_present':any(p.node_name==self.sink_name for p in sinks),
            'auto_publishers':len(self.get_publishers_info_by_topic('/bxu/auto/cmd_vel')),
            'supervisor_publishers':len(self.get_publishers_info_by_topic('/bxu/supervisor/state')),
            'enable_publishers':sum(len(self.get_publishers_info_by_topic(topic)) for topic in ('/bxu/front_controller/enabled','/bxu/docking_controller/enabled')),
            'marker_approach_enable_publishers':len(self.get_publishers_info_by_topic('/bxu/controllers/front_markers/enabled')),
            'operation_publishers':len(self.get_publishers_info_by_topic('/bxu/operations/marker_approach/authorized_cmd_vel')),
            'lift_command_publishers':len(self.get_publishers_info_by_topic('/bxu/lift/command')),
            'lift_state_publishers':len(self.get_publishers_info_by_topic('/bxu/lift/state')),
            'manual_publishers':len(self.get_publishers_info_by_topic('/bxu/teleop/cmd_vel')),
        }
        self.core.observe_graph(graph,now)
        self.last_graph = now

    def manual(self, request, response):
        try:
            self.graph_check()
            response.success,response.message = self.core.select_manual(time.monotonic())
        except Exception as error:
            self.core.stop('Graph check failed')
            response.success,response.message = False,'Manual refused: '+str(error)
        return response

    def authorize_cb(self, request, response):
        try:
            self.graph_check()
            response.success,response.message = self.core.select_operation(time.monotonic())
        except Exception as error:
            self.core.stop('Graph check failed')
            response.success,response.message = False,'Operation authorization refused: '+str(error)
        return response

    def stop_cb(self, request, response):
        self.core.stop('Operator stop requested')
        self.out.publish(Twist())
        response.success = True
        response.message = 'Zero velocity requested; manual authority cleared. Not a physical stop acknowledgement.'
        return response

    def auto_cb(self, request, response):
        response.success = False
        response.message = 'Legacy AUTO is unavailable. Use the explicit marker-approach operation authorization.'
        return response

    def tick(self):
        now = time.monotonic()
        try:
            if now-self.last_graph >= 0.2:
                self.graph_check()
        except Exception as error:
            self.core.stop('ROS graph check failed: '+str(error))
        now = time.monotonic()
        x,y,w = self.core.output(now)
        msg = Twist()
        msg.linear.x,msg.linear.y,msg.angular.z = x,y,w
        self.out.publish(msg)
        self.mode_pub.publish(String(data=self.core.mode))
        self.reason_pub.publish(String(data=self.core.reason))
        if now-self.last_status >= 0.2:
            status = {
                'platform':'manual_only_simulation',
                'mode':self.core.mode,
                'reason':self.core.reason,
                'blockers':self.core.blockers(now,entry=self.core.mode not in ('MANUAL','OPERATION')),
                'graph':self.core.graph,
                'auto_available':False,
                'lift_control_available':False,
                'lift_state':self.core.lift,
                'lift_feedback':'immediate simulated acknowledgement; no physical motion',
                'linear_limit_m_s':0.20,
                'yaw_limit_rad_s':0.15,
                'fixed_jog_duration_limit':None,
                'command_timeout_s':0.35,
                'single_operator_required':True,
            }
            self.platform_pub.publish(String(data=json.dumps(status)))
            self.last_status = now

    def destroy_node(self):
        self.core.stop('Platform gate shutdown')
        for _ in range(3):
            self.out.publish(Twist())
            time.sleep(0.02)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlatformGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
