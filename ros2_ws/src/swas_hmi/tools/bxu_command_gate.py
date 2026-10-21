#!/usr/bin/env python3
"""Optional SIMULATION gate. Remap all three demo velocity publishers BEFORE use."""
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from gate_core import GateCore

class SimulationCommandGate(Node):
    def __init__(self):
        super().__init__('bxu_command_gate')
        self.core = GateCore()
        self.output_pub = self.create_publisher(Twist, '/bxu/cmd_vel', 1)
        self.mode_pub = self.create_publisher(String, '/bxu/control/mode', 1)
        self.status_pub = self.create_publisher(String, '/bxu/control/status', 1)
        self.create_subscription(Twist, '/bxu/auto/cmd_vel', lambda m: self.velocity('auto', m), 1)
        self.create_subscription(Twist, '/bxu/teleop/cmd_vel', lambda m: self.velocity('manual', m), 1)
        self.create_subscription(ClockMsg, '/clock', self.clock_cb, qos_profile_sensor_data)
        for key, topic, msgtype in [
            ('mission', '/bxu/supervisor/state', String),
            ('front', '/bxu/front_controller/enabled', Bool),
            ('dock', '/bxu/docking_controller/enabled', Bool),
            ('lift', '/bxu_battery_lift/state', String),
        ]:
            self.create_subscription(msgtype, topic, lambda m, k=key: self.core.observe(k, m.data, time.monotonic()), 1)
        for mode, service in [('AUTO','auto'), ('MANUAL','manual'), ('STOP','stop')]:
            self.create_service(Trigger, '/bxu/control/'+service,
                                lambda req, response, m=mode: self.select(m, response))
        self.last_check = 0.0
        self.conflict = False
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning('SIMULATION ONLY. Verify this is the sole /bxu/cmd_vel publisher. Starts STOPPED.')

    def clock_cb(self, msg):
        self.core.observe('clock', msg.clock.sec + msg.clock.nanosec/1e9, time.monotonic())

    def velocity(self, source, msg):
        self.core.command(source, (msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def select(self, mode, response):
        if mode != 'STOP' and self.conflict:
            response.success, response.message = False, 'Other final velocity publishers detected. Fix remaps first.'
        else:
            response.success, response.message = self.core.select(mode, time.monotonic())
        return response

    def tick(self):
        now = time.monotonic()
        if now-self.last_check > 0.5:
            self.last_check = now
            others = [p for p in self.get_publishers_info_by_topic('/bxu/cmd_vel')
                      if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]
            self.conflict = bool(others)
        if self.conflict:
            # This detects incorrect wiring; it CANNOT neutralize another direct publisher.
            self.core.stop('Conflicting final velocity publisher; fix launch remaps')
        vx, vy, omega = self.core.output(now)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, omega
        self.output_pub.publish(cmd)
        self.mode_pub.publish(String(data=self.core.mode))
        self.status_pub.publish(String(data=self.core.reason))

    def destroy_node(self):
        self.core.stop('Gate shutdown')
        self.output_pub.publish(Twist())
        return super().destroy_node()

def main():
    rclpy.init()
    node = SimulationCommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
