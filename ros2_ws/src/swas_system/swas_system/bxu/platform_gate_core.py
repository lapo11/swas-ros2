"""Manual-only SIMULATION authority with an explicit bounded operation mode."""
import math

ZERO = (0.0, 0.0, 0.0)
SETTLED_LIFT = ('IDLE', 'LIFTED', 'LOWERED')

class PlatformGateCore:
    def __init__(self):
        self.mode = 'STOP'
        self.reason = 'Initial stop; manual-only simulation platform'
        self.clock = None
        self.clock_at = None
        self.clock_changed_at = None
        self.odom = None
        self.odom_at = None
        self.stationary_since = None
        self.lift = None
        self.lift_at = None
        self.graph = {}
        self.graph_at = None
        self.operation = None
        self.operation_at = None
        self.vector = ZERO
        self.command_at = None
        self.zero_primed = False
        self.has_moved_request = False

    @staticmethod
    def fresh(at, now, maximum):
        return at is not None and 0 <= now-at <= maximum

    def stop(self, reason='Stop requested'):
        self.mode, self.reason = 'STOP', reason
        self.vector = ZERO
        self.command_at = None
        self.zero_primed = False
        self.has_moved_request = False

    def observe_clock(self, value, now):
        if not math.isfinite(value):
            self.clock_at = None
            self.stop('Invalid simulation clock')
            return
        if self.clock is not None and value < self.clock:
            self.stop('Simulation time moved backwards; explicit manual selection required')
            self.stationary_since = None
        if value != self.clock:
            self.clock_changed_at = now
        self.clock, self.clock_at = value, now

    def observe_odom(self, vx, vy, yaw_rate, now):
        if not all(math.isfinite(v) for v in (vx, vy, yaw_rate)):
            self.odom_at = None
            self.stationary_since = None
            self.stop('Invalid odometry')
            return
        if not self.fresh(self.odom_at, now, 0.8):
            self.stationary_since = None
        self.odom = (vx, vy, yaw_rate)
        self.odom_at = now
        if math.hypot(vx, vy) <= 0.01 and abs(yaw_rate) <= 0.02:
            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None

    def observe_lift(self, state, now):
        self.lift, self.lift_at = state, now

    def observe_graph(self, graph, now):
        self.graph, self.graph_at = dict(graph), now

    def observe_operation(self, operation, now):
        self.operation, self.operation_at = operation, now

    def operation_active(self, now):
        return (
            self.fresh(self.operation_at, now, 1.0)
            and isinstance(self.operation, dict)
            and self.operation.get('operation') == 'marker_approach'
            and self.operation.get('state') == 'ACTIVE'
            and self.operation.get('motion_authorized') is False
            and self.operation.get('motion_output_connected') is False
        )

    def blockers(self, now, entry=False, mode=None):
        mode = self.mode if mode is None else mode
        reasons = []
        if not self.fresh(self.clock_at, now, 0.6) or not self.fresh(self.clock_changed_at, now, 0.6):
            reasons.append('Simulation clock absent, stale or paused')
        if not self.fresh(self.odom_at, now, 0.8):
            reasons.append('Recent simulated odometry required')
        if not self.fresh(self.lift_at, now, 2.0) or self.lift not in SETTLED_LIFT:
            reasons.append('Settled simulated lift acknowledgement required')
        if not self.fresh(self.graph_at, now, 0.8):
            reasons.append('ROS graph check is stale')
        else:
            g = self.graph
            if g.get('final_publishers') != 1 or g.get('own_final_publisher') is not True:
                reasons.append('This gate must be the sole final velocity publisher')
            if g.get('expected_sink_present') is not True:
                reasons.append('Expected Gazebo bridge subscriber is not discovered')
            if g.get('auto_publishers') != 0:
                reasons.append('Autonomous velocity publishers are not allowed')
            if g.get('supervisor_publishers') != 0:
                reasons.append('Mission supervisor publishers are not allowed')
            if g.get('lift_command_publishers') != 0:
                reasons.append('External lift command publishers are not allowed')
            if g.get('lift_state_publishers') != 1:
                reasons.append('Exactly one simulated lift-state publisher is required')
            if g.get('manual_publishers') not in (0, 1):
                reasons.append('More than one teleop publishing endpoint discovered')
            if mode == 'OPERATION':
                if g.get('operation_publishers') != 1:
                    reasons.append('Exactly one authorized marker-approach relay publisher is required')
                if g.get('marker_approach_enable_publishers') != 1:
                    reasons.append('Exactly one marker-approach controller-enable publisher is required')
                if g.get('enable_publishers') != 0:
                    reasons.append('Legacy controller-enable publishers are not allowed during marker-approach operation')
            else:
                if g.get('operation_publishers') != 1:
                    reasons.append('Exactly one authorized marker-approach relay publisher is required')
                if g.get('marker_approach_enable_publishers') != 1:
                    reasons.append('Exactly one marker-approach controller-enable publisher is required')
                if g.get('enable_publishers') != 0:
                    reasons.append('Legacy controller-enable publishers are not allowed in this manual-only milestone')
        if entry and (self.stationary_since is None or now-self.stationary_since < 0.25):
            reasons.append('Require 0.25 seconds of stationary simulated odometry before entry')
        return reasons

    def select_manual(self, now):
        reasons = self.blockers(now, entry=self.mode != 'MANUAL', mode='MANUAL')
        if reasons:
            self.stop('Manual refused: '+'; '.join(reasons))
            return False, self.reason
        if self.mode == 'MANUAL':
            return True, 'Manual already selected; no command state reset'
        self.mode, self.reason = 'MANUAL', 'Manual selected; zero frame required before first jog'
        self.vector, self.command_at = ZERO, None
        self.zero_primed, self.has_moved_request = False, False
        return True, self.reason

    def select_operation(self, now):
        if not self.operation_active(now):
            self.stop('Operation authority refused: marker approach is not ACTIVE and fresh')
            return False, self.reason
        reasons = self.blockers(now, entry=True, mode='OPERATION')
        if reasons:
            self.stop('Operation authority refused: '+'; '.join(reasons))
            return False, self.reason
        self.mode, self.reason = 'OPERATION', 'Marker-approach operation authorized; zero frame required'
        self.vector, self.command_at = ZERO, None
        self.zero_primed, self.has_moved_request = False, False
        return True, self.reason

    def command(self, values, now):
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            self.stop('Invalid teleop velocity')
            return False
        if self.mode != 'MANUAL':
            return False
        values = tuple(float(v) for v in values)
        if any(values) and not self.zero_primed:
            self.reason = 'Initial nonzero jog refused; release to send zero first'
            return False
        if not any(values):
            self.zero_primed = True
        self.vector, self.command_at = values, now
        self.has_moved_request = self.has_moved_request or any(values)
        return True

    def command_operation(self, values, now):
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            self.stop('Invalid marker-approach operation velocity')
            return False
        if self.mode != 'OPERATION':
            return False
        self.vector, self.command_at = tuple(float(v) for v in values), now
        return True

    def output(self, now):
        if self.mode == 'MANUAL':
            reasons = self.blockers(now, mode='MANUAL')
            if reasons:
                self.stop('Manual interlock lost: '+'; '.join(reasons))
                return ZERO
            if self.has_moved_request and not self.fresh(self.command_at, now, 0.35):
                self.stop('Teleop stream expired; select manual again before re-arming')
                return ZERO
            if not self.fresh(self.command_at, now, 0.35):
                return ZERO
            x,y,w = self.vector
            length = math.hypot(x,y)
            if length > 0.20:
                x,y = x*0.20/length,y*0.20/length
            self.reason = 'Manual velocity forwarding' if any((x,y,w)) else 'Manual selected; zero requested'
            return x,y,max(-0.15,min(w,0.15))

        if self.mode == 'OPERATION':
            if not self.operation_active(now):
                self.stop('Marker-approach operation authority lost')
                return ZERO
            reasons = self.blockers(now, mode='OPERATION')
            if reasons:
                self.stop('Operation interlock lost: '+'; '.join(reasons))
                return ZERO
            if not self.fresh(self.command_at, now, 0.35):
                return ZERO
            x,y,w = self.vector
            length = math.hypot(x,y)
            if length > 0.20:
                x,y = x*0.20/length,y*0.20/length
            self.reason = 'Marker-approach operation velocity forwarding' if any((x,y,w)) else 'Marker-approach operation authorized; zero requested'
            return x,y,max(-0.15,min(w,0.15))

        return ZERO
