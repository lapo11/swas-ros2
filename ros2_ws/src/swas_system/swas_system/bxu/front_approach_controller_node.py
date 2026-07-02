import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from swas_msgs.msg import BxuAlignmentOutput

SEARCH = 'SEARCH'
COARSE_PD = 'COARSE_PD'
FINE_YAW = 'FINE_YAW'
FINE_LATERAL = 'FINE_LATERAL'
APPROACH_FINE = 'APPROACH_FINE'
BLIND_FORWARD = 'BLIND_FORWARD'
DONE = 'DONE'


class BxuFrontApproachControllerNode(Node):
    def __init__(self):
        super().__init__('bxu_front_approach_controller_node')
        self._declare_params()
        self._load_params()

        self.pub = self.create_publisher(Twist, '/bxu/cmd_vel', 10)
        self.sub = self.create_subscription(
            BxuAlignmentOutput, '/bxu/front_vision/alignment',
            self._alignment_cb, 10
        )

        self._latest_msg = None
        self._latest_time = 0.0

        self.state = SEARCH
        self._last_fine_time = None

        # confirm-debounce
        self._confirm_key = None
        self._confirm_count = 0

        # pulse state
        self._pulse_active = False
        self._pulse_end = 0.0
        self._pulse_cmd = (0.0, 0.0, 0.0)

        # coarse PD derivative filter state
        self._prev_yaw = None
        self._prev_lat = None
        self._dyaw = 0.0
        self._dlat = 0.0

        self._done_logged = False

        self.timer = self.create_timer(self.action_dt, self._tick)
        self.get_logger().info('BXU front approach controller started — state=SEARCH')

    # ── Params ────────────────────────────────────────────────
    def _declare_params(self):
        defaults = {
            'rotate_search_speed': 0.30,
            'coarse_kp_yaw': 0.008, 'coarse_kd_yaw': 0.002,
            'coarse_kp_lat': 0.0030, 'coarse_kd_lat': 0.0006,
            'coarse_kp_dist': 0.010,
            'coarse_max_omega': 0.25, 'coarse_max_vy': 0.08, 'coarse_max_vx': 0.10,
            'coarse_min_omega': 0.07, 'coarse_min_vy': 0.04, 'coarse_min_vx': 0.04,
            'fine_rotate_speed': 0.10, 'fine_slide_speed': 0.05, 'fine_forward_speed': 0.05,
            'fine_rotate_pulse_s': 0.18, 'fine_slide_pulse_s': 0.16, 'fine_forward_pulse_s': 0.6,
            'blind_forward_speed': 0.15, 'blind_forward_pulse_s': 1.4667,
            'yaw_fine_deg': 0.7, 'lat_fine_cm': 1.0,
            'yaw_approach_deg': 3.5, 'lat_approach_cm': 3.0,
            'near_visible_distance_cm': 35.0, 'blind_entry_distance_cm': 23.0,
            'action_dt': 0.05, 'obs_stale_timeout_s': 0.35,
            'fine_memory_timeout_s': 2.0, 'confirm_frames': 2,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

    def _load_params(self):
        g = lambda k: self.get_parameter(k).value
        self.rotate_search_speed = g('rotate_search_speed')
        self.coarse_kp_yaw = g('coarse_kp_yaw'); self.coarse_kd_yaw = g('coarse_kd_yaw')
        self.coarse_kp_lat = g('coarse_kp_lat'); self.coarse_kd_lat = g('coarse_kd_lat')
        self.coarse_kp_dist = g('coarse_kp_dist')
        self.coarse_max_omega = g('coarse_max_omega')
        self.coarse_max_vy = g('coarse_max_vy')
        self.coarse_max_vx = g('coarse_max_vx')
        self.coarse_min_omega = g('coarse_min_omega')
        self.coarse_min_vy = g('coarse_min_vy')
        self.coarse_min_vx = g('coarse_min_vx')
        self.fine_rotate_speed = g('fine_rotate_speed')
        self.fine_slide_speed = g('fine_slide_speed')
        self.fine_forward_speed = g('fine_forward_speed')
        self.fine_rotate_pulse_s = g('fine_rotate_pulse_s')
        self.fine_slide_pulse_s = g('fine_slide_pulse_s')
        self.fine_forward_pulse_s = g('fine_forward_pulse_s')
        self.blind_forward_speed = g('blind_forward_speed')
        self.blind_forward_pulse_s = g('blind_forward_pulse_s')
        self.yaw_fine_deg = g('yaw_fine_deg')
        self.lat_fine_cm = g('lat_fine_cm')
        self.yaw_approach_deg = g('yaw_approach_deg')
        self.lat_approach_cm = g('lat_approach_cm')
        self.near_visible_distance_cm = g('near_visible_distance_cm')
        self.blind_entry_distance_cm = g('blind_entry_distance_cm')
        self.action_dt = g('action_dt')
        self.obs_stale_timeout_s = g('obs_stale_timeout_s')
        self.fine_memory_timeout_s = g('fine_memory_timeout_s')
        self.confirm_frames = g('confirm_frames')

    # ── Subscription ─────────────────────────────────────────
    def _alignment_cb(self, msg: BxuAlignmentOutput):
        self._latest_msg = msg
        self._latest_time = time.time()

    def _get_obs(self):
        """Returns (valid, num_markers, yaw_deg, lat_cm, dist_cm) or (False, ...) if stale/missing."""
        if self._latest_msg is None:
            return False, 0, None, None, None
        if (time.time() - self._latest_time) > self.obs_stale_timeout_s:
            return False, 0, None, None, None
        m = self._latest_msg
        if not m.markers_detected or m.num_markers_detected == 0:
            return False, 0, None, None, None
        yaw_deg = math.degrees(m.yaw_error_rad)
        lat_cm = m.lateral_error_m * 100.0
        dist_cm = m.distance_m * 100.0 if m.distance_m >= 0.0 else None
        return True, m.num_markers_detected, yaw_deg, lat_cm, dist_cm

    def _fine_aligned(self, yaw_deg, lat_cm, dist_cm):
        if yaw_deg is None or lat_cm is None or dist_cm is None:
            return False
        return (abs(yaw_deg) <= self.yaw_fine_deg
                and abs(lat_cm) <= self.lat_fine_cm
                and dist_cm <= self.near_visible_distance_cm)

    def _recent_fine(self):
        if self._last_fine_time is None:
            return False
        return (time.time() - self._last_fine_time) <= self.fine_memory_timeout_s

    # ── Publish helpers ──────────────────────────────────────
    def _publish(self, vx, vy, omega):
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = omega
        self.pub.publish(t)

    def _publish_zero(self):
        self._publish(0.0, 0.0, 0.0)

    # ── Discrete decision table (SEARCH / FINE_* / BLIND_FORWARD) ──
    def _decide(self, state, valid, num_markers, yaw_deg, lat_cm, dist_cm):
        if state == SEARCH:
            if not valid or num_markers < 2:
                return SEARCH, 'ROTATE_SEARCH'
            if self._fine_aligned(yaw_deg, lat_cm, dist_cm):
                return FINE_YAW, 'HOLD'
            if dist_cm is not None and dist_cm <= self.near_visible_distance_cm:
                return FINE_YAW, 'HOLD'
            return COARSE_PD, 'HOLD'

        if state == FINE_YAW:
            if not valid:
                return (BLIND_FORWARD, 'HOLD') if self._recent_fine() else (SEARCH, 'HOLD')
            if abs(yaw_deg) > self.yaw_fine_deg:
                return FINE_YAW, ('ROTATE_RIGHT_FINE' if yaw_deg > 0 else 'ROTATE_LEFT_FINE')
            return FINE_LATERAL, 'HOLD'

        if state == FINE_LATERAL:
            if not valid:
                return (BLIND_FORWARD, 'HOLD') if self._recent_fine() else (SEARCH, 'HOLD')
            if abs(yaw_deg) > self.yaw_fine_deg:
                return FINE_YAW, 'HOLD'
            if abs(lat_cm) > self.lat_fine_cm:
                return FINE_LATERAL, ('SLIDE_RIGHT_FINE' if lat_cm > 0 else 'SLIDE_LEFT_FINE')
            return APPROACH_FINE, 'HOLD'

        if state == APPROACH_FINE:
            if not valid:
                return (BLIND_FORWARD, 'HOLD') if self._recent_fine() else (SEARCH, 'HOLD')
            if dist_cm is None:
                return SEARCH, 'HOLD'
            if abs(yaw_deg) > self.yaw_fine_deg:
                return FINE_YAW, 'HOLD'
            if abs(lat_cm) > self.lat_fine_cm:
                return FINE_LATERAL, 'HOLD'
            if dist_cm <= self.blind_entry_distance_cm and self._recent_fine():
                return BLIND_FORWARD, 'HOLD'
            return APPROACH_FINE, 'FORWARD_FINE'

        if state == BLIND_FORWARD:
            return DONE, 'BLIND_FORWARD'

        return DONE, 'HOLD'

    def _start_pulse(self, vx, vy, omega, duration_s):
        self._pulse_active = True
        self._pulse_cmd = (vx, vy, omega)
        self._pulse_end = time.time() + duration_s

    def _execute_action(self, action):
        if action == 'HOLD':
            self._publish_zero()
        elif action == 'ROTATE_SEARCH':
            self._publish(0.0, 0.0, self.rotate_search_speed)
        elif action == 'ROTATE_LEFT_FINE':
            self._start_pulse(0.0, 0.0, +self.fine_rotate_speed, self.fine_rotate_pulse_s)
        elif action == 'ROTATE_RIGHT_FINE':
            self._start_pulse(0.0, 0.0, -self.fine_rotate_speed, self.fine_rotate_pulse_s)
        elif action == 'SLIDE_LEFT_FINE':
            self._start_pulse(0.0, +self.fine_slide_speed, 0.0, self.fine_slide_pulse_s)
        elif action == 'SLIDE_RIGHT_FINE':
            self._start_pulse(0.0, -self.fine_slide_speed, 0.0, self.fine_slide_pulse_s)
        elif action == 'FORWARD_FINE':
            self._start_pulse(+self.fine_forward_speed, 0.0, 0.0, self.fine_forward_pulse_s)
        elif action == 'BLIND_FORWARD':
            self._start_pulse(+self.blind_forward_speed, 0.0, 0.0, self.blind_forward_pulse_s)

    # ── Coarse PD (continuous, no debounce) ─────────────────────
    def _tick_coarse_pd(self, valid, num_markers, yaw_deg, lat_cm, dist_cm):
        if not valid:
            self._publish_zero()
            self._reset_coarse_pd()
            self.state = SEARCH
            self.get_logger().info('COARSE_PD: markers lost -> SEARCH')
            return

        if (abs(yaw_deg) <= self.yaw_approach_deg
                and abs(lat_cm) <= self.lat_approach_cm
                and dist_cm is not None and dist_cm <= self.near_visible_distance_cm):
            self._publish_zero()
            self._reset_coarse_pd()
            self.state = FINE_YAW
            self.get_logger().info('COARSE_PD converged -> FINE_YAW')
            return

        if dist_cm is None:
            self._publish_zero()
            return

        if self._prev_yaw is None:
            raw_dyaw = 0.0
        else:
            raw_dyaw = (yaw_deg - self._prev_yaw) / self.action_dt
        self._dyaw = 0.8 * self._dyaw + 0.2 * raw_dyaw
        self._prev_yaw = yaw_deg

        if self._prev_lat is None:
            raw_dlat = 0.0
        else:
            raw_dlat = (lat_cm - self._prev_lat) / self.action_dt
        self._dlat = 0.8 * self._dlat + 0.2 * raw_dlat
        self._prev_lat = lat_cm

        dist_err = dist_cm - self.near_visible_distance_cm

        omega = max(min(-(self.coarse_kp_yaw * yaw_deg + self.coarse_kd_yaw * self._dyaw),
                         self.coarse_max_omega), -self.coarse_max_omega)
        vy = max(min(-(self.coarse_kp_lat * lat_cm + self.coarse_kd_lat * self._dlat),
                      self.coarse_max_vy), -self.coarse_max_vy)
        vx = max(min(self.coarse_kp_dist * dist_err, self.coarse_max_vx), 0.0)

        if 0 < abs(omega) < self.coarse_min_omega:
            omega = self.coarse_min_omega if omega > 0 else -self.coarse_min_omega
        if 0 < abs(vy) < self.coarse_min_vy:
            vy = self.coarse_min_vy if vy > 0 else -self.coarse_min_vy
        if 0 < vx < self.coarse_min_vx:
            vx = self.coarse_min_vx

        self._publish(vx, vy, omega)

    def _reset_coarse_pd(self):
        self._prev_yaw = None
        self._prev_lat = None
        self._dyaw = 0.0
        self._dlat = 0.0

    # ── Main timer tick ──────────────────────────────────────
    def _tick(self):
        valid, num_markers, yaw_deg, lat_cm, dist_cm = self._get_obs()

        if valid and self._fine_aligned(yaw_deg, lat_cm, dist_cm):
            self._last_fine_time = time.time()

        # active pulse takes priority
        if self._pulse_active:
            if time.time() < self._pulse_end:
                self._publish(*self._pulse_cmd)
                return
            self._pulse_active = False
            self._publish_zero()
            return

        if self.state == COARSE_PD:
            self._tick_coarse_pd(valid, num_markers, yaw_deg, lat_cm, dist_cm)
            return

        if self.state == DONE:
            self._publish_zero()
            if not self._done_logged:
                self.get_logger().info('Front approach DONE — handing off to docking phase (not implemented yet).')
                self._done_logged = True
            return

        next_state, action = self._decide(self.state, valid, num_markers, yaw_deg, lat_cm, dist_cm)

        key = f'{self.state}->{next_state}:{action}'
        if key == self._confirm_key:
            self._confirm_count += 1
        else:
            self._confirm_key = key
            self._confirm_count = 1

        if self._confirm_count < self.confirm_frames:
            return

        if next_state != self.state:
            self.get_logger().info(f'{self.state} -> {next_state} ({action})')
        self.state = next_state
        self._execute_action(action)

    def destroy_node(self):
        self._publish_zero()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BxuFrontApproachControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()