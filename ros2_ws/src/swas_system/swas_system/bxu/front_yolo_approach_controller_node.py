import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from swas_msgs.msg import BxuFrontYoloObservation, BxuAlignmentOutput


SEARCH = 'SEARCH'
PD = 'PD'
BLIND_FORWARD = 'BLIND_FORWARD'
DONE = 'DONE'
FAULT = 'FAULT'


class BxuFrontYoloApproachControllerNode(Node):
    """Image-space YOLO front approach controller.

    State machine:
        SEARCH -> PD -> BLIND_FORWARD -> DONE
                     -> FAULT        -> FAULT

    The controller regulates:
      - horizontal image-centre error;
      - wheel-line image angle.

    It does not estimate metric distance, lateral offset, or vehicle yaw.
    """

    def __init__(self):
        super().__init__('bxu_front_yolo_approach_controller_node')

        self._declare_parameters()
        self._load_parameters()

        self.cmd_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, 10
        )
        self.status_pub = self.create_publisher(
            String, self.status_topic, 10
        )

        self.enable_sub = self.create_subscription(
            Bool, self.enable_topic, self._enable_cb, 10
        )
        self.observation_sub = self.create_subscription(
            BxuFrontYoloObservation,
            self.observation_topic,
            self._observation_cb,
            10,
        )
        self.dock_sub = self.create_subscription(
            BxuAlignmentOutput,
            self.dock_alignment_topic,
            self._dock_alignment_cb,
            10,
        )

        self.enabled = False

        self.state = SEARCH

        # Observation bookkeeping.
        self._latest_observation = None
        self._latest_observation_time = None
        self._last_observation_key = None

        # PD state.
        self._prev_lat_error = None
        self._prev_yaw_error = None
        self._prev_observation_stamp = None
        self._d_lat = 0.0
        self._d_yaw = 0.0

        # SEARCH confirmation.
        self._acquire_count = 0
        self._last_acquire_key = None

        # Blind-entry confirmation.
        self._blind_entry_count = 0
        self._last_blind_entry_key = None
        self._last_blind_entry_time = None

        # Blind-forward state.
        self._blind_start_time = None
        self._dock_seen_count = 0
        self._last_dock_key = None

        # Terminal-state one-shot flags.
        self._done_zero_sent = False
        self._done_logged = False
        self._fault_logged = False
        self._fault_reason = ''

        self.timer = self.create_timer(self.action_dt, self._tick)

        self.get_logger().info(
            'BXU front YOLO approach controller started; state=SEARCH.'
        )

        self.declare_parameter('enable_control_services', False)

        if self.get_parameter('enable_control_services').value:
            self.start_service = self.create_service(
                Trigger,
                '/bxu/front_yolo_approach/start',
                self._start_service_cb,
            )
            self.stop_service = self.create_service(
                Trigger,
                '/bxu/front_yolo_approach/stop',
                self._stop_service_cb,
            )

    def _declare_parameters(self):
        defaults = {
            # Topics. Defaults match the existing supervisor wiring so this
            # controller is a drop-in replacement for the ArUco front controller.
            'observation_topic': '/bxu/front_yolo_vision/observation',
            'dock_alignment_topic': '/bxu/docking_vision/alignment',
            'cmd_vel_topic': '/bxu/cmd_vel',
            'status_topic': '/bxu/front_approach/controller_status',
            'enable_topic': '/bxu/front_controller/enabled',

            # Timing.
            'action_dt': 0.05,
            'observation_stale_timeout_s': 1.0,

            # SEARCH.
            'search_omega_mps': 0.20,
            'acquire_frames': 3,
            'acquire_window_s': 1.5,

            # PD gains. Errors are normalized image-space quantities.
            'kp_lateral': 0.35,
            'kd_lateral': 0.05,
            'kp_yaw': 0.60,
            'kd_yaw': 0.08,

            # Command limits.
            'max_vy_mps': 0.08,
            'max_omega_rad_s': 0.20,

            # Set to -1.0 for the supplied front-camera optical transform:
            # image-right corresponds to BXU -Y, while BXU +Y is left.
            'lateral_command_sign': -1.0,

            # Validated in simulation: positive wheel-line error requires
            # negative omega; negative wheel-line error requires positive omega.
            'yaw_command_sign': -1.0, 

            # Forward motion is enabled only while both alignment features are
            # within tolerance. No metric distance is required.
            'approach_vx_mps': 0.05,

            # Alignment tolerances.
            'lateral_tolerance': 0.03,
            'yaw_tolerance_rad': 0.035,

            'no_target_timeout_s': 1.5,

            # Blind-entry criterion: wheel boxes approaching image boundary.
            # Choices: left, right, top, bottom,
            #          min_horizontal, min_vertical, min_any.
            'blind_edge': 'min_horizontal',
            'blind_margin_ratio': 0.08,
            'blind_entry_frames': 3,
            'blind_entry_memory_s': 1.0,
            'critical_margin_ratio': 0.03,

            # Blind forward.
            'blind_forward_axis': 'x',
            'blind_forward_speed_mps': 0.05,
            'blind_forward_ramp_high_mult': 3.0,
            'blind_forward_ramp_high_duration_s': 5.0,
            'blind_forward_ramp_transition_s': 4.0,
            'blind_forward_timeout_s': 25.0,
            'blind_forward_detection_confirm_frames': 3,
            'dock_observation_stale_timeout_s': 0.5,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        get = lambda name: self.get_parameter(name).value

        self.observation_topic = str(get('observation_topic'))
        self.dock_alignment_topic = str(get('dock_alignment_topic'))
        self.cmd_vel_topic = str(get('cmd_vel_topic'))
        self.status_topic = str(get('status_topic'))
        self.enable_topic = str(get('enable_topic'))

        self.action_dt = float(get('action_dt'))
        self.observation_stale_timeout_s = float(
            get('observation_stale_timeout_s')
        )

        self.search_omega_mps = float(get('search_omega_mps'))
        self.acquire_frames = int(get('acquire_frames'))

        self.kp_lateral = float(get('kp_lateral'))
        self.kd_lateral = float(get('kd_lateral'))
        self.kp_yaw = float(get('kp_yaw'))
        self.kd_yaw = float(get('kd_yaw'))

        self.max_vy_mps = float(get('max_vy_mps'))
        self.max_omega_rad_s = float(get('max_omega_rad_s'))

        self.lateral_command_sign = float(get('lateral_command_sign'))
        self.yaw_command_sign = float(get('yaw_command_sign'))

        self.approach_vx_mps = float(get('approach_vx_mps'))

        self.lateral_tolerance = float(get('lateral_tolerance'))
        self.yaw_tolerance_rad = float(get('yaw_tolerance_rad'))

        self.blind_edge = str(get('blind_edge'))
        self.blind_margin_ratio = float(get('blind_margin_ratio'))
        self.blind_entry_frames = int(get('blind_entry_frames'))
        self.blind_entry_memory_s = float(get('blind_entry_memory_s'))
        self.critical_margin_ratio = float(get('critical_margin_ratio'))
        self.no_target_timeout_s = float(get('no_target_timeout_s'))
        self.acquire_window_s = float(get('acquire_window_s'))

        if self.acquire_window_s <= 0.0:
            raise ValueError('acquire_window_s must be positive.')

        if self.no_target_timeout_s <= 0.0:
            raise ValueError('no_target_timeout_s must be positive.')   

        self.blind_forward_axis = str(get('blind_forward_axis'))
        self.blind_forward_speed_mps = float(get('blind_forward_speed_mps'))
        self.blind_forward_ramp_high_mult = float(
            get('blind_forward_ramp_high_mult')
        )
        self.blind_forward_ramp_high_duration_s = float(
            get('blind_forward_ramp_high_duration_s')
        )
        self.blind_forward_ramp_transition_s = float(
            get('blind_forward_ramp_transition_s')
        )
        self.blind_forward_timeout_s = float(get('blind_forward_timeout_s'))
        self.blind_forward_detection_confirm_frames = int(
            get('blind_forward_detection_confirm_frames')
        )
        self.dock_observation_stale_timeout_s = float(
            get('dock_observation_stale_timeout_s')
        )

        if self.action_dt <= 0.0:
            raise ValueError('action_dt must be positive.')
        if self.observation_stale_timeout_s <= 0.0:
            raise ValueError('observation_stale_timeout_s must be positive.')
        if self.acquire_frames < 1:
            raise ValueError('acquire_frames must be at least one.')
        if self.blind_entry_frames < 1:
            raise ValueError('blind_entry_frames must be at least one.')
        if self.blind_forward_detection_confirm_frames < 1:
            raise ValueError(
                'blind_forward_detection_confirm_frames must be at least one.'
            )
        if self.lateral_command_sign not in (-1.0, 1.0):
            raise ValueError('lateral_command_sign must be -1.0 or 1.0.')
        if self.yaw_command_sign not in (-1.0, 1.0):
            raise ValueError('yaw_command_sign must be -1.0 or 1.0.')
        if self.blind_edge not in (
            'left', 'right', 'top', 'bottom',
            'min_horizontal', 'min_vertical', 'min_any',
        ):
            raise ValueError(f'Unsupported blind_edge: {self.blind_edge}')

    def _start_service_cb(self, request, response):
        if self.enabled:
            response.success = False
            response.message = 'Controller is already enabled.'
            return response

        self._enable_cb(Bool(data=True))
        response.success = True
        response.message = 'Front YOLO approach enabled.'
        return response

    def _stop_service_cb(self, request, response):
        self._enable_cb(Bool(data=False))
        self._publish_zero()
        response.success = True
        response.message = 'Front YOLO approach disabled.'
        return response

    def _enable_cb(self, message: Bool):
        was_enabled = self.enabled
        self.enabled = bool(message.data)

        if was_enabled and not self.enabled:
            self._publish_zero()
            self._reset_runtime()
            self._publish_status('INACTIVE')
        elif not was_enabled and self.enabled:
            self._reset_runtime()
            self._publish_status('ACTIVE')

    def _reset_runtime(self):
        self.state = SEARCH

        self._latest_observation = None
        self._latest_observation_time = None
        self._last_observation_key = None

        self._prev_lat_error = None
        self._prev_yaw_error = None
        self._prev_observation_stamp = None
        self._d_lat = 0.0
        self._d_yaw = 0.0

        self._acquire_count = 0
        self._last_acquire_key = None

        self._usable_acquisition_times = []

        self._last_usable_time = None

        self._blind_entry_count = 0
        self._last_blind_entry_key = None
        self._last_blind_entry_time = None

        self._blind_start_time = None
        self._dock_seen_count = 0
        self._last_dock_key = None

        self._done_zero_sent = False
        self._done_logged = False
        self._fault_logged = False
        self._fault_reason = ''

    def _publish_status(self, text: str):
        message = String()
        message.data = text
        self.status_pub.publish(message)

    def _publish(self, vx: float, vy: float, omega: float):
        message = Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(omega)
        self.cmd_pub.publish(message)

    def _publish_zero(self):
        self._publish(0.0, 0.0, 0.0)

    def _observation_cb(self, message: BxuFrontYoloObservation):
        self._latest_observation = message
        self._latest_observation_time = time.monotonic()

    def _dock_alignment_cb(self, message: BxuAlignmentOutput):
        self._latest_dock_message = message
        self._latest_dock_time = time.monotonic()

    def _get_observation(self):
        if self._latest_observation is None:
            return None

        age = time.monotonic() - self._latest_observation_time
        if age > self.observation_stale_timeout_s:
            return None

        return self._latest_observation

    def _is_new_observation(self, message):
        key = (
            message.header.stamp.sec,
            message.header.stamp.nanosec,
        )
        is_new = key != self._last_observation_key
        self._last_observation_key = key
        return is_new

    def _wheel_boxes(self, message):
        if len(message.wheel_box_xyxy_px) != 4 * len(message.wheel_held):
            return []

        boxes = []
        for index in range(len(message.wheel_held)):
            start = 4 * index
            box = tuple(
                float(value)
                for value in message.wheel_box_xyxy_px[start:start + 4]
            )
            boxes.append(box)
        return boxes

    def _wheel_margin_ratio(self, message):
        boxes = self._wheel_boxes(message)
        if len(boxes) != 2:
            return None

        width = float(message.image_width)
        height = float(message.image_height)
        if width <= 0.0 or height <= 0.0:
            return None

        left = min(box[0] for box in boxes) / width
        right = min(width - box[2] for box in boxes) / width
        top = min(box[1] for box in boxes) / height
        bottom = min(height - box[3] for box in boxes) / height

        if self.blind_edge == 'left':
            return left
        if self.blind_edge == 'right':
            return right
        if self.blind_edge == 'top':
            return top
        if self.blind_edge == 'bottom':
            return bottom
        if self.blind_edge == 'min_horizontal':
            return min(left, right)
        if self.blind_edge == 'min_vertical':
            return min(top, bottom)
        return min(left, right, top, bottom)

    def _alignment_errors(self, message):
        if not message.valid:
            return None, None

        if not self._center_is_valid(message):
            return None, None

        lateral_error = float(message.center_error_x_normalized)

        if message.wheel_line_valid:
            yaw_error = float(message.wheel_line_angle_rad)
        else:
            # Car-side-only observation: centre is usable, heading is not.
            yaw_error = None

        return lateral_error, yaw_error

    def _pair_is_fresh(self, message):
        return (
            len(message.wheel_held) == 2
            and not any(message.wheel_held)
            and message.real_wheels == 2
            and message.held_wheels == 0
        )

    def _alignment_is_acceptable(self, lateral_error, yaw_error):
        return (
            lateral_error is not None
            and yaw_error is not None
            and abs(lateral_error) <= self.lateral_tolerance
            and abs(yaw_error) <= self.yaw_tolerance_rad
        )

    def _count_new(self, message, key_attribute, count_attribute):
        key = (
            message.header.stamp.sec,
            message.header.stamp.nanosec,
        )

        if key == getattr(self, key_attribute):
            return False

        setattr(self, key_attribute, key)
        setattr(self, count_attribute, getattr(self, count_attribute) + 1)
        return True

    def _tick(self):
        if not self.enabled:
            return

        if self.state == BLIND_FORWARD:
            self._tick_blind_forward()
            return

        if self.state == DONE:
            if not self._done_zero_sent:
                self._publish_zero()
                self._done_zero_sent = True
            if not self._done_logged:
                self._publish_status('DONE')
                self._done_logged = True
            return

        if self.state == FAULT:
            self._publish_zero()
            if not self._fault_logged:
                self._publish_status(f'FAULT:{self._fault_reason}')
                self._fault_logged = True
            return

        observation = self._get_observation()

        if observation is None:
            self._handle_lost_observation()
            return

        if not self._is_new_observation(observation):
            return

        lateral_error, yaw_error = self._alignment_errors(observation)

        if self.state == SEARCH:
            self._tick_search(observation, lateral_error, yaw_error)
            return

        if self.state == PD:
            self._tick_pd(
                observation, lateral_error, yaw_error
            )
            return

    def _handle_lost_observation(self):
        if self.state == PD and self._blind_entry_is_recent():
            self._enter_blind_forward()
            return

        self._publish_zero()
        self._reset_pd()

        if self.state == SEARCH:
            self._acquire_count = 0
            self._last_acquire_key = None
            self._publish_status('SEARCH_LOST')
        else:
            self._set_fault('FRONT_OBSERVATION_LOST')

    def _tick_search(self, observation, lateral_error, yaw_error):
        usable = (
            self._pair_is_fresh(observation)
            and lateral_error is not None
            and yaw_error is not None
        )

        now = time.monotonic()

        if usable:
            self._usable_acquisition_times.append(now)
        else:
            self._publish_zero()
            self._publish(0.0, 0.0, self.search_omega_mps)
            self._publish_status('SEARCH')
            return

        # Keep only observations inside the acquisition window.
        cutoff = now - self.acquire_window_s
        self._usable_acquisition_times = [
            timestamp
            for timestamp in self._usable_acquisition_times
            if timestamp >= cutoff
        ]

        if len(self._usable_acquisition_times) >= self.acquire_frames:
            self._reset_pd()
            self.state = PD
            self._publish_status('PD')
            return

        self._publish_zero()
        self._publish_status('SEARCH_CONFIRMING')

    def _reset_pd(self):
        self._prev_lat_error = None
        self._prev_yaw_error = None
        self._prev_observation_stamp = None
        self._d_lat = 0.0
        self._d_yaw = 0.0

    def _tick_pd(self, observation, lateral_error, yaw_error):
        margin = self._wheel_margin_ratio(observation)
        now = time.monotonic()

        # A valid centre is the minimum requirement for remaining in PD.
        if not self._center_is_valid(observation):
            self._publish_zero()

            if (
                self._last_usable_time is None
                or now - self._last_usable_time > self.no_target_timeout_s
            ):
                self._set_fault('NO_USABLE_FRONT_TARGET')
            else:
                self._publish_status('PD_WAITING_FOR_TARGET')

            return

        self._last_usable_time = now

        # Do not enter blind approach unless both wheels are fresh.
        if not self._pair_is_fresh(observation):
            self._blind_entry_count = 0
            self._last_blind_entry_key = None

        # Full wheel evidence: regulate both image features.
        if self._pair_is_fresh(observation):
            if lateral_error is None or yaw_error is None:
                self._publish_zero()
                self._set_fault('INVALID_ALIGNMENT_FEATURES')
                return

            self._update_derivatives(
                observation, lateral_error, yaw_error
            )

            vy = self.lateral_command_sign * (
                self.kp_lateral * lateral_error
                + self.kd_lateral * self._d_lat
            )
            omega = self.yaw_command_sign * (
                self.kp_yaw * yaw_error
                + self.kd_yaw * self._d_yaw
            )

            vy = max(-self.max_vy_mps, min(self.max_vy_mps, vy))
            omega = max(
                -self.max_omega_rad_s,
                min(self.max_omega_rad_s, omega),
            )

            aligned = self._alignment_is_acceptable(
                lateral_error, yaw_error
            )
            vx = self.approach_vx_mps if aligned else 0.0

            self._publish(vx, vy, omega)

            if aligned:
                self._update_blind_entry_confirmation(
                    observation, margin
                )
                self._publish_status('PD_ALIGNED')
            else:
                self._publish_status('PD_ALIGNING')

            return

        # Held wheel pair: still regulate both image features, but never
        # advance toward the target on held evidence.
        if self._wheel_pair_is_held(observation):
            if lateral_error is None or yaw_error is None:
                self._publish_zero()
                self._publish_status('PD_HELD_WHEELS')
                return

            self._update_derivatives(
                observation, lateral_error, yaw_error
            )

            vy = self.lateral_command_sign * (
                self.kp_lateral * lateral_error
                + self.kd_lateral * self._d_lat
            )
            omega = self.yaw_command_sign * (
                self.kp_yaw * yaw_error
                + self.kd_yaw * self._d_yaw
            )

            vy = max(-self.max_vy_mps, min(self.max_vy_mps, vy))
            omega = max(
                -self.max_omega_rad_s,
                min(self.max_omega_rad_s, omega),
            )

            self._publish(0.0, vy, omega)
            self._publish_status('PD_HELD_WHEELS')
            return

        # Car-side-only fallback: correct lateral image-centre error only.
        # Wheel-line yaw is unavailable, so angular velocity must be zero.
        if lateral_error is None:
            self._publish_zero()
            self._publish_status('PD_CAR_ONLY')
            return

        self._update_derivatives(
            observation, lateral_error, 0.0
        )

        vy = self.lateral_command_sign * (
            self.kp_lateral * lateral_error
            + self.kd_lateral * self._d_lat
        )
        vy = max(-self.max_vy_mps, min(self.max_vy_mps, vy))

        self._publish(0.0, vy, 0.0)
        self._publish_status('PD_CAR_ONLY')

    def _wheel_pair_is_held(self, observation):
        return (
            observation.state == 'HELD_WHEELS'
            and observation.wheel_line_valid
        )

    def _center_is_valid(self, observation):
        return (
            observation.valid
            and observation.center_x_px > 0.0
            and observation.center_y_px > 0.0
        )

    def _update_derivatives(
        self, observation, lateral_error, yaw_error
    ):
        stamp = (
            observation.header.stamp.sec
            + observation.header.stamp.nanosec * 1e-9
        )

        if self._prev_observation_stamp is None:
            raw_d_lat = 0.0
            raw_d_yaw = 0.0
        else:
            dt = stamp - self._prev_observation_stamp
            if dt <= 0.0:
                raw_d_lat = 0.0
                raw_d_yaw = 0.0
            else:
                raw_d_lat = (
                    (lateral_error - self._prev_lat_error) / dt
                )
                raw_d_yaw = (
                    (yaw_error - self._prev_yaw_error) / dt
                )

        # Lightweight exponential derivative filter.
        alpha = 0.2
        self._d_lat = alpha * raw_d_lat + (1.0 - alpha) * self._d_lat
        self._d_yaw = alpha * raw_d_yaw + (1.0 - alpha) * self._d_yaw

        self._prev_lat_error = lateral_error
        self._prev_yaw_error = yaw_error
        self._prev_observation_stamp = stamp

    def _update_blind_entry_confirmation(self, observation, margin):

        if not self._pair_is_fresh(observation):
            self._blind_entry_count = 0
            self._last_blind_entry_key = None
            return

        key = (
            observation.header.stamp.sec,
            observation.header.stamp.nanosec,
        )

        if key == self._last_blind_entry_key:
            return

        if margin is None or margin > self.blind_margin_ratio:
            self._blind_entry_count = 0
            self._last_blind_entry_key = None
            return

        self._last_blind_entry_key = key
        self._blind_entry_count += 1
        self._last_blind_entry_time = time.monotonic()

        if self._blind_entry_count >= self.blind_entry_frames:
            self._enter_blind_forward()

    def _blind_entry_is_recent(self):
        if self._last_blind_entry_time is None:
            return False

        return (
            time.monotonic() - self._last_blind_entry_time
            <= self.blind_entry_memory_s
        )

    def _enter_blind_forward(self):
        self._publish_zero()
        self._reset_pd()

        self.state = BLIND_FORWARD
        self._blind_start_time = None
        self._dock_seen_count = 0
        self._last_dock_key = None

        self._publish_status('BLIND_FORWARD')
        self.get_logger().info('Entering blind forward approach.')

    def _dock_marker_seen(self):
        message = getattr(self, '_latest_dock_message', None)
        latest_time = getattr(self, '_latest_dock_time', None)

        if message is None or latest_time is None:
            return False

        if (
            time.monotonic() - latest_time
            > self.dock_observation_stale_timeout_s
        ):
            return False

        return bool(message.markers_detected) and (
            message.num_markers_detected >= 1
        )

    def _tick_blind_forward(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self._blind_start_time is None:
            self._blind_start_time = now
            self._dock_seen_count = 0
            self._last_dock_key = None
            self._publish_status('BLIND_FORWARD')

        if self._dock_marker_seen():
            message = self._latest_dock_message
            key = (
                message.header.stamp.sec,
                message.header.stamp.nanosec,
            )

            if key != self._last_dock_key:
                self._last_dock_key = key
                self._dock_seen_count += 1

            if (
                self._dock_seen_count
                >= self.blind_forward_detection_confirm_frames
            ):
                self._publish_zero()
                self.state = DONE
                self._blind_start_time = None
                self._publish_status('DONE')
                self.get_logger().info(
                    'Docking markers confirmed; front approach complete.'
                )
                return

        elapsed = now - self._blind_start_time

        self._publish_status(f'BLIND_FORWARD:{elapsed:.1f}s')

        simulation_mode = bool(self.get_parameter('use_sim_time').value)

        # Retained requested behaviour: no blind-forward timeout in simulation.
        if (
            not simulation_mode
            and elapsed > self.blind_forward_timeout_s
        ):
            self._publish_zero()
            self._blind_start_time = None
            self._set_fault('BLIND_FORWARD_TIMEOUT')
            return

        base_speed = self.blind_forward_speed_mps
        high_multiplier = self.blind_forward_ramp_high_mult
        high_duration = self.blind_forward_ramp_high_duration_s
        transition_duration = self.blind_forward_ramp_transition_s

        if elapsed < high_duration:
            speed = high_multiplier * base_speed
        elif elapsed < high_duration + transition_duration:
            progress = (
                (elapsed - high_duration) / transition_duration
            )
            speed = (
                high_multiplier
                - (high_multiplier - 1.0) * progress
            ) * base_speed
        else:
            speed = base_speed

        vx = speed if self.blind_forward_axis == 'x' else 0.0
        vy = speed if self.blind_forward_axis == 'y' else 0.0

        self._publish(vx, vy, 0.0)

    def _set_fault(self, reason: str):
        self._publish_zero()
        self._reset_pd()

        self.state = FAULT
        self._fault_reason = reason
        self._blind_start_time = None

        self.get_logger().error(f'Front YOLO approach fault: {reason}')
        self._publish_status(f'FAULT:{reason}')

    def destroy_node(self):
        self._publish_zero()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = BxuFrontYoloApproachControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()