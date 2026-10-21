"""Image-space vehicle centre estimation. No ROS, camera, or inference dependency."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Detection:
    box: tuple
    confidence: float
    class_id: int


def center(box):
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def area(box):
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def iou(a, b):
    intersection = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0.0, min(a[3], b[3]) - max(a[1], b[1]))
    union = area(a) + area(b) - intersection
    return intersection / union if union > 0 else 0.0


def padded(box, ratio):
    x, y = ratio * (box[2] - box[0]), ratio * (box[3] - box[1])
    return (box[0] - x, box[1] - y, box[2] + x, box[3] + y)


def contains(box, point):
    return box[0] <= point[0] <= box[2] and box[1] <= point[1] <= box[3]


def dedup(detections, threshold):
    # Retain the original script's larger-box-first dedup policy.
    kept = []
    for detection in sorted(detections,
                            key=lambda d: (area(d.box), d.confidence), reverse=True):
        if all(iou(detection.box, k.box) <= threshold for k in kept):
            kept.append(detection)
    return kept


@dataclass
class EstimatorConfig:
    dedup_iou: float = 0.4
    car_pad_ratio: float = 0.05
    ema_alpha: float = 0.2
    car_weight: float = 0.03
    history_length: int = 10
    wheel_memory_frames: int = 7
    wheel_memory_seconds: float = 0.6
    wheel_gate_ratio: float = 0.30
    primary_lost_frames: int = 20
    primary_memory_seconds: float = 2.0
    primary_switch_ratio: float = 1.5
    primary_switch_frames: int = 15
    fusion_memory_seconds: float = 0.6
    center_memory_seconds: float = 1.5 
    wheel_only_mode: bool = False
    confirmed_pair_frames: int = 3
    confirmed_wheel_memory_frames: int = 15
    confirmed_wheel_memory_seconds: float = 1.5

    def __post_init__(self):
        for name in ('dedup_iou', 'ema_alpha', 'car_weight'):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError(f'{name} must be in [0, 1].')
        if self.ema_alpha <= 0:
            raise ValueError('ema_alpha must be positive.')
        for name in ('history_length', 'primary_switch_frames'):
            if getattr(self, name) < 1:
                raise ValueError(f'{name} must be at least one.')
        for name in ('wheel_memory_frames', 'primary_lost_frames'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} cannot be negative.')
        for name in ('car_pad_ratio', 'wheel_memory_seconds', 'wheel_gate_ratio',
                     'primary_memory_seconds', 'fusion_memory_seconds'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and nonnegative.')
        if not math.isfinite(self.primary_switch_ratio) or self.primary_switch_ratio <= 1:
            raise ValueError('primary_switch_ratio must exceed one.')
        if not isinstance(self.wheel_only_mode, bool):
            raise ValueError('wheel_only_mode must be a boolean.')
        if not math.isfinite(self.center_memory_seconds) or (self.center_memory_seconds < 0):
            raise ValueError(
                'center_memory_seconds must be finite and nonnegative.'
            )
        if self.confirmed_pair_frames < 1:
            raise ValueError('confirmed_pair_frames must be at least one.')

        for name in ('confirmed_wheel_memory_frames',):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} cannot be negative.')

        for name in ('confirmed_wheel_memory_seconds',):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and nonnegative.')

class FrontYoloEstimator:
    def __init__(self, config=None):
        self.config = config or EstimatorConfig()
        self.reset()

    def reset(self):
        self.primary = None
        self.primary_seen = None
        self.lost = 0
        self.challenger = None
        self.challenge_count = 0
        self.last_time = None
        self._reset_fusion()
        self._reset_confirmed_pair()

    def _reset_fusion(self):
        self.wheels = {}
        self.ema = None
        self.complete = None
        self.complete_time = None
        self.history = deque(maxlen=self.config.history_length)
        self.last_observation_time = None
        self._memory_center = None
        self._memory_center_time = None
        self._memory_confidence = 0.0


    def _reset_confirmed_pair(self):
        self.confirmed = False
        self.confirmed_count = 0
        self.confirmed_gate_px = None
        self.confirmed_entries = {}

    def _associate(self, cars, wheels):
        assigned = set()
        candidates = []
        for car in cars:
            cx = center(car.box)[0]
            region = padded(car.box, self.config.car_pad_ratio)
            nearby = [i for i, w in enumerate(wheels)
                      if i not in assigned and contains(region, center(w.box))]
            left = [i for i in nearby if center(wheels[i].box)[0] < cx]
            right = [i for i in nearby if center(wheels[i].box)[0] >= cx]
            chosen = {}
            if left:
                chosen['left'] = max(left, key=lambda i: center(wheels[i].box)[0])
            if right:
                chosen['right'] = min(right, key=lambda i: center(wheels[i].box)[0])
            used = set(chosen.values())
            candidates.append({
                'pair': {side: wheels[i] for side, i in chosen.items()},
                'secondary': [wheels[i] for i in nearby if i not in used],
            })
            # Deliberately preserve the original larger-car-first ownership policy.
            assigned.update(nearby)
        return candidates, [w for i, w in enumerate(wheels) if i not in assigned]

    def _select(self, cars, now):
        cfg = self.config
        if self.primary_seen is not None and now - self.primary_seen > cfg.primary_memory_seconds:
            self.primary = None
            self.challenge_count = 0
            self.challenger = None
            self._reset_fusion()
        if not cars:
            self.lost += 1
            self.challenge_count = 0
            self.challenger = None
            if self.lost > cfg.primary_lost_frames:
                self.primary = None
                self._reset_fusion()
            return None, False
        biggest = max(range(len(cars)), key=lambda i: area(cars[i].box))
        if self.primary is None:
            self.primary = cars[biggest].box
            self.primary_seen = now
            self.lost = 0
            self._reset_fusion()
            return biggest, True
        previous = self.primary
        pc = center(previous)
        match = min(range(len(cars)), key=lambda i: math.dist(center(cars[i].box), pc))
        diagonal = math.hypot(previous[2] - previous[0], previous[3] - previous[1])
        matched = math.dist(center(cars[match].box), pc) <= 0.75 * diagonal
        if matched:
            self.primary = cars[match].box
            self.primary_seen = now
            self.lost = 0
        else:
            self.lost += 1
        competitors = [i for i in range(len(cars)) if not (matched and i == match)]
        challenger_idx = max(competitors, key=lambda i: area(cars[i].box)) if competitors else None
        if challenger_idx is not None and area(cars[challenger_idx].box) > cfg.primary_switch_ratio * area(self.primary):
            box = cars[challenger_idx].box
            if self.challenger is not None and iou(self.challenger, box) > 0.1:
                self.challenge_count += 1
            else:
                self.challenge_count = 1
            self.challenger = box
        else:
            self.challenge_count = 0
            self.challenger = None
        if self.lost > cfg.primary_lost_frames or self.challenge_count >= cfg.primary_switch_frames:
            self.primary = cars[biggest].box
            self.primary_seen = now
            self.lost = 0
            self.challenge_count = 0
            self.challenger = None
            self._reset_fusion()
            return biggest, True
        return (match if matched else None), False

    def _remember_wheels(self, car, pair, now):
        cfg = self.config
        region = padded(car.box, cfg.car_pad_ratio)
        cx = center(car.box)[0]
        gate = cfg.wheel_gate_ratio * math.hypot(car.box[2] - car.box[0], car.box[3] - car.box[1])
        entries = []
        for side in ('left', 'right'):
            old = self.wheels.get(side)
            detection = pair.get(side)
            if detection is not None:
                # Large jumps invalidate other held evidence and the old fused history.
                if old and math.dist(center(old['detection'].box), center(detection.box)) > gate:
                    self.wheels = {}
                    self.complete = None
                    self.complete_time = None
                    self.ema = None
                    self.history.clear()
                    self._reset_confirmed_pair()
                self.wheels[side] = {'detection': detection, 'age': 0, 'seen': now}
        for side in ('left', 'right'):
            slot = self.wheels.get(side)
            if slot is None:
                continue
            if side not in pair:
                slot['age'] += 1
            point = center(slot['detection'].box)
            correct_side = point[0] < cx if side == 'left' else point[0] >= cx
            if (slot['age'] > cfg.wheel_memory_frames or
                    now - slot['seen'] > cfg.wheel_memory_seconds or
                    not contains(region, point) or not correct_side):
                self.wheels.pop(side, None)
                continue
            entries.append({'side': side, 'detection': slot['detection'],
                            'held': slot['age'] > 0, 'age_frames': slot['age']})
        return entries

    def _confirm_pair(self, car, entries, now):
        """Confirm a wheel pair only after repeated, fully fresh observations."""
        if len(entries) == 2 and all(not entry['held'] for entry in entries):
            self.confirmed_count += 1
            if self.confirmed_count >= self.config.confirmed_pair_frames:
                self.confirmed = True
                self.confirmed_gate_px = (
                    self.config.wheel_gate_ratio
                    * math.hypot(car.box[2] - car.box[0], car.box[3] - car.box[1])
                )
                self.confirmed_entries = {
                    entry['side']: {
                        'detection': entry['detection'],
                        'age': 0,
                        'seen': now,
                    }
                    for entry in entries
                }
        else:
            self.confirmed_count = 0

    def _update_confirmed_wheels(self, wheels, now):
        """Track an already-confirmed wheel pair without requiring a car side."""
        cfg = self.config
        used_indices = set()
        entries = []

        for side in ('left', 'right'):
            slot = self.confirmed_entries.get(side)
            if slot is None:
                continue

            previous_point = center(slot['detection'].box)
            best_index = None
            best_distance = None

            for index, detection in enumerate(wheels):
                if index in used_indices:
                    continue

                distance = math.dist(center(detection.box), previous_point)
                if distance <= self.confirmed_gate_px and (
                    best_distance is None or distance < best_distance
                ):
                    best_index = index
                    best_distance = distance

            if best_index is not None:
                used_indices.add(best_index)
                slot['detection'] = wheels[best_index]
                slot['age'] = 0
                slot['seen'] = now
            else:
                slot['age'] += 1
                if (
                    slot['age'] > cfg.confirmed_wheel_memory_frames
                    or now - slot['seen'] > cfg.confirmed_wheel_memory_seconds
                ):
                    slot = None

            if slot is not None:
                self.confirmed_entries[side] = slot
                entries.append({
                    'side': side,
                    'detection': slot['detection'],
                    'held': slot['age'] > 0,
                    'age_frames': slot['age'],
                })
            else:
                self.confirmed_entries.pop(side, None)

        if not self.confirmed_entries:
            self._reset_confirmed_pair()

        return entries, used_indices

    def _update_wheel_only(self, wheels, now, result):
        """Test-only estimator using exactly two wheel detections."""
        result['state'] = 'WHEEL_ONLY'
        result['wheel_only_mode'] = True

        if len(wheels) != 2:
            self._reset_fusion()
            return result

        left, right = sorted(wheels, key=lambda detection: center(detection.box)[0])
        entries = [
            {'side': 'left', 'detection': left, 'held': False, 'age_frames': 0},
            {'side': 'right', 'detection': right, 'held': False, 'age_frames': 0},
        ]

        a = center(left.box)
        b = center(right.box)
        wheel_center = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        angle = math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))

        # No car bbox and no history-assisted fusion in this mode.
        fused_center = wheel_center

        alpha = self.config.ema_alpha
        self.ema = fused_center if self.ema is None else tuple(
            alpha * fused_center[i] + (1 - alpha) * self.ema[i]
            for i in (0, 1)
        )
        self.history.append(self.ema)
        self.last_observation_time = now

        result.update(
            valid=True,
            state='WHEEL_ONLY',
            cars=[],
            candidates=[],
            orphans=[],
            primary_index=None,
            primary_switched=False,
            primary=None,
            wheel_entries=entries,
            wheel_center=wheel_center,
            car_center=None,
            fused_center=fused_center,
            ema_center=self.ema,
            wheel_line_angle_deg=angle,
            history=list(self.history),
            real_wheels=2,
            held_wheels=0,
            detector_confidence=min(left.confidence, right.confidence),
            confirmed_pair=False,
        )
        return result


    def update(self, detections, now):
        result = self._update_impl(detections, now)
        return self._apply_center_memory(result, now)

    def _apply_center_memory(self, result, now):
        """Hold the last valid image-space centre through short gaps.

        This does not extrapolate motion and does not claim wheel evidence.
        It only prevents the centre estimate from disappearing instantly when
        the car side or wheel detections are briefly missed.
        """
        ema_center = result.get('ema_center')

        if ema_center is not None:
            self._memory_center = tuple(float(value) for value in ema_center)
            self._memory_center_time = now
            self._memory_confidence = float(
                result.get('detector_confidence') or 0.0
            )
            return result

        if (
            self._memory_center is None
            or now - self._memory_center_time
            > self.config.center_memory_seconds
        ):
            return result

        remembered_center = self._memory_center

        result.update(
            valid=True,
            state='CENTER_MEMORY',
            ema_center=remembered_center,
            fused_center=remembered_center,
            wheel_center=None,
            car_center=None,
            wheel_entries=[],
            wheel_line_angle_deg=None,
            real_wheels=0,
            held_wheels=0,
            confirmed_pair=False,
            detector_confidence=0.5 * self._memory_confidence,
        )

        return result

    def _update_impl(self, detections, now):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('Estimator time must be finite.')

        if self.last_time is not None and now < self.last_time:
            self.reset()
        self.last_time = now

        if (
            self.last_observation_time is not None
            and now - self.last_observation_time > self.config.fusion_memory_seconds
        ):
            self._reset_fusion()

        valid = [
            d for d in detections
            if len(d.box) == 4 and area(d.box) > 0
            and all(math.isfinite(v) for v in (*d.box, d.confidence))
        ]
        cars = dedup(
            [d for d in valid if d.class_id == 0], self.config.dedup_iou
        )
        wheels = dedup(
            [d for d in valid if d.class_id == 1], self.config.dedup_iou
        )

        result = {
            'valid': False,
            'state': 'NONE',
            'cars': cars,
            'wheels': wheels,
            'candidates': [],
            'orphans': [],
            'primary_index': None,
            'primary_switched': False,
            'primary': None,
            'wheel_entries': [],
            'wheel_center': None,
            'car_center': None,
            'fused_center': None,
            'ema_center': None,
            'wheel_line_angle_deg': None,
            'history': [],
            'real_wheels': 0,
            'held_wheels': 0,
            'detector_confidence': 0.0,
            'wheel_only_mode': self.config.wheel_only_mode,
            'confirmed_pair': self.confirmed,
        }

        # Explicit test-only bypass of all car-side association logic.
        if self.config.wheel_only_mode:
            return self._update_wheel_only(wheels, now, result)

        candidates, orphans = self._associate(cars, wheels)
        index, switched = self._select(cars, now)

        result.update(
            candidates=candidates,
            orphans=orphans,
            primary_index=index,
            primary_switched=switched,
        )

        if index is None:
            if not self.confirmed:
                return result

            entries, used_indices = self._update_confirmed_wheels(wheels, now)

            # Wheels consumed by the confirmed-pair tracker are no longer orphans.
            used_detections = {id(wheels[i]) for i in used_indices}
            result['orphans'] = [
                wheel for wheel in orphans
                if id(wheel) not in used_detections
            ]

            if len(entries) != 2:
                result['state'] = 'CONFIRMED_WHEELS_INCOMPLETE'
                result['wheel_entries'] = entries
                result['real_wheels'] = sum(
                    not entry['held'] for entry in entries
                )
                result['held_wheels'] = len(entries) - result['real_wheels']
                return result

            a, b = sorted(
                [center(entry['detection'].box) for entry in entries]
            )
            wheel_center = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            angle = math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))

            # Wheel-only centre estimate: deliberately no car-bbox weighting.
            fused_center = wheel_center

            alpha = self.config.ema_alpha
            self.ema = fused_center if self.ema is None else tuple(
                alpha * fused_center[i] + (1 - alpha) * self.ema[i]
                for i in (0, 1)
            )
            self.history.append(self.ema)
            self.last_observation_time = now

            real_wheels = sum(not entry['held'] for entry in entries)
            held_wheels = len(entries) - real_wheels

            result.update(
                valid=True,
                state=(
                    'CONFIRMED_WHEELS'
                    if held_wheels == 0
                    else 'CONFIRMED_WHEELS_HELD'
                ),
                wheel_entries=entries,
                wheel_center=wheel_center,
                car_center=None,
                fused_center=fused_center,
                ema_center=self.ema,
                wheel_line_angle_deg=angle,
                history=list(self.history),
                real_wheels=real_wheels,
                held_wheels=held_wheels,
                detector_confidence=min(
                    entry['detection'].confidence for entry in entries
                ),
                confirmed_pair=True,
            )
            return result

        car = cars[index]
        entries = self._remember_wheels(
            car, candidates[index]['pair'], now
        )
        self._confirm_pair(car, entries, now)

        
        cc = center(car.box)
        wc = None
        angle = None
        real = sum(not e['held'] for e in entries)
        held = len(entries) - real
        if len(entries) == 2:
            a, b = sorted([center(e['detection'].box) for e in entries])
            wc = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            angle = math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))
        if wc is not None:
            weight = self.config.car_weight
            fused = tuple((1 - weight) * wc[i] + weight * cc[i] for i in (0, 1))
            state = 'CURRENT_FULL' if held == 0 else 'HELD_WHEELS'
            if real == 2:
                self.complete, self.complete_time = fused, now
        elif self.complete is not None and now - self.complete_time <= self.config.fusion_memory_seconds:
            fused = tuple(0.30 * cc[i] + 0.70 * self.complete[i] for i in (0, 1))
            state = 'HISTORY_ASSISTED'
        else:
            fused, state = cc, 'CAR_ONLY'
        alpha = self.config.ema_alpha
        self.ema = fused if self.ema is None else tuple(alpha * fused[i] + (1 - alpha) * self.ema[i] for i in (0, 1))
        self.history.append(self.ema)
        self.last_observation_time = now
        result.update(valid=True, state=state, primary=car, wheel_entries=entries,
                      wheel_center=wc, car_center=cc, fused_center=fused,
                      ema_center=self.ema, wheel_line_angle_deg=angle,
                      history=list(self.history), real_wheels=real, held_wheels=held,
                      detector_confidence=car.confidence)
        return result