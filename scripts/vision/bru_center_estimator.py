#!/usr/bin/env python3
"""
BRU Center Estimator from Video (YOLOv8 ONNX) - Minimal Real-Time + Wheel Line Angle
+ Wheel Memory + Multi-Car Wheel/Car-Side Association + Primary Car Selection
+ Detection Dedup (NMS-style) + Corrected Wheel-Side Selection

Changes vs. previous version:
- Added a generic IoU-based dedup pass (NMS-style) applied to BOTH car_side and
  wheel detections right after they come out of the model, before any
  association/primary logic runs. This removes duplicate/near-duplicate boxes
  for the same physical object, which was the root cause of:
    (a) a wheel appearing simultaneously as "primary/held" and as
        "secondary/other car" (duplicate wheel detection), and
    (b) the primary car losing its wheels for a stretch of frames because a
        duplicate car_side box greedily claimed them first (duplicate
        car_side detection).
- Fixed the left/right wheel candidate selection within
  associate_wheels_to_car_sides(): previously the tie-break picked the
  candidate closest in VERTICAL position to the car_side center, which is not
  physically meaningful. It now picks the candidate closest in HORIZONTAL
  position to the car_side center on each side - i.e. the two wheels nearest
  the car's own center - which correctly excludes outlier/extra wheels
  further from the center as secondary.

Behavior (everything else unchanged from before):
- Loads the ONNX model.
- Reads a video.
- For each frame (real-time, no preloading):
    - Runs YOLO inference.
    - Extracts ALL wheel and car_side detections, deduplicates them.
    - Associates wheels with car_side boxes (10% padded bbox, one wheel
      required on each side of the car_side center; extras become secondary,
      unmatched wheels become orphans).
    - Selects a primary car (sticky, size-based, with memory/hysteresis).
    - Applies short-term wheel memory (scoped to the primary car's pair only,
      reset when the primary switches to a different physical car).
    - Computes wheel-average center, car-side center, adaptive fused center
      (EMA-smoothed red cross with history), and wheel line angle.
    - Draws everything with clear color coding.
- Shows real-time video with controls: q, s, p, a, d.
- Logs to CSV. Saves the output video (no timestamp in filename).
"""

from ultralytics import YOLO
import cv2
import numpy as np
import os
import csv
from datetime import datetime
import math


# === CONFIGURATION ===
model_path = '/home/lapoveca/swas-ros2/models/first_yolov8_24092026.onnx'
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/two_cars_vid.mp4'
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/white_car_vid.mp4'
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/many_cars_vid.mp4'


class_names = ['car_side', 'wheel']  # 0: car_side, 1: wheel
conf_thresh = 0.30

output_dir = '/home/lapoveca/swas-ros2/scripts/vision/output/'
output_video_name = 'bru_center_output_estimates5.mp4'
output_csv_name = 'bru_centers_estimates5.csv'

# EMA smoothing factor (0 < alpha <= 1); smaller = smoother, slower response
EMA_ALPHA = 0.2

# Weighted combined center configs (car_side weight, wheel weight)
WEIGHT_15_CAR = 0.03  # 10% car-side, 90% wheels

# History length for yellow crosses (last N frames of red cross)
HISTORY_LENGTH = 10

# How many consecutive frames a wheel detection can be "missed" before
# the slot is dropped (memory scoped to the primary car's wheel pair).
WHEEL_MEMORY_FRAMES = 7

# --- Detection dedup (NMS-style), applied to raw detections before anything else ---
DEDUP_IOU_THRESH = 0.4

# --- Wheel <-> car_side association ---
CAR_PAD_RATIO = 0.05

# --- Primary car selection / hysteresis ---
PRIMARY_LOST_THRESHOLD = 20
PRIMARY_SWITCH_SIZE_RATIO = 1.5
PRIMARY_SWITCH_SIZE_FRAMES = 15
# =====================


def box_center(box):
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def box_area(box):
    x1, y1, x2, y2 = box
    return max(0.0, (x2 - x1)) * max(0.0, (y2 - y1))


def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def expand_box(box, pad_ratio_w, pad_ratio_h):
    x1, y1, x2, y2 = box
    w = x2 - x1
    h = y2 - y1
    return (x1 - pad_ratio_w * w, y1 - pad_ratio_h * h,
             x2 + pad_ratio_w * w, y2 + pad_ratio_h * h)


def point_in_box(pt, box):
    x, y = pt
    x1, y1, x2, y2 = box
    return x1 <= x <= x2 and y1 <= y <= y2


def iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = box_area(box_a)
    area_b = box_area(box_b)
    union = area_a + area_b - inter
    if union <= 0:
        return 0.0
    return inter / union


def dedup_boxes(boxes, iou_thresh=DEDUP_IOU_THRESH):
    """
    Generic NMS-style dedup: for boxes that overlap more than iou_thresh,
    keep only the larger one. Order-independent, no class/dataset assumptions.
    """
    if not boxes:
        return []
    boxes_sorted = sorted(boxes, key=box_area, reverse=True)
    kept = []
    for b in boxes_sorted:
        is_dup = False
        for k in kept:
            if iou(b, k) > iou_thresh:
                is_dup = True
                break
        if not is_dup:
            kept.append(b)
    return kept


def compute_wheel_center_avg(wheel_boxes):
    if len(wheel_boxes) != 2:
        return None
    centers = [box_center(b) for b in wheel_boxes]
    cx_avg = sum(c[0] for c in centers) / len(centers)
    cy_avg = sum(c[1] for c in centers) / len(centers)
    return (cx_avg, cy_avg)


def compute_car_side_center(car_side_boxes):
    if len(car_side_boxes) == 0:
        return None
    return box_center(car_side_boxes[0])


def associate_wheels_to_car_sides(car_side_boxes, wheel_boxes):
    """
    For each car_side box, find wheels within its expanded bbox and split
    them into left/right of the car_side center. On each side, the
    candidate closest to the car_side's HORIZONTAL center is chosen as the
    car's wheel (this correctly favors wheels near the car's own center
    over outlier wheels further out). Any extra candidates near that car
    are 'secondary'. Wheels inside no car_side's expanded box are orphans.
    """
    candidates = []
    assigned = set()

    for cs_box in car_side_boxes:
        cs_center = box_center(cs_box)
        expanded = expand_box(cs_box, CAR_PAD_RATIO, CAR_PAD_RATIO)

        left_c = []
        right_c = []
        for i, wb in enumerate(wheel_boxes):
            if i in assigned:
                continue
            wc = box_center(wb)
            if point_in_box(wc, expanded):
                if wc[0] < cs_center[0]:
                    left_c.append(i)
                else:
                    right_c.append(i)

        # Pick the candidate on each side closest to the car_side's
        # horizontal center (not vertical - that was the bug).
        chosen_left = (
            max(left_c, key=lambda i: box_center(wheel_boxes[i])[0])
            if left_c else None
        )
        chosen_right = (
            min(right_c, key=lambda i: box_center(wheel_boxes[i])[0])
            if right_c else None
        )

        pair = []
        used_now = []
        if chosen_left is not None:
            pair.append(wheel_boxes[chosen_left])
            used_now.append(chosen_left)
        if chosen_right is not None:
            pair.append(wheel_boxes[chosen_right])
            used_now.append(chosen_right)

        secondary_idx = [i for i in (left_c + right_c) if i not in used_now]

        for i in used_now + secondary_idx:
            assigned.add(i)

        candidates.append({
            'car_side_box': cs_box,
            'wheel_pair': pair,
            'secondary_wheels': [wheel_boxes[i] for i in secondary_idx],
        })

    orphan_wheels = [wb for i, wb in enumerate(wheel_boxes) if i not in assigned]
    return candidates, orphan_wheels


def select_primary_car_side(car_side_boxes, primary_state):
    primary_state['just_switched'] = False

    if not car_side_boxes:
        primary_state['lost_counter'] += 1
        if primary_state['lost_counter'] > PRIMARY_LOST_THRESHOLD:
            primary_state['box'] = None
            primary_state['area'] = None
        return None

    areas = [box_area(b) for b in car_side_boxes]

    if primary_state['box'] is None:
        best_idx = max(range(len(car_side_boxes)), key=lambda i: areas[i])
        primary_state['box'] = car_side_boxes[best_idx]
        primary_state['area'] = areas[best_idx]
        primary_state['lost_counter'] = 0
        primary_state['size_challenge_counter'] = 0
        primary_state['just_switched'] = True
        return best_idx

    prev_center = box_center(primary_state['box'])
    dists = [dist(box_center(b), prev_center) for b in car_side_boxes]
    match_idx = min(range(len(car_side_boxes)), key=lambda i: dists[i])

    pw = primary_state['box'][2] - primary_state['box'][0]
    ph = primary_state['box'][3] - primary_state['box'][1]
    max_dist = 0.75 * math.hypot(pw, ph)
    matched = dists[match_idx] <= max_dist

    if matched:
        primary_state['box'] = car_side_boxes[match_idx]
        primary_state['area'] = areas[match_idx]
        primary_state['lost_counter'] = 0
    else:
        primary_state['lost_counter'] += 1

    competitor_areas = [areas[i] for i in range(len(car_side_boxes))
                         if not (matched and i == match_idx)]
    if competitor_areas and primary_state['area']:
        biggest_competitor = max(competitor_areas)
        if biggest_competitor > PRIMARY_SWITCH_SIZE_RATIO * primary_state['area']:
            primary_state['size_challenge_counter'] += 1
        else:
            primary_state['size_challenge_counter'] = 0
    else:
        primary_state['size_challenge_counter'] = 0

    if (primary_state['lost_counter'] > PRIMARY_LOST_THRESHOLD or
            primary_state['size_challenge_counter'] > PRIMARY_SWITCH_SIZE_FRAMES):
        best_idx = max(range(len(car_side_boxes)), key=lambda i: areas[i])
        old_box = primary_state['box']
        primary_state['box'] = car_side_boxes[best_idx]
        primary_state['area'] = areas[best_idx]
        primary_state['lost_counter'] = 0
        primary_state['size_challenge_counter'] = 0
        primary_state['just_switched'] = (old_box != car_side_boxes[best_idx])
        return best_idx

    return match_idx if matched else None


def update_wheel_memory(wheel_slots, detected_wheel_boxes):
    unmatched_detections = list(detected_wheel_boxes)
    matched_slot_indices = set()

    for det_box in list(unmatched_detections):
        det_center = box_center(det_box)
        best_slot_idx = None
        best_dist = None
        for idx, slot in enumerate(wheel_slots):
            if idx in matched_slot_indices:
                continue
            slot_center = box_center(slot['box'])
            d = dist(det_center, slot_center)
            if best_dist is None or d < best_dist:
                best_dist = d
                best_slot_idx = idx

        if best_slot_idx is not None:
            wheel_slots[best_slot_idx]['box'] = det_box
            wheel_slots[best_slot_idx]['age'] = 0
            matched_slot_indices.add(best_slot_idx)
            unmatched_detections.remove(det_box)

    for det_box in unmatched_detections:
        if len(wheel_slots) < 2:
            wheel_slots.append({'box': det_box, 'age': 0})
        else:
            oldest_idx = max(range(len(wheel_slots)), key=lambda i: wheel_slots[i]['age'])
            if wheel_slots[oldest_idx]['age'] > 0:
                wheel_slots[oldest_idx] = {'box': det_box, 'age': 0}

    for idx, slot in enumerate(wheel_slots):
        if idx not in matched_slot_indices:
            slot['age'] += 1

    wheel_slots[:] = [s for s in wheel_slots if s['age'] <= WHEEL_MEMORY_FRAMES]

    effective_boxes = [(s['box'], s['age'] > 0) for s in wheel_slots]
    return wheel_slots, effective_boxes


def draw_point_marker(frame, pt, color, radius=6, thickness=-1, label=None):
    if pt is None:
        return
    x, y = pt
    cv2.circle(frame, (int(x), int(y)), radius, color, thickness)
    if label is not None:
        cv2.putText(frame, label, (int(x) + 8, int(y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def draw_cross_marker(frame, pt, color, size=8, thickness=2):
    if pt is None:
        return
    x, y = pt
    half = size // 2
    cv2.line(frame, (int(x - half), int(y - half)),
              (int(x + half), int(y + half)), color, thickness)
    cv2.line(frame, (int(x + half), int(y - half)),
              (int(x - half), int(y + half)), color, thickness)


def draw_box_with_label(frame, box, color, label):
    x1, y1, x2, y2 = box
    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
    (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(frame,
                  (int(x1), int(y1) - text_h - baseline - 5),
                  (int(x1) + text_w, int(y1)),
                  color, -1)
    cv2.putText(frame, label, (int(x1), int(y1) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def main():
    os.makedirs(output_dir, exist_ok=True)

    print(f"🔍 Loading ONNX model: {model_path}")
    model = YOLO(model_path)

    print(f"🎥 Loading video: {video_path}")
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"❌ Error: Could not open video {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"   Video: {width}x{height} @ {fps:.1f} FPS, {total_frames} frames")

    output_video_path = os.path.join(output_dir, output_video_name)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    print(f"💾 Output video will be saved to: {output_video_path}")

    output_csv_path = os.path.join(output_dir, output_csv_name)
    csv_file = open(output_csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        'frame_index',
        'num_car_sides_detected',
        'num_wheels_detected_total',
        'num_secondary_wheels',
        'num_wheels',
        'num_wheels_real',
        'num_wheels_held',
        'primary_car_area',
        'wheel_center_avg_x', 'wheel_center_avg_y',
        'car_side_center_x', 'car_side_center_y',
        'adaptive_fused_x', 'adaptive_fused_y',
        'ema_adaptive_fused_x', 'ema_adaptive_fused_y',
        'last_complete_fused_x', 'last_complete_fused_y',
        'last_complete_has_both',
        'wheel_line_angle_deg'
    ])

    print(f"📄 CSV log will be saved to: {output_csv_path}")
    print(f"\n🚀 Starting real-time processing (conf={conf_thresh})...")
    print("Controls:")
    print("  q - quit")
    print("  s - toggle saving video frames (current: ON)")
    print("  p - pause/unpause")
    print("  a - step backward (when paused)")
    print("  d - step forward (when paused)")

    frame_index = 0
    paused = False
    save_video = True

    last_complete_fused = None
    last_complete_has_both = False
    ema_adaptive_fused = None
    adaptive_fused_history = []

    wheel_slots = []

    primary_state = {
        'box': None,
        'area': None,
        'lost_counter': 0,
        'size_challenge_counter': 0,
        'just_switched': False,
    }

    frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = frame.copy()
        frame_index += 1
        frames.append(frame)

        results = model.predict(frame, conf=conf_thresh, verbose=False)
        detections = results[0].boxes

        car_side_boxes_raw = []
        wheel_boxes_raw = []

        for box in detections:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            if cls == 0:
                car_side_boxes_raw.append((x1, y1, x2, y2))
            elif cls == 1:
                wheel_boxes_raw.append((x1, y1, x2, y2))

        # === Dedup raw detections (NMS-style) before any association logic ===
        car_side_boxes_all = dedup_boxes(car_side_boxes_raw, DEDUP_IOU_THRESH)
        wheel_boxes_all = dedup_boxes(wheel_boxes_raw, DEDUP_IOU_THRESH)

        # === Associate wheels to car_sides (multi-car aware) ===
        candidates, orphan_wheels = associate_wheels_to_car_sides(
            car_side_boxes_all, wheel_boxes_all
        )

        # === Select primary car (sticky, size-based, with memory) ===
        primary_idx = select_primary_car_side(car_side_boxes_all, primary_state)

        if primary_idx is not None:
            primary_candidate = candidates[primary_idx]
            primary_car_side_box = primary_candidate['car_side_box']
            primary_wheel_pair_raw = primary_candidate['wheel_pair']
            primary_secondary_wheels = primary_candidate['secondary_wheels']
        else:
            primary_car_side_box = None
            primary_wheel_pair_raw = []
            primary_secondary_wheels = []

        if primary_state.get('just_switched'):
            wheel_slots = []

        wheel_slots, effective_wheel_entries = update_wheel_memory(
            wheel_slots, primary_wheel_pair_raw
        )
        wheel_boxes = [box for (box, held) in effective_wheel_entries]

        car_side_boxes = [primary_car_side_box] if primary_car_side_box is not None else []

        num_wheels_real = sum(1 for (_, held) in effective_wheel_entries if not held)
        num_wheels_held = sum(1 for (_, held) in effective_wheel_entries if held)

        # === Drawing: non-primary / excluded detections first (dim) ===
        for i, cs_box in enumerate(car_side_boxes_all):
            if primary_idx is not None and i == primary_idx:
                continue
            draw_box_with_label(frame, cs_box, (90, 90, 90), "car_side (other)")

        for wb in primary_secondary_wheels:
            draw_box_with_label(frame, wb, (160, 32, 160), "wheel (secondary)")

        for wb in orphan_wheels:
            draw_box_with_label(frame, wb, (160, 32, 160), "wheel (orphan)")

        for i, cand in enumerate(candidates):
            if primary_idx is not None and i == primary_idx:
                continue
            for wb in cand['wheel_pair']:
                draw_box_with_label(frame, wb, (120, 60, 60), "wheel (other car)")
            for wb in cand['secondary_wheels']:
                draw_box_with_label(frame, wb, (160, 32, 160), "wheel (secondary)")

        if primary_car_side_box is not None:
            draw_box_with_label(frame, primary_car_side_box, (0, 255, 0), "car_side (primary)")

        for (box, held) in effective_wheel_entries:
            color = (0, 165, 255) if held else (0, 0, 255)
            tag = "wheel (held)" if held else "wheel"
            draw_box_with_label(frame, box, color, tag)

        num_wheels = len(wheel_boxes)
        wheel_center_avg = compute_wheel_center_avg(wheel_boxes)
        car_side_center = compute_car_side_center(car_side_boxes)

        both_wheels_detected = (num_wheels == 2)
        car_side_detected = (car_side_center is not None)

        wheel_line_angle_deg = None
        if both_wheels_detected and len(wheel_boxes) == 2:
            c1 = box_center(wheel_boxes[0])
            c2 = box_center(wheel_boxes[1])
            dx = c2[0] - c1[0]
            dy = c2[1] - c1[1]
            wheel_line_angle_rad = math.atan2(-dy, dx)
            angle = math.degrees(wheel_line_angle_rad)
            while angle <= -180:
                angle += 360
            while angle > 180:
                angle -= 360
            if angle > 90:
                angle = 180 - angle
            elif angle < -90:
                angle = -180 - angle
            wheel_line_angle_deg = angle

        adaptive_fused = None
        start_condition = both_wheels_detected or car_side_detected

        if start_condition:
            complete_detection_now = both_wheels_detected and car_side_detected

            if complete_detection_now and wheel_center_avg is not None and car_side_center is not None:
                w_w = 1.0 - WEIGHT_15_CAR
                w_c = WEIGHT_15_CAR
                adaptive_fused = (
                    w_w * wheel_center_avg[0] + w_c * car_side_center[0],
                    w_w * wheel_center_avg[1] + w_c * car_side_center[1]
                )
                last_complete_fused = adaptive_fused
                last_complete_has_both = True

            if last_complete_has_both and last_complete_fused is not None:
                if not complete_detection_now:
                    if both_wheels_detected and wheel_center_avg is not None and not car_side_detected:
                        adaptive_fused = wheel_center_avg
                    elif not both_wheels_detected:
                        if car_side_center is not None:
                            w_c = 0.30
                            w_l = 0.70
                            adaptive_fused = (
                                w_c * car_side_center[0] + w_l * last_complete_fused[0],
                                w_c * car_side_center[1] + w_l * last_complete_fused[1]
                            )
                        elif wheel_center_avg is not None:
                            w_w = 0.70
                            w_l = 0.30
                            adaptive_fused = (
                                w_w * wheel_center_avg[0] + w_l * last_complete_fused[0],
                                w_w * wheel_center_avg[1] + w_l * last_complete_fused[1]
                            )
                        else:
                            adaptive_fused = last_complete_fused

        if adaptive_fused is not None:
            if ema_adaptive_fused is None:
                ema_adaptive_fused = adaptive_fused
            else:
                alpha = EMA_ALPHA
                ema_adaptive_fused = (
                    alpha * adaptive_fused[0] + (1 - alpha) * ema_adaptive_fused[0],
                    alpha * adaptive_fused[1] + (1 - alpha) * ema_adaptive_fused[1]
                )

        if ema_adaptive_fused is not None:
            adaptive_fused_history.append(ema_adaptive_fused)
            if len(adaptive_fused_history) > HISTORY_LENGTH:
                adaptive_fused_history.pop(0)

        for hist_pt in adaptive_fused_history[:-1]:
            draw_cross_marker(frame, hist_pt, color=(0, 255, 255), size=6, thickness=1)

        if both_wheels_detected and wheel_center_avg is not None:
            draw_point_marker(frame, wheel_center_avg, color=(255, 0, 0),
                               radius=6, thickness=-1, label=None)

        draw_point_marker(frame, car_side_center, color=(255, 255, 0),
                           radius=6, thickness=-1, label=None)

        draw_cross_marker(frame, ema_adaptive_fused, color=(0, 0, 255), size=10, thickness=2)

        if both_wheels_detected and len(wheel_boxes) == 2:
            c1 = box_center(wheel_boxes[0])
            c2 = box_center(wheel_boxes[1])
            cv2.line(frame, (int(c1[0]), int(c1[1])), (int(c2[0]), int(c2[1])),
                      (0, 255, 0), 2)

        total_frames_current = len(frames)
        status_text = (
            f"Frame: {frame_index}/{total_frames_current} | Dets: {len(detections)} | "
            f"Cars: {len(car_side_boxes_all)} | "
            f"Wheels: {num_wheels} (real:{num_wheels_real} held:{num_wheels_held}) | "
            f"Pause: {paused} | Save: {save_video}"
        )
        cv2.putText(frame, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if wheel_line_angle_deg is not None:
            angle_text = f"Wheel line angle: {wheel_line_angle_deg:.1f} deg"
            cv2.putText(frame, angle_text, (10, height - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if save_video:
            out.write(frame)

        csv_writer.writerow([
            frame_index,
            len(car_side_boxes_all),
            len(wheel_boxes_all),
            len(primary_secondary_wheels),
            num_wheels,
            num_wheels_real,
            num_wheels_held,
            primary_state['area'] if primary_state['area'] else None,
            wheel_center_avg[0] if wheel_center_avg else None,
            wheel_center_avg[1] if wheel_center_avg else None,
            car_side_center[0] if car_side_center else None,
            car_side_center[1] if car_side_center else None,
            adaptive_fused[0] if adaptive_fused else None,
            adaptive_fused[1] if adaptive_fused else None,
            ema_adaptive_fused[0] if ema_adaptive_fused else None,
            ema_adaptive_fused[1] if ema_adaptive_fused else None,
            last_complete_fused[0] if last_complete_fused else None,
            last_complete_fused[1] if last_complete_fused else None,
            1 if last_complete_has_both else 0,
            wheel_line_angle_deg if wheel_line_angle_deg is not None else None
        ])

        cv2.imshow("BRU Center Estimator (Multi-Car Aware + Dedup)", frame)

        key = cv2.waitKey(0 if paused else 1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            save_video = not save_video
            print(f"   Save video toggled: {save_video}")
        elif key == ord('p'):
            paused = not paused
            print(f"   Pause toggled: {paused}")
        elif key == ord('a') and paused:
            if frame_index > 1:
                frame_index -= 2
                if len(frames) > 0:
                    frames.pop()
                if len(adaptive_fused_history) > 0:
                    adaptive_fused_history.pop()
        elif key == ord('d') and paused:
            pass

    cap.release()
    out.release()
    csv_file.close()
    cv2.destroyAllWindows()

    print(f"\n✅ Processing complete!")
    print(f"   Processed: {frame_index} frames")
    print(f"   Output video saved to: {output_video_path}")
    print(f"   CSV log saved to: {output_csv_path}")


if __name__ == '__main__':
    main()
