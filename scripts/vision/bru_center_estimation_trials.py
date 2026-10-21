#!/usr/bin/env python3
"""
BRU Center Estimator from Video (YOLOv8 ONNX)

Behavior:
- Loads the ONNX model.
- Reads a video.
- For each frame:
    - Runs YOLO inference.
    - Extracts wheel and car_side detections.
    - Computes BRU centers:
        1. Average between wheel centers (only if both wheels detected)
        2. Center of car_side box
        3. Simple average of (1) and (2) if both wheels detected, otherwise just (2)
        4. EMA-smoothed version of (3)
        5. EMA-smoothed version of (1) (wheel-average EMA)
        6. EMA-smoothed version of (2) (car-side EMA)
        7. Weighted combined center: 85% wheel, 15% car-side (if 2 wheels, else car-side only)
        8. EMA of (7)
        9. Weighted combined center: 70% wheel, 30% car-side (if 2 wheels, else car-side only)
        10. EMA of (9)
        11. Adaptive fused center (red cross):
            - Does not exist until at least one detection (wheels or car_side) has occurred
            - If both wheels visible and no car_side: use wheel-average
            - If both wheels and car_side visible: use 85% wheel + 15% car-side
            - If not both wheels visible:
                - If last complete detection had both wheels+car_side:
                    - Use 30% car-side + 70% last fused position when both were visible
                    - OR use 70% wheel + 30% last fused position when both were visible
                - Else: fallback to car-side or wheel-average if available
            - EMA-smoothed over time (only once it starts existing)
    - Draws:
        - Wheel boxes
        - Car_side box
        - Line between the two wheel centers (if both wheels detected)
        - Markers for all BRU centers (with conditions)
        - A clear legend for the markers
        - Red cross for the adaptive fused center (only after first detection)
- Shows real-time video with controls:
    - q: quit
    - s: toggle save video
    - p: pause/unpause
    - a: step backward (when paused)
    - d: step forward (when paused)
- Logs BRU center positions to a CSV for later analysis.
- Saves the video (without timestamp in filename).
"""

from ultralytics import YOLO
import cv2
import numpy as np
import os
import csv
from datetime import datetime


# === CONFIGURATION ===
model_path = '/home/lapoveca/swas-ros2/models/first_yolov8_24092026.onnx'
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/white_car_vid.mp4'

class_names = ['car_side', 'wheel']  # 0: car_side, 1: wheel
conf_thresh = 0.25

output_dir = '/home/lapoveca/swas-ros2/scripts/vision/output/'
output_video_name = 'bru_center_output.mp4'  # no timestamp
output_csv_name = 'bru_centers.csv'

# EMA smoothing factor (0 < alpha <= 1); smaller = smoother, slower response
EMA_ALPHA = 0.2

# Weighted combined center configs (car_side weight, wheel weight)
WEIGHT_15_CAR = 0.15  # 15% car-side, 85% wheels
WEIGHT_30_CAR = 0.30  # 30% car-side, 70% wheels
# =====================


def compute_wheel_center_avg(wheel_boxes):
    """
    Compute average center of all wheel bounding boxes.
    wheel_boxes: list of (x1, y1, x2, y2)
    Returns (cx, cy) or None if no wheels.
    """
    if len(wheel_boxes) == 0:
        return None
    centers = []
    for (x1, y1, x2, y2) in wheel_boxes:
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        centers.append((cx, cy))
    cx_avg = sum(c[0] for c in centers) / len(centers)
    cy_avg = sum(c[1] for c in centers) / len(centers)
    return (cx_avg, cy_avg)


def compute_car_side_center(car_side_boxes):
    """
    Compute center of car_side box.
    If multiple car_side detections, use the first (or you could average).
    Returns (cx, cy) or None.
    """
    if len(car_side_boxes) == 0:
        return None
    # Use first detection
    x1, y1, x2, y2 = car_side_boxes[0]
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    return (cx, cy)


def draw_point_marker(frame, pt, color, radius=6, thickness=-1, label=None):
    """
    Draw a filled circle marker with optional label.
    pt: (x, y) in image coordinates
    """
    if pt is None:
        return
    x, y = pt
    cv2.circle(frame, (int(x), int(y)), radius, color, thickness)
    if label is not None:
        cv2.putText(
            frame, label, (int(x) + 8, int(y)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
        )


def draw_cross_marker(frame, pt, color, size=8, thickness=2):
    """
    Draw a red cross marker at pt.
    pt: (x, y) in image coordinates
    """
    if pt is None:
        return
    x, y = pt
    # Draw two diagonal lines forming a cross
    half = size // 2
    cv2.line(frame, (int(x - half), int(y - half)),
             (int(x + half), int(y + half)), color, thickness)
    cv2.line(frame, (int(x + half), int(y - half)),
             (int(x - half), int(y + half)), color, thickness)


def draw_legend(frame):
    """
    Draw a legend explaining the markers.
    """
    # Legend position and spacing
    x0 = 10
    y0 = 60
    line_height = 20
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.42
    thickness = 1

    lines = [
        ("Blue solid circle", (255, 0, 0), "Wheel-average center (both wheels detected)"),
        ("Blue outlined circle", (255, 0, 0), "EMA of wheel-average center (both wheels detected)"),
        ("Cyan solid circle", (255, 255, 0), "Car-side center"),
        ("Cyan outlined circle", (255, 255, 0), "EMA of car-side center"),
        ("Yellow solid circle", (0, 255, 255), "Combined center (50% wheel + 50% car-side if 2 wheels, else car-side only)"),
        ("Yellow outlined circle", (0, 255, 255), "EMA of combined center"),
        ("White solid circle", (255, 255, 255), "Combined center (85% wheel + 15% car-side if 2 wheels, else car-side only)"),
        ("White outlined circle", (255, 255, 255), "EMA of 85/15 combined center"),
        ("Orange solid circle", (0, 165, 255), "Combined center (70% wheel + 30% car-side if 2 wheels, else car-side only)"),
        ("Orange outlined circle", (0, 165, 255), "EMA of 70/30 combined center"),
        ("Red cross", (0, 0, 255), "Adaptive fused center (see logic in code)"),
        ("Green line", (0, 255, 0), "Line between two wheel centers (if both detected)"),
    ]

    for i, (label, color, description) in enumerate(lines):
        y = y0 + i * line_height

        # Draw small marker sample
        if "line" in label:
            # Draw a short line segment
            x_start = x0 + 5
            x_end = x0 + 20
            yy = y - 4
            cv2.line(frame, (x_start, yy), (x_end, yy), color, 1)
        elif "cross" in label:
            # Draw a small cross
            cx = x0 + 10
            cy = y - 4
            half = 4
            cv2.line(frame, (cx - half, cy - half), (cx + half, cy + half), color, 2)
            cv2.line(frame, (cx + half, cy - half), (cx - half, cy + half), color, 2)
        else:
            cv2.circle(frame, (x0 + 10, y - 4), 5, color, -1)
            if "outlined" in label:
                cv2.circle(frame, (x0 + 10, y - 4), 5, color, 1)

        text = f"{description}"
        cv2.putText(frame, text, (x0 + 25, y), font, font_scale, (255, 255, 255), thickness)


def main():
    os.makedirs(output_dir, exist_ok=True)

    print(f"🔍 Loading ONNX model: {model_path}")
    model = YOLO(model_path)

    print(f"🎥 Loading video: {video_path}")
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"❌ Error: Could not open video {video_path}")
        return

    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"   Video: {width}x{height} @ {fps:.1f} FPS, {total_frames} frames")

    # Output video writer
    output_video_path = os.path.join(output_dir, output_video_name)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    print(f"💾 Output video will be saved to: {output_video_path}")

    # CSV logging
    output_csv_path = os.path.join(output_dir, output_csv_name)
    csv_file = open(output_csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        'frame_index',
        'num_wheels',
        'wheel_center_avg_x', 'wheel_center_avg_y',
        'car_side_center_x', 'car_side_center_y',
        'combined_center_x', 'combined_center_y',
        'ema_combined_center_x', 'ema_combined_center_y',
        'ema_wheel_center_avg_x', 'ema_wheel_center_avg_y',
        'ema_car_side_center_x', 'ema_car_side_center_y',
        'combined_85w_15c_x', 'combined_85w_15c_y',
        'ema_combined_85w_15c_x', 'ema_combined_85w_15c_y',
        'combined_70w_30c_x', 'combined_70w_30c_y',
        'ema_combined_70w_30c_x', 'ema_combined_70w_30c_y',
        'adaptive_fused_x', 'adaptive_fused_y',
        'ema_adaptive_fused_x', 'ema_adaptive_fused_y',
        'last_complete_fused_x', 'last_complete_fused_y',
        'last_complete_has_both'
    ])

    print(f"📄 CSV log will be saved to: {output_csv_path}")
    print(f"\n🚀 Starting interactive processing (conf={conf_thresh})...")
    print("Controls:")
    print("  q - quit")
    print("  s - toggle saving video frames (current: ON)")
    print("  p - pause/unpause")
    print("  a - step backward (when paused)")
    print("  d - step forward (when paused)")

    frame_index = 0
    paused = False
    save_video = True

    # For EMA smoothing
    ema_combined = None  # (x, y) or None (50/50)
    ema_wheel_avg = None  # (x, y) or None
    ema_car_side = None  # (x, y) or None

    ema_combined_85w_15c = None  # 85% wheel, 15% car-side
    ema_combined_70w_30c = None  # 70% wheel, 30% car-side

    # For adaptive fused center (red cross)
    # These do not exist until at least one detection occurs
    last_complete_fused = None  # last fused position when both wheels+car_side were visible
    last_complete_has_both = False
    ema_adaptive_fused = None   # EMA of adaptive fused center (None until first detection)

    # Pre-load all frames into a list if you want backward stepping.
    frames = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)
    cap.release()

    total_frames = len(frames)
    print(f"   Loaded {total_frames} frames into memory for A/D stepping.")

    while frame_index < total_frames:
        frame = frames[frame_index].copy()
        frame_index += 1

        # Run inference
        results = model.predict(frame, conf=conf_thresh, verbose=False)
        detections = results[0].boxes

        wheel_boxes = []
        car_side_boxes = []

        # Collect boxes by class
        for box in detections:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            label = f"{class_names[cls]} {conf:.2f}"

            if cls == 0:  # car_side
                car_side_boxes.append((x1, y1, x2, y2))
                color = (0, 255, 0)  # green
            elif cls == 1:  # wheel
                wheel_boxes.append((x1, y1, x2, y2))
                color = (0, 0, 255)  # red
            else:
                continue

            # Draw bounding box
            cv2.rectangle(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                color, 2
            )

            # Draw label background
            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            cv2.rectangle(
                frame,
                (int(x1), int(y1) - text_h - baseline - 5),
                (int(x1) + text_w, int(y1)),
                color, -1
            )

            # Draw label text
            cv2.putText(
                frame, label, (int(x1), int(y1) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
            )

        # Compute BRU centers
        num_wheels = len(wheel_boxes)
        wheel_center_avg = compute_wheel_center_avg(wheel_boxes)
        car_side_center = compute_car_side_center(car_side_boxes)

        # Require both wheels to use wheel-average in combined centers
        both_wheels_detected = (num_wheels == 2)
        car_side_detected = (car_side_center is not None)

        # 50/50 combined center
        if both_wheels_detected and wheel_center_avg is not None and car_side_center is not None:
            combined_center = (
                (wheel_center_avg[0] + car_side_center[0]) / 2.0,
                (wheel_center_avg[1] + car_side_center[1]) / 2.0
            )
        else:
            combined_center = car_side_center

        # 85% wheel, 15% car-side combined center
        if both_wheels_detected and wheel_center_avg is not None and car_side_center is not None:
            w_w = 1.0 - WEIGHT_15_CAR  # 0.85
            w_c = WEIGHT_15_CAR        # 0.15
            combined_85w_15c = (
                w_w * wheel_center_avg[0] + w_c * car_side_center[0],
                w_w * wheel_center_avg[1] + w_c * car_side_center[1]
            )
        else:
            combined_85w_15c = car_side_center

        # 70% wheel, 30% car-side combined center
        if both_wheels_detected and wheel_center_avg is not None and car_side_center is not None:
            w_w = 1.0 - WEIGHT_30_CAR  # 0.70
            w_c = WEIGHT_30_CAR        # 0.30
            combined_70w_30c = (
                w_w * wheel_center_avg[0] + w_c * car_side_center[0],
                w_w * wheel_center_avg[1] + w_c * car_side_center[1]
            )
        else:
            combined_70w_30c = car_side_center

        # EMA-smoothed combined centers
        if combined_center is not None:
            if ema_combined is None:
                ema_combined = combined_center
            else:
                alpha = EMA_ALPHA
                ema_combined = (
                    alpha * combined_center[0] + (1 - alpha) * ema_combined[0],
                    alpha * combined_center[1] + (1 - alpha) * ema_combined[1]
                )

        if combined_85w_15c is not None:
            if ema_combined_85w_15c is None:
                ema_combined_85w_15c = combined_85w_15c
            else:
                alpha = EMA_ALPHA
                ema_combined_85w_15c = (
                    alpha * combined_85w_15c[0] + (1 - alpha) * ema_combined_85w_15c[0],
                    alpha * combined_85w_15c[1] + (1 - alpha) * ema_combined_85w_15c[1]
                )

        if combined_70w_30c is not None:
            if ema_combined_70w_30c is None:
                ema_combined_70w_30c = combined_70w_30c
            else:
                alpha = EMA_ALPHA
                ema_combined_70w_30c = (
                    alpha * combined_70w_30c[0] + (1 - alpha) * ema_combined_70w_30c[0],
                    alpha * combined_70w_30c[1] + (1 - alpha) * ema_combined_70w_30c[1]
                )

        # EMA-smoothed wheel-average center (only meaningful if both wheels detected)
        if wheel_center_avg is not None:
            if ema_wheel_avg is None:
                ema_wheel_avg = wheel_center_avg
            else:
                alpha = EMA_ALPHA
                ema_wheel_avg = (
                    alpha * wheel_center_avg[0] + (1 - alpha) * ema_wheel_avg[0],
                    alpha * wheel_center_avg[1] + (1 - alpha) * ema_wheel_avg[1]
                )

        # EMA-smoothed car-side center
        if car_side_center is not None:
            if ema_car_side is None:
                ema_car_side = car_side_center
            else:
                alpha = EMA_ALPHA
                ema_car_side = (
                    alpha * car_side_center[0] + (1 - alpha) * ema_car_side[0],
                    alpha * car_side_center[1] + (1 - alpha) * ema_car_side[1]
                )

        # === Adaptive fused center (red cross) ===
        # This marker and its EMA do not exist until at least one detection occurs.
        # Logic:
        # - If both wheels visible and no car_side: use wheel-average
        # - If both wheels and car_side visible: use 85% wheel + 15% car-side
        # - If not both wheels visible:
        #     - If we have a last complete fused point (both wheels+car_side were visible before):
        #         - Use a blend between current best estimate and last complete fused:
        #           - If car_side exists: 30% car-side + 70% last_complete_fused
        #           - Else if wheel_center_avg exists: 70% wheel + 30% last_complete_fused
        #           - Else: just last_complete_fused
        #     - Else: fallback to car_side if available, else wheel_center_avg if available, else None
        # - EMA-smooth this adaptive fused point over time (only once it exists)

        adaptive_fused = None

        # Check if there is any detection at all in this frame
        any_detection = (num_wheels > 0 or car_side_detected)

        if any_detection:
            if both_wheels_detected and wheel_center_avg is not None:
                if not car_side_detected:
                    # Only wheels visible, no car_side
                    adaptive_fused = wheel_center_avg
                else:
                    # Both wheels and car_side visible: 85% wheel, 15% car-side
                    w_w = 1.0 - WEIGHT_15_CAR  # 0.85
                    w_c = WEIGHT_15_CAR        # 0.15
                    adaptive_fused = (
                        w_w * wheel_center_avg[0] + w_c * car_side_center[0],
                        w_w * wheel_center_avg[1] + w_c * car_side_center[1]
                    )
                    # Update last complete fused
                    last_complete_fused = adaptive_fused
                    last_complete_has_both = True
            else:
                # Not both wheels visible
                if last_complete_has_both and last_complete_fused is not None:
                    # Blend with last complete fused
                    if car_side_center is not None:
                        # 30% car-side, 70% last complete fused
                        w_c = 0.30
                        w_l = 0.70
                        adaptive_fused = (
                            w_c * car_side_center[0] + w_l * last_complete_fused[0],
                            w_c * car_side_center[1] + w_l * last_complete_fused[1]
                        )
                    elif wheel_center_avg is not None:
                        # 70% wheel, 30% last complete fused
                        w_w = 0.70
                        w_l = 0.30
                        adaptive_fused = (
                            w_w * wheel_center_avg[0] + w_l * last_complete_fused[0],
                            w_w * wheel_center_avg[1] + w_l * last_complete_fused[1]
                        )
                    else:
                        # No reliable current detection, use last complete fused
                        adaptive_fused = last_complete_fused
                else:
                    # No last complete fused; fallback
                    if car_side_center is not None:
                        adaptive_fused = car_side_center
                    elif wheel_center_avg is not None:
                        adaptive_fused = wheel_center_avg
                    else:
                        adaptive_fused = None

        # EMA-smooth adaptive fused center (only if it exists)
        if adaptive_fused is not None:
            if ema_adaptive_fused is None:
                ema_adaptive_fused = adaptive_fused
            else:
                alpha = EMA_ALPHA
                ema_adaptive_fused = (
                    alpha * adaptive_fused[0] + (1 - alpha) * ema_adaptive_fused[0],
                    alpha * adaptive_fused[1] + (1 - alpha) * ema_adaptive_fused[1]
                )

        # Draw BRU center markers

        # Wheel-average center and its EMA: only if both wheels detected
        if both_wheels_detected and wheel_center_avg is not None:
            # 1. Wheel-average center (blue solid)
            draw_point_marker(
                frame, wheel_center_avg, color=(255, 0, 0),
                radius=6, thickness=-1, label=None
            )

        if both_wheels_detected and ema_wheel_avg is not None:
            # 2. EMA of wheel-average center (blue outlined)
            draw_point_marker(
                frame, ema_wheel_avg, color=(255, 0, 0),
                radius=8, thickness=2, label=None
            )

        # 3. Car-side center (cyan solid)
        draw_point_marker(
            frame, car_side_center, color=(255, 255, 0),
            radius=6, thickness=-1, label=None
        )

        # 4. EMA of car-side center (cyan outlined)
        draw_point_marker(
            frame, ema_car_side, color=(255, 255, 0),
            radius=8, thickness=2, label=None
        )

        # 5. Combined center 50/50 (yellow solid)
        draw_point_marker(
            frame, combined_center, color=(0, 255, 255),
            radius=7, thickness=-1, label=None
        )

        # 6. EMA-smoothed combined center 50/50 (yellow outlined)
        draw_point_marker(
            frame, ema_combined, color=(0, 255, 255),
            radius=9, thickness=2, label=None
        )

        # 7. Combined center 85% wheel / 15% car-side (white solid)
        draw_point_marker(
            frame, combined_85w_15c, color=(255, 255, 255),
            radius=7, thickness=-1, label=None
        )

        # 8. EMA of 85/15 combined center (white outlined)
        draw_point_marker(
            frame, ema_combined_85w_15c, color=(255, 255, 255),
            radius=9, thickness=2, label=None
        )

        # 9. Combined center 70% wheel / 30% car-side (orange solid)
        # BGR: (0, 165, 255)
        draw_point_marker(
            frame, combined_70w_30c, color=(0, 165, 255),
            radius=7, thickness=-1, label=None
        )

        # 10. EMA of 70/30 combined center (orange outlined)
        draw_point_marker(
            frame, ema_combined_70w_30c, color=(0, 165, 255),
            radius=9, thickness=2, label=None
        )

        # 11. Adaptive fused center (red cross) – only drawn if it exists
        draw_cross_marker(
            frame, ema_adaptive_fused, color=(0, 0, 255), size=10, thickness=2
        )

        # Draw line between the two wheel centers if both wheels detected
        if both_wheels_detected and len(wheel_boxes) == 2:
            w1 = wheel_boxes[0]
            w2 = wheel_boxes[1]
            c1 = ((w1[0] + w1[2]) / 2.0, (w1[1] + w1[3]) / 2.0)
            c2 = ((w2[0] + w2[2]) / 2.0, (w2[1] + w2[3]) / 2.0)
            cv2.line(
                frame,
                (int(c1[0]), int(c1[1])),
                (int(c2[0]), int(c2[1])),
                (0, 255, 0),  # green
                2
            )

        # Info overlay
        status_text = (
            f"Frame: {frame_index}/{total_frames} | Dets: {len(detections)} | "
            f"Wheels: {num_wheels} | Pause: {paused} | Save: {save_video}"
        )
        cv2.putText(
            frame, status_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
        )

        # Draw legend
        draw_legend(frame)

        # Write frame to output video if saving
        if save_video:
            out.write(frame)

        # Log to CSV
        csv_writer.writerow([
            frame_index,
            num_wheels,
            wheel_center_avg[0] if wheel_center_avg else None,
            wheel_center_avg[1] if wheel_center_avg else None,
            car_side_center[0] if car_side_center else None,
            car_side_center[1] if car_side_center else None,
            combined_center[0] if combined_center else None,
            combined_center[1] if combined_center else None,
            ema_combined[0] if ema_combined else None,
            ema_combined[1] if ema_combined else None,
            ema_wheel_avg[0] if ema_wheel_avg else None,
            ema_wheel_avg[1] if ema_wheel_avg else None,
            ema_car_side[0] if ema_car_side else None,
            ema_car_side[1] if ema_car_side else None,
            combined_85w_15c[0] if combined_85w_15c else None,
            combined_85w_15c[1] if combined_85w_15c else None,
            ema_combined_85w_15c[0] if ema_combined_85w_15c else None,
            ema_combined_85w_15c[1] if ema_combined_85w_15c else None,
            combined_70w_30c[0] if combined_70w_30c else None,
            combined_70w_30c[1] if combined_70w_30c else None,
            ema_combined_70w_30c[0] if ema_combined_70w_30c else None,
            ema_combined_70w_30c[1] if ema_combined_70w_30c else None,
            adaptive_fused[0] if adaptive_fused else None,
            adaptive_fused[1] if adaptive_fused else None,
            ema_adaptive_fused[0] if ema_adaptive_fused else None,
            ema_adaptive_fused[1] if ema_adaptive_fused else None,
            last_complete_fused[0] if last_complete_fused else None,
            last_complete_fused[1] if last_complete_fused else None,
            1 if last_complete_has_both else 0
        ])

        # Show frame
        cv2.imshow("BRU Center Estimator", frame)

        # Handle controls
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
            # Step backward
            if frame_index > 1:
                frame_index -= 2  # will be incremented at top of loop
        elif key == ord('d') and paused:
            # Step forward (just let loop continue; maybe step extra if desired)
            pass

    # Cleanup
    out.release()
    csv_file.close()
    cv2.destroyAllWindows()

    print(f"\n✅ Processing complete!")
    print(f"   Processed: {frame_index} frames")
    print(f"   Output video saved to: {output_video_path}")
    print(f"   CSV log saved to: {output_csv_path}")


if __name__ == '__main__':
    main()