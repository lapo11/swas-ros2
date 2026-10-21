#!/usr/bin/env python3
"""
BRU Center Estimator from Video (YOLOv8 ONNX) - Minimal Real-Time + Wheel Line Angle

Behavior:
- Loads the ONNX model.
- Reads a video.
- For each frame (real-time, no preloading):
    - Runs YOLO inference.
    - Extracts wheel and car_side detections.
    - Computes:
        1. Center between the two wheel centers (only if both wheels detected)
        2. Car-side center
        3. Adaptive fused center (red cross):
            - Starts existing as soon as:
                - both wheels are detected (even without car_side), OR
                - car_side is detected (even without both wheels)
            - If both wheels visible and no car_side: use wheel-average
            - If both wheels and car_side visible: use 90% wheel + 10% car-side
            - If not both wheels visible:
                - If last complete detection had both wheels+car_side:
                    - Use 30% car-side + 70% last fused position when both were visible
                    - OR use 70% wheel + 30% last fused position when both were visible
                - Else: no adaptive fused center
            - EMA-smoothed over time (only once it starts existing)
        4. Wheel line angle estimate from the line between the two wheel centers (if both detected)
           - Incline from horizontal in the camera frame
           - Flat line = 0 deg
           - Positive = counter-clockwise
           - Wrapped to [-90, 90] deg
    - Draws:
        - Wheel boxes (red)
        - Car_side box (green)
        - Center between the two wheel centers (blue solid circle)
        - Car-side center (cyan solid circle)
        - Red cross for the current adaptive fused center (once active)
        - Yellow smaller crosses for the last 10 frames of the red cross (if applicable)
        - Green line between the two wheel centers (if both detected)
        - Wheel line angle text overlay (if both wheels detected)
- Shows real-time video with controls:
    - q: quit
    - s: toggle save video
    - p: pause/unpause
    - a: step backward (when paused)
    - d: step forward (when paused)
- Logs BRU center positions and wheel line angle to a CSV for later analysis.
- Saves the video (without timestamp in filename).
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
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/white_car_vid.mp4'
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/two_cars_vid.mp4'

class_names = ['car_side', 'wheel']  # 0: car_side, 1: wheel
conf_thresh = 0.18

output_dir = '/home/lapoveca/swas-ros2/scripts/vision/output/'
output_video_name = 'bru_center_output_estimates2.mp4' 
output_csv_name = 'bru_centers_estimates2.csv'

# EMA smoothing factor (0 < alpha <= 1); smaller = smoother, slower response
EMA_ALPHA = 0.2

# Weighted combined center configs (car_side weight, wheel weight)
WEIGHT_15_CAR = 0.10  # 10% car-side, 90% wheels

# History length for yellow crosses (last N frames of red cross)
HISTORY_LENGTH = 10
# =====================


def compute_wheel_center_avg(wheel_boxes):
    """
    Compute average center of all wheel bounding boxes.
    wheel_boxes: list of (x1, y1, x2, y2)
    Returns (cx, cy) or None if not exactly two wheels.
    """
    # Only consider the average if both wheels are detected
    if len(wheel_boxes) != 2:
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
    Draw a cross marker at pt.
    pt: (x, y) in image coordinates
    """
    if pt is None:
        return
    x, y = pt
    half = size // 2
    cv2.line(frame, (int(x - half), int(y - half)),
             (int(x + half), int(y + half)), color, thickness)
    cv2.line(frame, (int(x + half), int(y - half)),
             (int(x - half), int(y + half)), color, thickness)


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

    # For adaptive fused center (red cross)
    last_complete_fused = None  # last fused position when both wheels+car_side were visible
    last_complete_has_both = False
    ema_adaptive_fused = None   # EMA of adaptive fused center (None until first activation)

    # History of adaptive fused positions for yellow crosses (last N frames)
    adaptive_fused_history = []  # list of (x, y), newest appended each frame

    # For backward stepping, we still need to store frames.
    frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = frame.copy()
        frame_index += 1
        frames.append(frame)

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

        both_wheels_detected = (num_wheels == 2)
        car_side_detected = (car_side_center is not None)

        # === Wheel line angle estimation (incline from horizontal) ===
        # Flat line = 0 deg, positive = counter-clockwise, wrapped to [-90, 90]
        wheel_line_angle_deg = None
        if both_wheels_detected and len(wheel_boxes) == 2:
            w1 = wheel_boxes[0]
            w2 = wheel_boxes[1]
            c1 = ((w1[0] + w1[2]) / 2.0, (w1[1] + w1[3]) / 2.0)
            c2 = ((w2[0] + w2[2]) / 2.0, (w2[1] + w2[3]) / 2.0)
            dx = c2[0] - c1[0]
            dy = c2[1] - c1[1]
            # In image coordinates, y increases downward, so negate dy for standard math
            wheel_line_angle_rad = math.atan2(-dy, dx)
            angle = math.degrees(wheel_line_angle_rad)
            # Wrap angle to [-90, 90]
            # First wrap to [-180, 180]
            while angle <= -180:
                angle += 360
            while angle > 180:
                angle -= 360
            # Then map to [-90, 90] using symmetry of the line
            if angle > 90:
                angle = 180 - angle
            elif angle < -90:
                angle = -180 - angle
            wheel_line_angle_deg = angle

        # === Adaptive fused center (red cross) ===
        # Now starts existing as soon as:
        # - both wheels detected (even without car_side), OR
        # - car_side detected (even without both wheels)
        adaptive_fused = None

        # Condition to start the adaptive fused tracker
        start_condition = both_wheels_detected or car_side_detected

        if start_condition:
            # Check if we have a "complete" detection in this frame
            complete_detection_now = both_wheels_detected and car_side_detected

            if complete_detection_now and wheel_center_avg is not None and car_side_center is not None:
                # Both wheels and car_side visible: 90% wheel, 10% car-side
                w_w = 1.0 - WEIGHT_15_CAR  # 0.90
                w_c = WEIGHT_15_CAR        # 0.10
                adaptive_fused = (
                    w_w * wheel_center_avg[0] + w_c * car_side_center[0],
                    w_w * wheel_center_avg[1] + w_c * car_side_center[1]
                )
                # Update last complete fused
                last_complete_fused = adaptive_fused
                last_complete_has_both = True

            # If we already have a last complete fused, allow fallback logic
            if last_complete_has_both and last_complete_fused is not None:
                if not complete_detection_now:
                    # Not both wheels visible or car_side missing
                    if both_wheels_detected and wheel_center_avg is not None and not car_side_detected:
                        # Only wheels visible, no car_side: use wheel-average
                        adaptive_fused = wheel_center_avg
                    elif not both_wheels_detected:
                        # Not both wheels visible
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

        # Update history for yellow crosses (store EMA position each frame if active)
        if ema_adaptive_fused is not None:
            adaptive_fused_history.append(ema_adaptive_fused)
            if len(adaptive_fused_history) > HISTORY_LENGTH:
                adaptive_fused_history.pop(0)

        # Draw BRU center markers

        # Draw yellow smaller crosses for last N frames of red cross (history)
        for i, hist_pt in enumerate(adaptive_fused_history[:-1]):  # exclude current frame
            # Smaller, semi-transparent-ish yellow cross
            draw_cross_marker(
                frame, hist_pt, color=(0, 255, 255), size=6, thickness=1
            )

        # Center between the two wheel centers (blue solid circle)
        if both_wheels_detected and wheel_center_avg is not None:
            draw_point_marker(
                frame, wheel_center_avg, color=(255, 0, 0),
                radius=6, thickness=-1, label=None
            )

        # Car-side center (cyan solid circle)
        draw_point_marker(
            frame, car_side_center, color=(255, 255, 0),
            radius=6, thickness=-1, label=None
        )

        # Adaptive fused center (red cross) – only drawn if it exists
        draw_cross_marker(
            frame, ema_adaptive_fused, color=(0, 0, 255), size=10, thickness=2
        )

        # Green line between the two wheel centers if both wheels detected
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
        total_frames_current = len(frames)
        status_text = (
            f"Frame: {frame_index}/{total_frames_current} | Dets: {len(detections)} | "
            f"Wheels: {num_wheels} | Pause: {paused} | Save: {save_video}"
        )
        cv2.putText(
            frame, status_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
        )

        # Wheel line angle overlay
        if wheel_line_angle_deg is not None:
            angle_text = f"Wheel line angle: {wheel_line_angle_deg:.1f} deg"
            cv2.putText(
                frame, angle_text, (10, height - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

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
            adaptive_fused[0] if adaptive_fused else None,
            adaptive_fused[1] if adaptive_fused else None,
            ema_adaptive_fused[0] if ema_adaptive_fused else None,
            ema_adaptive_fused[1] if ema_adaptive_fused else None,
            last_complete_fused[0] if last_complete_fused else None,
            last_complete_fused[1] if last_complete_fused else None,
            1 if last_complete_has_both else 0,
            wheel_line_angle_deg if wheel_line_angle_deg is not None else None
        ])

        # Show frame
        cv2.imshow("BRU Center Estimator (Minimal + Wheel Line Angle)", frame)

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
                # Remove last frame from frames list to stay consistent
                if len(frames) > 0:
                    frames.pop()
                # Also remove last history entry to stay consistent
                if len(adaptive_fused_history) > 0:
                    adaptive_fused_history.pop()
        elif key == ord('d') and paused:
            # Step forward (just let loop continue)
            pass

    # Cleanup
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
