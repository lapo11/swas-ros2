#!/usr/bin/env python3
"""
Batch YOLOv8 ONNX video processing with output video saved
Process entire video and save annotated result to file.
"""

from ultralytics import YOLO
import cv2
import numpy as np
from datetime import datetime


# === CONFIGURATION ===
model_path = '/home/lapoveca/swas-ros2/models/first_yolov8_24092026.onnx'
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/white_car_vid.mp4'

class_names = ['car_side', 'wheel']
conf_thresh = 0.25
output_dir = '/home/lapoveca/swas-ros2/scripts/vision/output'
# =====================


def main():
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

    # Setup output video writer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = f"output_video_{timestamp}.mp4"
    output_path = output_dir + output_filename
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    print(f"💾 Output will be saved to: {output_path}")
    print(f"\n🚀 Processing video (conf={conf_thresh})...")

    frame_count = 0
    total_detections = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1

        # Progress indicator every 10%
        if frame_count % max(1, total_frames // 10) == 0:
            progress = (frame_count / total_frames) * 100
            print(f"   Progress: {progress:.0f}% ({frame_count}/{total_frames} frames)")

        # Run inference
        results = model.predict(frame, conf=conf_thresh, verbose=False)
        detections = results[0].boxes
        total_detections += len(detections)

        # Draw detections on frame
        for box in detections:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            label = f"{class_names[cls]} {conf:.2f}"

            color = (0, 255, 0) if cls == 0 else (0, 0, 255)  # green=car, red=wheel

            # Draw bounding box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

            # Draw label background
            (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, 
                         (int(x1), int(y1) - text_h - baseline - 5),
                         (int(x1) + text_w, int(y1)),
                         color, -1)

            # Draw label text
            cv2.putText(frame, label, (int(x1), int(y1) - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Add info overlay
        info_text = f"Frame: {frame_count}/{total_frames} | Detections: {len(detections)}"
        cv2.putText(frame, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Write frame to output
        out.write(frame)

    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"\n✅ Processing complete!")
    print(f"   Processed: {frame_count} frames")
    print(f"   Total detections: {total_detections}")
    print(f"   Average detections/frame: {total_detections / max(1, frame_count):.2f}")
    print(f"   Output saved to: {output_path}")


if __name__ == '__main__':
    main()