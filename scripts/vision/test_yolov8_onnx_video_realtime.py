#!/usr/bin/env python3
"""
Real-time YOLOv8 ONNX video inference
Process video frame-by-frame and display detections in real time.
Press 'q' to quit, 's' to save current frame with detections.
"""

from ultralytics import YOLO
import cv2
import numpy as np


# === CONFIGURATION ===
model_path = '/home/lapoveca/swas-ros2/models/first_yolov8_24092026.onnx'
video_path = '/home/lapoveca/swas-ros2/datasets/lapo_custom/white_car_vid.mp4'
class_names = ['car_side', 'wheel']
conf_thresh = 0.25
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
    print(f"\n🎮 Controls: 'q' = quit, 's' = save frame, 'p' = pause/resume")
    print(f"🚀 Starting inference (conf={conf_thresh})...\n")

    frame_count = 0
    paused = False

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print(f"\n✅ End of video ({frame_count} frames processed)")
                break

            frame_count += 1

            # Run inference
            results = model.predict(frame, conf=conf_thresh, verbose=False)
            detections = results[0].boxes

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
            info_text = f"Frame: {frame_count}/{total_frames} | Detections: {len(detections)} | FPS: {fps:.1f}"
            cv2.putText(frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Display frame
        display_title = f"YOLOv8 ONNX - {video_path.split('/')[-1]} (Press 'q' to quit)"
        cv2.imshow(display_title, frame)

        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print(f"\n⏹️  Stopped by user at frame {frame_count}")
            break
        elif key == ord('s'):
            save_path = f'/home/lapoveca/swas-ros2/frame_{frame_count:05d}.jpg'
            cv2.imwrite(save_path, frame)
            print(f"💾 Saved frame {frame_count} to {save_path}")
        elif key == ord('p'):
            paused = not paused
            if paused:
                print(f"⏸️  Paused at frame {frame_count}")
            else:
                print(f"▶️  Resumed")

    cap.release()
    cv2.destroyAllWindows()
    print("\n✅ Video processing complete!")


if __name__ == '__main__':
    main()