#!/usr/bin/env python3
"""
Test YOLOv8 ONNX model locally using Ultralytics (matches Colab behavior exactly)
Run inference on a test image and visualize detections at multiple confidence thresholds.
"""

from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
import numpy as np


# === CONFIGURATION ===
model_path = '/home/lapoveca/swas-ros2/models/first_yolov8_24092026.onnx'

# test_img = '/home/lapoveca/swas-ros2/datasets/car_side_wheels_merged/val/images/2008_000193.jpg'
test_img = '/home/lapoveca/swas-ros2/datasets/lapo_custom/white_car3.jpeg'

class_names = ['car_side', 'wheel']
conf_thresholds = [0.1, 0.25, 0.5, 0.75]
# =====================


def main():
    print(f"🔍 Loading ONNX model: {model_path}")
    best_model = YOLO(model_path)

    print(f"📷 Loading test image: {test_img}")
    img_rgb = cv2.imread(test_img)
    if img_rgb is None:
        print(f"❌ Error: Could not load image from {test_img}")
        return
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)

    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes = axes.flatten()

    for idx, conf_thresh in enumerate(conf_thresholds):
        results = best_model.predict(test_img, conf=conf_thresh, verbose=False)
        detections = results[0].boxes

        print(f"\nConfidence threshold {conf_thresh}: {len(detections)} detections")
        for box in detections:
            cls = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            print(f"  - {class_names[cls]}: {conf:.3f}")

        # Plot image with detections
        ax = axes[idx]
        ax.imshow(img_rgb)
        ax.set_title(f"Confidence Threshold: {conf_thresh}\n{len(detections)} detections",
                     fontsize=14, fontweight='bold')

        # Draw detections
        for box in detections:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            label = f"{class_names[cls]} {conf:.2f}"

            color = 'green' if cls == 0 else 'red'

            rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1,
                                  fill=False, color=color, linewidth=2, linestyle='-')
            ax.add_patch(rect)

            ax.text(x1, y1 - 5, label, color=color, fontsize=9,
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor=color, pad=2))

        ax.axis('off')
        ax.set_xlim(0, img_rgb.shape[1])
        ax.set_ylim(img_rgb.shape[0], 0)

    plt.tight_layout()
    plt.suptitle('YOLOv8 ONNX Detection Results (Local, via Ultralytics)', fontsize=16, y=1.02)
    plt.savefig('/home/lapoveca/swas-ros2/scripts/vision/output/output_detections.png', dpi=150, bbox_inches='tight')
    print(f"\n✅ Plot saved to '/home/lapoveca/swas-ros2/scripts/vision/output/'")

    plt.show()

    print("\n📊 Summary:")
    print(f"Image: {test_img}")
    print(f"Best threshold for your use case: Start with 0.25, adjust based on false positives/negatives")


if __name__ == '__main__':
    main()
