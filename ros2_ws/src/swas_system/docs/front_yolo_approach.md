# YOLO-Based Front Approach

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

## Purpose

The YOLO-based front-approach subsystem detects a vehicle side and its wheels using a front camera, then centers and aligns the BXU beneath the BRU without requiring front ArUco markers.

It is an alternative to the marker-based front-approach pipeline. When active, `bxu_front_vision_node` and `bxu_front_approach_controller_node` are not used.

## Current status

The YOLO-based front-approach pipeline has been tested in simulation only. It is not yet part of a hardware launch configuration.

The pipeline uses the same supervisor, motion, and docking interfaces as the marker-based approach. A dedicated simulation launch file and, later, a hardware launch file should explicitly start the YOLO vision and YOLO approach-controller nodes instead of the marker-based front-vision and front-approach nodes.

TODO: Add a dedicated simulation launch file for the YOLO-based front-approach pipeline. A hardware launch file should be added only after hardware validation.

## Summary

- `bxu_front_yolo_vision_node` runs YOLO inference, rectifies the camera image, tracks vehicle and wheel detections, and publishes an image-space observation.
- `front_yolo_estimator.py` performs detection association, target selection, wheel tracking, centre fusion, smoothing, and short-term centre memory.
- `bxu_front_yolo_approach_controller_node` regulates horizontal image-centre error and wheel-line image angle, then enters blind forward when the wheel pair approaches the image boundary.
- The controller completes when the docking vision reports valid docking markers.

The YOLO pipeline is image-space only. It does not estimate metric distance, lateral offset, or vehicle yaw.

## YOLO vision node

Source: [`swas_system/bxu/front_yolo_vision_node.py`](../swas_system/bxu/front_yolo_vision_node.py)

### Input modes

| `input_source` | Image source | Calibration |
|---|---|---|
| `device` | USB camera opened directly | `front_cam_1280x720_calib.npz` |
| `ros` | `/bxu/front_camera/image_raw` and `/bxu/front_camera/camera_info` | `CameraInfo`; expects zero distortion |

### Model requirements

The node loads:

```text
models/first_yolov8_24092026.onnx
```

It requires:

- ONNX input shape `[1, 3, 640, 640]`.
- Float32 input.
- Class metadata `{0: 'car_side', 1: 'wheel'}`.
- CPU inference.
- Fixed inference size of 640 × 640.
- `rect=False`.

The active model detects two classes:

| Class ID | Class |
|---:|---|
| 0 | `car_side` |
| 1 | `wheel` |

### Image processing

1. Acquire a USB frame or receive a ROS image.
2. Rectify hardware images using the front-camera calibration.
3. Run YOLO inference.
4. Keep only `car_side` and `wheel` detections.
5. Pass detections to `FrontYoloEstimator`.
6. Publish an observation, diagnostics JSON, placeholder alignment message, and annotated image.

Frames older than `max_frame_age_s` are rejected. If no frame arrives within `stream_timeout_s`, the node marks the stream stale and resets the estimator.

### Published topics

| Topic | Type | Purpose |
|---|---|---|
| `/bxu/front_yolo_vision/observation` | `swas_msgs/BxuFrontYoloObservation` | Image-space observation for the controller |
| `/bxu/front_yolo_vision/image_raw` | `sensor_msgs/Image` | Annotated overlay image |
| `/bxu/front_yolo_vision/diagnostics` | `std_msgs/String` | JSON diagnostics |
| `/bxu/front_yolo_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Placeholder output, not valid for metric control |

The alignment message intentionally sets `markers_detected=false`, zero metric errors, `distance_m=-1`, and `visibility_class='ML_IMAGE_ONLY'`. It must not be connected to a metric alignment controller.

## Estimator

Source: [`swas_system/bxu/front_yolo_estimator.py`](../swas_system/bxu/front_yolo_estimator.py)

The estimator is independent of ROS, camera hardware, and inference. It receives detections and a monotonic timestamp, then returns a filtered image-space result.

### Main responsibilities

- Remove duplicate detections using IoU.
- Associate wheels with vehicle-side detections.
- Select and track a primary vehicle target.
- Hold wheel observations through short detection gaps.
- Confirm a wheel pair after repeated fresh observations.
- Fuse vehicle-centre and wheel-centre information.
- Apply exponential smoothing to the target centre.
- Retain the last valid centre for a short period when detections are temporarily lost.

### Target selection

The estimator initially selects the largest vehicle-side detection. It then tracks the candidate closest to the previous primary target.

A challenger can replace the primary target only if it is substantially larger and remains present for `primary_switch_frames` observations. This reduces target switching when another vehicle is partially visible.

### Centre estimation

When both wheels are available, the estimator uses the midpoint between the two primary wheel centres. The vehicle-centre detection is given a small weight through `car_weight`.

When only the vehicle side is visible, the estimator uses the vehicle-centre position. When a previously complete observation is recent, it can use history-assisted fusion.

The final centre is smoothed with an exponential moving average using `ema_alpha`.

### Wheel-line angle

When two wheel centres are available, the estimator computes the angle of the line between them:

$\theta_{\text{image}} = \operatorname{atan2}\left(-(y_B-y_A), x_B-x_A\right)$

This is an image feature. It is not a metric vehicle yaw estimate.

### Observation states

| State | Meaning |
|---|---|
| `NONE` | No usable estimate |
| `CURRENT_FULL` | Fresh vehicle and wheel observations |
| `HELD_WHEELS` | One or more wheel observations are held from previous frames |
| `HISTORY_ASSISTED` | Current vehicle observation combined with recent complete evidence |
| `CAR_ONLY` | Only a vehicle-side observation is available |
| `CONFIRMED_WHEELS` | Confirmed wheel pair with fresh observations |
| `CONFIRMED_WHEELS_HELD` | Confirmed wheel pair with one or more held wheels |
| `CONFIRMED_WHEELS_INCOMPLETE` | Confirmed pair tracking is active but incomplete |
| `WHEEL_ONLY` | Test-only mode using exactly two wheel detections |
| `CENTER_MEMORY` | Last valid centre retained through a short detection gap |

## YOLO approach controller

Source: [`swas_system/bxu/front_yolo_approach_controller_node.py`](../swas_system/bxu/front_yolo_approach_controller_node.py)

### State machine

```text
SEARCH
  → PD
  → BLIND_FORWARD
  → DONE
```

The controller can also enter `FAULT`.

| State | Behavior |
|---|---|
| `SEARCH` | Rotates until a fresh wheel pair and usable alignment features are observed for `acquire_frames` within `acquire_window_s` |
| `PD` | Regulates normalized image-centre error and wheel-line angle |
| `BLIND_FORWARD` | Moves forward until docking markers are confirmed |
| `DONE` | Docking markers detected; front approach complete |
| `FAULT` | Stopped after a timeout, lost target, or invalid observation |

### PD control

With a fresh wheel pair, the controller regulates:

- `center_error_x_normalized` as lateral error.
- `wheel_line_angle_rad` as heading error.

It applies independent PD control and clamps the resulting lateral velocity and yaw rate.

Forward motion is enabled only when both lateral and yaw errors are within tolerance:

- `lateral_tolerance`: `0.03`
- `yaw_tolerance_rad`: `0.035`

With held wheel evidence, the controller corrects lateral and heading error but does not move forward. With only a vehicle-side observation, it corrects lateral error only and keeps yaw rate at zero.

### Blind entry

When the BXU is aligned and the wheel boxes approach the configured image boundary, the controller enters `BLIND_FORWARD`.

The default blind-entry criterion is:

```text
blind_edge: min_horizontal
blind_margin_ratio: 0.08
blind_entry_frames: 3
```

This means the smaller of the left and right wheel-box image margins must remain below 8% of the image dimension for three fresh observations.

### Blind forward

During blind forward, the controller moves along `blind_forward_axis` until the docking vision reports valid markers for `blind_forward_detection_confirm_frames` consecutive observations.

The forward-speed profile is:

1. `blind_forward_ramp_high_mult` times the base speed for `blind_forward_ramp_high_duration_s`.
2. Linear transition to the base speed over `blind_forward_ramp_transition_s`.
3. Base speed until docking markers are confirmed.

On hardware, blind forward faults after `blind_forward_timeout_s`. In simulation, the timeout is disabled.

### Status values

| Status | Meaning |
|---|---|
| `INACTIVE` | Controller disabled |
| `ACTIVE` | Controller enabled |
| `SEARCH` | Rotating to acquire a target |
| `SEARCH_CONFIRMING` | Usable observations are being confirmed |
| `SEARCH_LOST` | No usable observation during search |
| `PD` | PD alignment in progress |
| `PD_ALIGNING` | PD active, alignment not yet within tolerance |
| `PD_ALIGNED` | PD alignment within tolerance |
| `PD_HELD_WHEELS` | Correcting using held wheel evidence |
| `PD_CAR_ONLY` | Correcting lateral error from vehicle-side detection only |
| `PD_WAITING_FOR_TARGET` | Valid centre temporarily unavailable |
| `BLIND_FORWARD` | Final forward movement |
| `BLIND_FORWARD:<elapsed>s` | Blind forward elapsed time |
| `DONE` | Docking markers confirmed |
| `FAULT:<reason>` | Controller fault |

## Interfaces

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/bxu/front_yolo_vision/observation` | `swas_msgs/BxuFrontYoloObservation` | YOLO vision → controller | Image-space target observation |
| `/bxu/docking_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Docking vision → controller | Docking-marker confirmation |
| `/bxu/cmd_vel` | `geometry_msgs/Twist` | Controller → motion node | Approach velocity command |
| `/bxu/front_approach/controller_status` | `std_msgs/String` | Controller → supervisor | Approach status |
| `/bxu/front_controller/enabled` | `std_msgs/Bool` | Supervisor → controller | Enable/disable control |

## Configuration

Configured by:

- [`config/bxu_front_yolo_vision.yaml`](../config/bxu_front_yolo_vision.yaml)
- [`config/bxu_front_yolo_approach_controller.yaml`](../config/bxu_front_yolo_approach_controller.yaml)

Important controller parameters:

| Parameter | Default | Meaning |
|---|---:|---|
| `search_omega_mps` | `0.20` rad/s | Search rotation rate |
| `acquire_frames` | `3` | Fresh observations required to leave search |
| `acquire_window_s` | `1.5` s | Time window for acquisition |
| `kp_lateral` | `0.35` | Lateral proportional gain |
| `kd_lateral` | `0.05` | Lateral derivative gain |
| `kp_yaw` | `0.60` | Yaw proportional gain |
| `kd_yaw` | `0.08` | Yaw derivative gain |
| `max_vy_mps` | `0.08` m/s | Maximum lateral velocity |
| `max_omega_rad_s` | `0.20` rad/s | Maximum yaw rate |
| `lateral_command_sign` | `-1.0` | Maps image-right to BXU `-Y` for the current camera transform |
| `yaw_command_sign` | `1.0` | Yaw-command direction; must be validated for the deployment |
| `approach_vx_mps` | `0.05` m/s | Forward speed while aligned |
| `blind_forward_speed_mps` | `0.05` m/s | Base blind-forward speed |
| `blind_forward_timeout_s` | `25.0` s | Hardware blind-forward timeout |

## Related documentation

- [Supervisor](supervisor.md)
- [Docking vision and control](docking.md)
- [`BxuFrontYoloObservation`](../swas_msgs/README.md#bxufrontyoloobservation)