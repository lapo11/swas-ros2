# Marker-Based Front Approach

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

## Purpose

The marker-based front-approach subsystem centers and aligns the BXU beneath the BRU using ArUco markers on the BRU. It brings the BXU close enough for the docking cameras to detect the docking markers, but it does not perform the final precise docking alignment.

## Summary

- `bxu_front_vision_node` detects front ArUco markers and estimates distance, lateral error, and yaw.
- `bxu_front_approach_controller_node` uses those estimates to search, align, approach, and enter a final blind-forward phase.
- The controller completes when the docking vision reports at least one valid marker.

## Front vision

Source: [`swas_system/bxu/front_vision_node.py`](../swas_system/bxu/front_vision_node.py)

### Input modes

| `input_source` | Image source | Calibration |
|---|---|---|
| `device` | USB camera opened directly | `front_cam_1280x720_calib.npz` |
| `ros` | `/bxu/front_camera/image_raw` and `/bxu/front_camera/camera_info` | `CameraInfo`; expects zero distortion |

### Markers

The node tracks four front markers:

| Marker ID | Size | Configured centre |
|---:|---:|---|
| 2 | 64 mm | `-433 mm`, `0 mm` |
| 3 | 64 mm | `442 mm`, `0 mm` |
| 4 | 66 mm | `-105 mm`, `0 mm` |
| 5 | 64 mm | `110 mm`, `0 mm` |

Marker positions are expressed relative to the configured target centre.

### Pose estimation

The node:

1. Captures or receives a front-camera image.
2. Rectifies hardware images using the front-camera calibration.
3. Applies CLAHE.
4. Detects configured ArUco markers.
5. Persists detections for up to `detection_persist_frames`.
6. Selects an appropriate marker subset.
7. Solves PnP to estimate the target pose.
8. Smooths distance, yaw, and lateral estimates over `pose_smoothing_window` frames.

If markers 4 and 5 are both visible and the estimated distance is below `center_pair_only_below_cm`, the node uses only that central pair. This reduces sensitivity to perspective distortion at close range.

### Visibility classes

| Class | Marker condition |
|---|---|
| `FULL` | Markers 2, 3, 4, and 5 are visible |
| `CENTER_PAIR` | Markers 4 and 5 are visible |
| `OUTER_PAIR` | Markers 2 and 3 are visible |
| `LEFT_SIDE_ONLY` | Only left-side markers are visible |
| `RIGHT_SIDE_ONLY` | Only right-side markers are visible |
| `SINGLE_MARKER` | One marker is visible |
| `MIXED_PARTIAL` | Another partial combination is visible |
| `NONE` | No marker is visible |

### Published topics

| Topic | Type | Purpose |
|---|---|---|
| `/bxu/front_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Distance, lateral error, yaw error, visibility, and confidence |
| `/bxu/front_vision/raw_markers` | `swas_msgs/MarkerDetectionArray` | Raw detected markers and poses |
| `/bxu/front_vision/image_raw` | `sensor_msgs/Image` | Rectified image or annotated overlay |

## Front-approach controller

Source: [`swas_system/bxu/front_approach_controller_node.py`](../swas_system/bxu/front_approach_controller_node.py)

### State machine

```text
SEARCH
  → COARSE_PD
  → FINE_YAW
  → FINE_LATERAL
  → APPROACH_FINE
  → BLIND_FORWARD
  → DONE
```

The controller can also enter `FAULT`.

| State | Behavior |
|---|---|
| `SEARCH` | Rotates until at least two markers are detected |
| `COARSE_PD` | PD control on yaw, lateral error, and distance |
| `FINE_YAW` | Discrete rotation pulses until yaw is within tolerance |
| `FINE_LATERAL` | Discrete lateral pulses until lateral error is within tolerance |
| `APPROACH_FINE` | Discrete forward pulses while alignment remains acceptable |
| `BLIND_FORWARD` | Moves forward without front-marker feedback until docking markers are seen |
| `DONE` | Front approach complete |
| `FAULT` | Stopped after a timeout or invalid condition |

### Coarse control

The coarse phase uses filtered derivative terms and independent limits for forward velocity, lateral velocity, and yaw rate. Minimum nonzero commands prevent the BXU from stalling at low commanded speeds.

### Fine control

The fine phase uses short, discrete motion pulses rather than continuous velocity commands. This improves controllability during the final centimetre-scale alignment.

### Blind forward

When the BXU is within `blind_entry_distance_cm` and has recently been finely aligned, it enters `BLIND_FORWARD`. The forward speed follows a high-speed ramp, transitions to the configured approach speed, and continues until docking markers are detected.

The controller requires `blind_forward_detection_confirm_frames` consecutive valid docking observations before reporting `DONE`.

On hardware, blind forward times out after `blind_forward_timeout_s`. In simulation, this timeout is disabled.

### Status values

| Status | Meaning |
|---|---|
| `INACTIVE` | Controller disabled |
| `ACTIVE` | Controller enabled |
| `SEARCH` | Rotating to find markers |
| `COARSE_PD` | Coarse alignment in progress |
| `FINE_YAW` | Fine yaw alignment |
| `FINE_LATERAL` | Fine lateral alignment |
| `APPROACH_FINE` | Fine forward approach |
| `BLIND_FORWARD` | Final forward movement without front-marker feedback |
| `DONE` | Front approach complete |
| `FAULT` | Front approach failed |

## Interfaces

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/bxu/front_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Vision → controller | Front alignment estimate |
| `/bxu/docking_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Docking vision → controller | Used to detect docking markers during blind forward |
| `/bxu/cmd_vel` | `geometry_msgs/Twist` | Controller → motion node | Approach velocity command |
| `/bxu/front_approach/controller_status` | `std_msgs/String` | Controller → supervisor | Approach status |
| `/bxu/front_controller/enabled` | `std_msgs/Bool` | Supervisor → controller | Enable/disable control |

## Configuration

Configured by [`config/bxu_front_vision.yaml`](../config/bxu_front_vision.yaml) and [`config/bxu_front_approach_controller.yaml`](../config/bxu_front_approach_controller.yaml).

Important controller thresholds:

| Parameter | Default | Meaning |
|---|---:|---|
| `yaw_fine_deg` | `0.7` deg | Yaw tolerance for fine alignment |
| `lat_fine_cm` | `1.0` cm | Lateral tolerance for fine alignment |
| `yaw_approach_deg` | `3.5` deg | Yaw tolerance to enter fine alignment |
| `lat_approach_cm` | `3.0` cm | Lateral tolerance to enter fine alignment |
| `near_visible_distance_cm` | `35.0` cm | Distance considered near the BRU |
| `blind_entry_distance_cm` | `23.0` cm | Distance at which blind forward may begin |
| `blind_forward_timeout_s` | `25.0` s | Hardware blind-forward timeout |

## Related documentation

- [Supervisor](supervisor.md)
- [Docking vision and control](docking.md)
- [`BxuAlignmentOutput`](../swas_msgs/README.md#bxualignmentoutput)