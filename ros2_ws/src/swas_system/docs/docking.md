# Docking Vision and Control

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

## Purpose

The docking subsystem performs the final precise alignment between the BXU and BRU. It uses two upward-facing cameras to detect ArUco markers beneath the BRU, estimates the remaining alignment error, and drives the BXU until the error is within tolerance.

## Summary

- `bxu_docking_vision_node` detects docking markers, fuses observations from two cameras, and publishes `BxuAlignmentOutput`.
- `bxu_docking_controller_node` uses the forward, lateral, and yaw errors to publish a `Twist` command.
- The supervisor enables the docking controller only after front approach completes.

## Docking vision

Source: [`swas_system/bxu/docking_vision_node.py`](../swas_system/bxu/docking_vision_node.py)

### Inputs

The node supports two input modes:

| `input_source` | Image source | Calibration source |
|---|---|---|
| `hardware` | Two cameras opened directly through OpenCV/GStreamer | `camera_0_640x480_calibration.npz`, `camera_1_640x480_calibration.npz` |
| `ros` | `/bxu/docking_camera_0/image_raw` and `/bxu/docking_camera_1/image_raw` | `CameraInfo` messages |

The ROS input mode expects ideal, zero-distortion images, as produced by the simulation environment.

### Marker configuration

| Parameter | Default | Meaning |
|---|---:|---|
| `marker_size_mm` | `9.0` | Physical ArUco marker side length |
| `cam0_target_id` | `0` | Marker ID tracked by camera 0 |
| `cam1_target_id` | `1` | Marker ID tracked by camera 1 |
| `aruco_dict` | `DICT_4X4_50` | ArUco dictionary |
| `cam0_pos_mm` | `[0.0, 0.0]` | Camera 0 position relative to the BXU reference |
| `cam1_pos_mm` | `[600.0, 0.0]` | Camera 1 position relative to the BXU reference |
| `cam_to_robot_r` | identity | 2 × 2 rotation from camera plane to BXU robot plane |

### Processing

For each camera:

1. Acquire or receive an image.
2. Rectify the image using its calibration, in hardware mode.
3. Apply CLAHE contrast enhancement.
4. Detect ArUco markers.
5. Select the configured target marker.
6. Estimate marker position using PnP.
7. Convert the marker-centre pixel error into a camera-plane metric error.
8. Subtract the alignment-zero pixel offset.
9. Transform the error into the BXU robot frame.

When both cameras provide valid observations, the node computes yaw error from the difference between their horizontal errors and the known camera baseline.

### Published alignment

The node publishes `BxuAlignmentOutput` at `publish_rate_hz`.

| Condition | `visibility_class` | `confidence` |
|---|---|---:|
| No valid marker | `NONE` | `0.0` |
| One valid camera observation | `PARTIAL` | `0.5` |
| Both camera observations valid | `FULL` | `1.0` |

The output fields are:

- `forward_error_m`: BXU forward-axis error.
- `lateral_error_m`: BXU lateral error.
- `yaw_error_rad`: Yaw misalignment.
- `distance_m`: Euclidean planar distance to the aligned reference.
- `markers_detected`: Whether at least one valid marker was observed.
- `num_markers_detected`: Number of contributing camera observations.

### Debug stream

When `enable_mjpeg_stream` is true, the node serves a combined MJPEG stream of both processed camera images on port `5000`.

## Docking controller

Source: [`swas_system/bxu/docking_controller_node.py`](../swas_system/bxu/docking_controller_node.py)

### Control law

The controller implements independent PD control on forward error, lateral error, and yaw error:

$v_x = -\left(k_{p,xy} e_x + k_{d,xy} \dot{e}_x\right)$

$v_y = -\left(k_{p,xy} e_y + k_{d,xy} \dot{e}_y\right)$

$\omega = -\left(k_{p,\theta} e_\theta + k_{d,\theta} \dot{e}_\theta\right)$

Derivative terms are low-pass filtered using `d_filter_alpha`. Commands are clamped to `max_vxy` and `max_omega`.

### Convergence

Docking is considered converged when:

- Planar position error is below `xy_threshold_mm`.
- Absolute yaw error is below `theta_threshold_deg`.

The controller must observe this condition for `stable_frames` consecutive alignment messages before publishing `DOCKED`.

### Status values

| Status | Meaning |
|---|---|
| `INACTIVE` | Controller disabled |
| `ACTIVE` | Controller enabled |
| `ALIGNING dist_mm=... theta_deg=... stable=...` | Alignment in progress |
| `DOCKED` | Position and yaw have remained within tolerance |
| `LOST` | No valid marker observation for `lost_tolerance_frames` |

### Failure behavior

If `markers_detected` is false for `lost_tolerance_frames` consecutive messages, the controller publishes zero velocity and reports `LOST`. The supervisor then enters `FAULT`.

## Interfaces

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/bxu/docking_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Vision → controller | Docking alignment estimate |
| `/bxu/cmd_vel` | `geometry_msgs/Twist` | Controller → motion node | Docking velocity command |
| `/bxu/docking/controller_status` | `std_msgs/String` | Controller → supervisor | Docking status |
| `/bxu/docking_controller/enabled` | `std_msgs/Bool` | Supervisor → controller | Enable/disable control |

## Configuration

The docking nodes are configured by:

- [`config/bxu_docking_vision.yaml`](../config/bxu_docking_vision.yaml)
- [`config/bxu_docking_controller.yaml`](../config/bxu_docking_controller.yaml)

Important controller parameters:

| Parameter | Default | Meaning |
|---|---:|---|
| `kp_xy` | `0.0040` | Proportional gain for planar position |
| `kd_xy` | `0.0004` | Derivative gain for planar position |
| `kp_theta` | `10.0` | Proportional gain for yaw |
| `kd_theta` | `0.25` | Derivative gain for yaw |
| `max_vxy` | `0.04` m/s | Maximum planar velocity |
| `max_omega` | `0.12` rad/s | Maximum yaw rate |
| `xy_threshold_mm` | `1.0` mm | Required planar accuracy |
| `theta_threshold_deg` | `2.0` deg | Required yaw accuracy |
| `stable_frames` | `10` | Consecutive stable observations required |
| `lost_tolerance_frames` | `3` | Missing-marker tolerance |

## Related documentation

- [Supervisor](supervisor.md)
- [`BxuAlignmentOutput`](../swas_msgs/README.md#bxualignmentoutput)
- [Hardware](../../../docs/HARDWARE.md)