# Configuration Reference

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

All active node parameters are stored in [`config/`](../config).

| File | Nodes configured |
|---|---|
| [`bxu_motion.yaml`](../config/bxu_motion.yaml) | `bxu_motion_node` |
| [`bxu_front_vision.yaml`](../config/bxu_front_vision.yaml) | `bxu_front_vision_node` |
| [`bxu_front_approach_controller.yaml`](../config/bxu_front_approach_controller.yaml) | `bxu_front_approach_controller_node` |
| [`bxu_docking_vision.yaml`](../config/bxu_docking_vision.yaml) | `bxu_docking_vision_node` |
| [`bxu_docking_controller.yaml`](../config/bxu_docking_controller.yaml) | `bxu_docking_controller_node` |
| [`bxu_front_yolo_vision.yaml`](../config/bxu_front_yolo_vision.yaml) | `bxu_front_yolo_vision_node` |
| [`bxu_front_yolo_approach_controller.yaml`](../config/bxu_front_yolo_approach_controller.yaml) | `bxu_front_yolo_approach_controller_node` |

## Parameter groups

### Motion

- Serial port and baud rate.
- Velocity limits.
- Direction inversion.
- Heartbeat behavior.
- Command formatting.

See [Motion control](motion.md).

### Front vision

- Camera device or ROS image topics.
- Calibration file.
- Capture resolution and frame rate.
- ArUco dictionary and marker geometry.
- Detection persistence and pose smoothing.
- Distance and visibility thresholds.

See [Marker-based front approach](front_marker_approach.md).

### Marker front controller

- Search rotation speed.
- Coarse PD gains and command limits.
- Fine pulse speeds and durations.
- Alignment and approach thresholds.
- Blind-forward speed profile and timeout.

See [Marker-based front approach](front_marker_approach.md).

### Docking vision

- Camera devices or ROS image topics.
- Calibration and alignment-zero files.
- Marker size, IDs, dictionary, and camera geometry.
- ArUco detector tuning.
- CLAHE settings.
- Publication rate and MJPEG stream settings.

See [Docking vision and control](docking.md).

### Docking controller

- Alignment topic.
- PD gains.
- Velocity and yaw-rate limits.
- Derivative filtering.
- Convergence thresholds.
- Lost-marker tolerance.

See [Docking vision and control](docking.md).

### YOLO vision

- Input source and camera configuration.
- ONNX model path.
- Inference size, confidence, IoU, and maximum detections.
- Image, diagnostics, overlay, and observation topics.
- Frame-age and stream-timeout limits.
- Detection deduplication, tracking, smoothing, and memory settings.

See [YOLO-based front approach](front_yolo_approach.md).

### YOLO approach controller

- Observation and docking topics.
- Search acquisition settings.
- PD gains and command limits.
- Camera-to-BXU direction signs.
- Alignment tolerances.
- Blind-entry and blind-forward behavior.
- Fault and timeout settings.

See [YOLO-based front approach](front_yolo_approach.md).

## Supervisor and battery lift

The supervisor and battery-lift node are configured directly in launch files rather than through YAML files.

See:

- [Supervisor](supervisor.md)
- [Battery lift](battery_lift.md)
- [Launch files](launch_files.md)

## Simulation overrides

Simulation launch files override hardware-oriented parameters where required:

| Parameter group | Typical simulation override |
|---|---|
| `input_source` | `ros` |
| Camera topics | Gazebo-published image and camera-info topics |
| `use_sim_time` | `true` |
| Docking alignment-zero file | Often disabled with an empty `zero_file` |
| MJPEG stream | Usually disabled |
| Overlay images | Usually enabled for debugging |

See [`swas_simulation`](../../swas_simulation/README.md) for the simulation launch configurations.