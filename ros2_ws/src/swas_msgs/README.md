# `swas_msgs`

[← Repository README](../../../README.md) · [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)

`swas_msgs` defines the custom ROS 2 messages used by the SWAS BXU perception, alignment, and control nodes.

The package contains four message types:

- `MarkerDetection` — one detected fiducial marker.
- `MarkerDetectionArray` — a set of detected fiducial markers.
- `BxuAlignmentOutput` — a vision-derived alignment estimate.
- `BxuFrontYoloObservation` — image-space observations from the YOLO-based front-vision pipeline.

## Purpose

The message definitions separate perception output from control logic. Vision nodes publish detections or alignment estimates; controllers consume those messages and produce motion commands.

This package intentionally contains only message definitions. It does not contain nodes, launch files, configuration, or executable logic.

## Package contents

```text
swas_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── BxuAlignmentOutput.msg
    ├── BxuFrontYoloObservation.msg
    ├── MarkerDetection.msg
    └── MarkerDetectionArray.msg
```

## Message usage

| Message | Main producers | Main consumers | Used by |
|---|---|---|---|
| [`MarkerDetection`](#markerdetection) | Marker-based front-vision and docking-vision nodes | Marker-based front-approach and docking controllers | Original front approach, docking |
| [`MarkerDetectionArray`](#markerdetectionarray) | Marker-based front-vision and docking-vision nodes | Marker-based front-approach and docking controllers | Original front approach, docking |
| [`BxuAlignmentOutput`](#bxualignmentoutput) | Front-vision and docking-vision nodes | Front-approach and docking controllers | Original front approach, docking, YOLO front approach |
| [`BxuFrontYoloObservation`](#bxufrontyoloobservation) | `front_yolo_vision_node.py` | `front_yolo_approach_controller_node.py` | YOLO-based front approach |

The marker-based and YOLO-based front-approach paths are alternative implementations. Only one is active in a given launch configuration.

## `MarkerDetection`

**File:** [`msg/MarkerDetection.msg`](msg/MarkerDetection.msg)

`MarkerDetection` describes one detected fiducial marker. It includes the marker identifier, its four image-space corner coordinates, and an estimated pose.

### Fields

| Field | Type | Unit / frame | Description |
|---|---|---|---|
| `marker_id` | `int32` | — | Identifier of the detected marker |
| `corners_u` | `float32[4]` | pixels | Horizontal image coordinates of the four marker corners |
| `corners_v` | `float32[4]` | pixels | Vertical image coordinates of the four marker corners |
| `pose` | `geometry_msgs/Pose` | camera-related pose frame | Estimated marker pose |

TODO: Document the corner ordering convention and the exact pose frame once the vision-node implementation is reviewed.

### Usage

`MarkerDetection` is used by the marker-based front-vision and docking-vision pipelines. Controllers use the marker geometry and pose information to estimate the BXU’s alignment error relative to the BRU.

## `MarkerDetectionArray`

**File:** [`msg/MarkerDetectionArray.msg`](msg/MarkerDetectionArray.msg)

`MarkerDetectionArray` is a timestamped collection of `MarkerDetection` messages.

### Fields

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp and frame information for the detection set |
| `detections` | `MarkerDetection[]` | All markers detected in the current observation |

### Usage

This is the primary message used when more than one marker may be visible. It allows a vision node to publish a complete marker observation, while a controller can decide which markers are valid or relevant for alignment.

## `BxuAlignmentOutput`

**File:** [`msg/BxuAlignmentOutput.msg`](msg/BxuAlignmentOutput.msg)

`BxuAlignmentOutput` is the alignment estimate published by a BXU vision node. It expresses the BXU’s error relative to a target in a form suitable for a controller.

The message is used by:

- `front_vision_node.py` and `front_approach_controller_node.py`
- `docking_vision_node.py` and `docking_controller_node.py`
- `front_yolo_approach_controller_node.py`

### Fields

| Field | Type | Unit / convention | Description |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | Timestamp and frame information |
| `lateral_error_m` | `float32` | metres; positive = right | Lateral offset from the target |
| `yaw_error_rad` | `float32` | radians; positive = clockwise | Yaw misalignment relative to the target |
| `distance_m` | `float32` | metres | Estimated distance to the target; `-1.0` if unknown |
| `markers_detected` | `bool` | — | True if at least one valid marker contributed to the estimate |
| `num_markers_detected` | `int32` | — | Number of markers used in the estimate |
| `visibility_class` | `string` | — | Marker visibility category, e.g. `NONE`, `PARTIAL`, or `FULL` |
| `confidence` | `float32` | 0.0–1.0 | Quality of the current alignment estimate |
| `forward_error_m` | `float32` | metres; BXU forward axis | Signed forward-axis error, `ex_r`; populated by docking vision, `0.0` for front vision |

### Interpretation

- `lateral_error_m` and `yaw_error_rad` are the primary alignment errors used for centering and heading correction.
- `distance_m` provides an approximate range to the target when available.
- `markers_detected`, `num_markers_detected`, `visibility_class`, and `confidence` allow a controller or supervisor to distinguish a usable estimate from an unreliable one.
- `forward_error_m` is relevant during docking, where the BXU must also correct its longitudinal position beneath the BRU.

TODO: Document the exact target pose, sign convention in the robot frame, and the conditions under which each visibility class is produced.

## `BxuFrontYoloObservation`

**File:** [`msg/BxuFrontYoloObservation.msg`](msg/BxuFrontYoloObservation.msg)

`BxuFrontYoloObservation` carries image-space observations from the YOLO-based front-vision pipeline. It deliberately does not contain a metric pose estimate; metric interpretation is handled downstream by `front_yolo_estimator.py` and the YOLO front-approach controller.

### Coordinate convention

All pixel values are expressed in the rectified front-camera image frame.

| Axis | Convention |
|---|---|
| `x` | Horizontal image coordinate, increasing to the right |
| `y` | Vertical image coordinate, increasing downward |
| Origin | Upper-left corner of the rectified image |

### Fields

| Field | Type | Unit / convention | Description |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | Timestamp and frame information |
| `valid` | `bool` | — | True if the observation is usable |
| `state` | `string` | — | Pipeline state label |
| `image_width` | `uint32` | pixels | Width of the rectified image |
| `image_height` | `uint32` | pixels | Height of the rectified image |
| `principal_point_x_px` | `float32` | pixels | Horizontal principal point of the rectified camera |
| `center_x_px` | `float32` | pixels | Smoothed horizontal target centre |
| `center_y_px` | `float32` | pixels | Smoothed vertical target centre |
| `center_error_x_px` | `float32` | pixels | Horizontal error between target centre and reference |
| `center_error_x_normalized` | `float32` | normalized | Horizontal centre error normalized by image geometry |
| `wheel_line_valid` | `bool` | — | True if a valid wheel-centre line was estimated |
| `wheel_line_angle_rad` | `float32` | radians | Angle of the line between the two primary wheel centres; image feature, not metric vehicle yaw |
| `real_wheels` | `uint8` | — | Number of detected wheel candidates |
| `held_wheels` | `uint8` | — | Number of wheels currently held by the tracking/filtering logic |
| `confirmed_pair` | `bool` | — | True if a usable primary wheel pair is confirmed |
| `wheel_only_mode` | `bool` | — | True when the pipeline relies only on wheel observations |
| `wheel_box_xyxy_px` | `float32[]` | pixels | Wheel bounding boxes in `[x_min, y_min, x_max, y_max]` order |
| `wheel_held` | `bool[]` | — | Whether each corresponding wheel is currently held |
| `wheel_side` | `string[]` | — | Side label for each corresponding wheel |
| `detector_confidence` | `float32` | 0.0–1.0 | Confidence associated with the detector output |

The `wheel_box_xyxy_px`, `wheel_held`, and `wheel_side` arrays always have equal length.

TODO: Document the allowed values of `state` and `wheel_side`, and the exact rule used to select the primary wheel pair.

### Usage

`front_yolo_vision_node.py` publishes this message after detecting and filtering vehicle/wheel observations. `front_yolo_estimator.py` and `front_yolo_approach_controller_node.py` use the image-space target centre, wheel-line angle, and observation quality to estimate and control the BXU’s front approach.

The YOLO pipeline also uses `BxuAlignmentOutput` when a metric alignment estimate is required.

## Build and dependencies

The package uses `ament_cmake` and `rosidl_default_generators` to generate ROS 2 message bindings.

Dependencies:

| Dependency | Purpose |
|---|---|
| `std_msgs` | `Header` and primitive message support |
| `geometry_msgs` | `Pose` used by `MarkerDetection` |
| `rosidl_default_generators` | Message generation at build time |
| `rosidl_default_runtime` | Generated-message runtime support |

### Build

```bash
cd ros2_ws
colcon build --packages-select swas_msgs
source install/setup.bash
```

## Related documentation

- [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)
- [`swas_system`](../swas_system/README.md)
- [`swas_simulation`](../swas_simulation/README.md)
- [`swas_hmi`](../swas_hmi/README.md)