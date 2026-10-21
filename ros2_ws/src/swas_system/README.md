# `swas_system`

[← Repository README](../../../README.md) · [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)

`swas_system` contains the active ROS 2 software for the SWAS Battery Exchange Unit (BXU). It provides mission supervision, camera-based perception, front approach, precise docking alignment, mecanum motion control, and battery-lift control.

The package supports both physical hardware and ROS-image-based simulation inputs. The same core nodes are used in both cases; camera acquisition and some timing/failure behavior differ depending on the input source.

The marker-based front-approach, docking, motion, and supervisor nodes are used in both hardware and simulation. The YOLO-based front-approach pipeline has currently been tested in simulation only.

## Purpose

The BXU aligns itself beneath the Battery Receiving Unit (BRU), transfers a Battery Unit (BU) using its lift plate, and drives away after the transfer. `swas_system` implements the software needed to perform this sequence autonomously.

The package contains:

- Mission supervision and controller coordination.
- Marker-based front approach.
- YOLO-based front approach.
- Docking vision and precise docking control.
- Mecanum-drive motion control.
- Battery-lift control through a relay Pico.

## Package layout

```text
swas_system/
├── calib/       # Camera intrinsics and docking alignment-zero calibration
├── config/      # Node parameters
├── launch/      # Hardware launch files
├── models/      # Active YOLO ONNX model
├── resource/    # ROS 2 package resource marker
├── swas_system/ # Python source code
│   ├── bxu/     # Active BXU nodes
│   ├── bru/     # Empty package namespace
│   └── common/  # Empty package namespace
└── test/        # Test directory
```

The active source code is in [`swas_system/bxu/`](swas_system/bxu). The `bru` and `common` namespaces are currently empty.

## Runtime architecture

```text
                 +---------------------+
                 |  bxu_supervisor_node |
                 +----------+----------+
                            |
        enables / disables  |  lift commands / zero velocity
           +----------------+----------------+
           |                                 |
           v                                 v
+---------------------+             +----------------------+
| Front approach      |             | bxu_battery_lift_node |
| controller          |             +----------+-----------+
| - marker-based, or  |                        |
| - YOLO-based        |                        v
+----------+----------+                 Relay Pico
           |                                 |
           v                                 v
      /bxu/cmd_vel ----------------> bxu_motion_node
                                             |
                                             v
                                         Motion Pico
```

Perception nodes do not command motion directly. They publish alignment estimates or observations, and the active approach or docking controller converts them into `geometry_msgs/Twist` commands on `/bxu/cmd_vel`.

## Nodes

| Executable | Source | Responsibility |
|---|---|---|
| `bxu_supervisor_node` | [`supervisor_node.py`](swas_system/bxu/supervisor_node.py) | Coordinates the complete battery-transfer mission |
| `bxu_motion_node` | [`motion_node.py`](swas_system/bxu/motion_node.py) | Converts velocity commands into motion-Pico serial commands |
| `bxu_battery_lift_node` | [`battery_lift_node.py`](swas_system/bxu/battery_lift_node.py) | Controls the battery lift plate through the relay Pico |
| `bxu_front_vision_node` | [`front_vision_node.py`](swas_system/bxu/front_vision_node.py) | Marker-based front-camera perception and alignment estimate |
| `bxu_front_approach_controller_node` | [`front_approach_controller_node.py`](swas_system/bxu/front_approach_controller_node.py) | Marker-based front approach control |
| `bxu_docking_vision_node` | [`docking_vision_node.py`](swas_system/bxu/docking_vision_node.py) | Upward-camera ArUco perception and docking alignment estimate |
| `bxu_docking_controller_node` | [`docking_controller_node.py`](swas_system/bxu/docking_controller_node.py) | Precise PD docking alignment control |
| `bxu_front_yolo_vision_node` | [`front_yolo_vision_node.py`](swas_system/bxu/front_yolo_vision_node.py) | YOLO-based vehicle/wheel perception and image-space observation |
| `bxu_front_yolo_approach_controller_node` | [`front_yolo_approach_controller_node.py`](swas_system/bxu/front_yolo_approach_controller_node.py) | YOLO-based front approach control |

Detailed node documentation:

- [Supervisor](docs/supervisor.md)
- [Motion control](docs/motion.md)
- [Battery lift](docs/battery_lift.md)
- [Docking vision and control](docs/docking.md)
- [Marker-based front approach](docs/front_marker_approach.md)
- [YOLO-based front approach](docs/front_yolo_approach.md)
- [Configuration reference](docs/configuration.md)
- [Launch files](docs/launch_files.md)

## Main topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/bxu/cmd_vel` | `geometry_msgs/Twist` | Controllers → motion node | BXU velocity command |
| `/bxu/front_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Front vision → marker controller | Marker-based front alignment |
| `/bxu/front_vision/raw_markers` | `swas_msgs/MarkerDetectionArray` | Front vision → external consumers | Raw detected front markers |
| `/bxu/front_vision/image_raw` | `sensor_msgs/Image` | Front vision → visualization | Rectified front-camera image or overlay |
| `/bxu/docking_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Docking vision → docking controller | Precise docking alignment |
| `/bxu/front_yolo_vision/observation` | `swas_msgs/BxuFrontYoloObservation` | YOLO vision → YOLO controller | Image-space YOLO observation |
| `/bxu/front_yolo_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | YOLO vision → diagnostics | Placeholder alignment output; not for metric control |
| `/bxu/front_yolo_vision/image_raw` | `sensor_msgs/Image` | YOLO vision → visualization | Rectified YOLO overlay image |
| `/bxu/front_approach/controller_status` | `std_msgs/String` | Front controller → supervisor | Front-approach state |
| `/bxu/docking/controller_status` | `std_msgs/String` | Docking controller → supervisor | Docking state |
| `/bxu/supervisor/state` | `std_msgs/String` | Supervisor → HMI / monitoring | Supervisor mission state |
| `/bxu/motion/status` | `std_msgs/String` | Motion node → monitoring | Serial command and connection status |

The battery-lift topic names are listed in [Battery lift](docs/battery_lift.md). They currently contain an interface mismatch that must be resolved before using the supervisor-driven lift sequence.

## Front-approach implementations

Two alternative front-approach pipelines are implemented:

| Pipeline | Vision | Controller | Primary measurement |
|---|---|---|---|
| Marker-based | `bxu_front_vision_node` | `bxu_front_approach_controller_node` | Metric distance, lateral error, and yaw from ArUco markers |
| YOLO-based | `bxu_front_yolo_vision_node` | `bxu_front_yolo_approach_controller_node` | Normalized image-centre error and wheel-line image angle; simulation-tested |

Only one front-approach controller should be enabled at a time. Both controllers publish to `/bxu/cmd_vel` and use the same supervisor enable topic, `/bxu/front_controller/enabled`.

## Calibration and model assets

| Asset | Used by | Purpose |
|---|---|---|
| `calib/front_cam_1280x720_calib.npz` | Front vision and YOLO vision | Front-camera intrinsics and distortion correction |
| `calib/camera_0_640x480_calibration.npz` | Docking vision | Intrinsics and distortion for upward camera 0 |
| `calib/camera_1_640x480_calibration.npz` | Docking vision | Intrinsics and distortion for upward camera 1 |
| `calib/alignment_zero.npz` | Docking vision | Pixel offsets defining the aligned BXU/BRU docking reference |
| `models/first_yolov8_24092026.onnx` | YOLO vision | ONNX YOLOv8 detector for `car_side` and `wheel` classes |

The docking alignment-zero calibration defines where the BRU docking markers appear in the BXU upward-camera image frames when the BXU and BRU are aligned. It is required for the docking phase.

## Dependencies

ROS 2 dependencies:

- `rclpy`
- `geometry_msgs`
- `std_msgs`
- `std_srvs`
- `sensor_msgs`
- `cv_bridge`
- `swas_msgs`
- `ament_index_python`

System/Python dependencies:

- `pyserial`
- `numpy`
- OpenCV
- ONNX Runtime
- Ultralytics YOLO

The YOLO node requires ONNX Runtime and Ultralytics at runtime. It runs inference on CPU with a fixed 640 × 640 input size.

## Build

```bash
cd ros2_ws
colcon build --packages-select swas_system
source install/setup.bash
```

## Related documentation

- [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)
- [Hardware](../../../docs/HARDWARE.md)
- [Operation](../../../docs/OPERATION.md)
- [`swas_msgs`](../swas_msgs/README.md)
- [`swas_simulation`](../swas_simulation/README.md)
- [`swas_hmi`](../swas_hmi/README.md)