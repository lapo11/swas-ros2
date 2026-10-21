# System Architecture

[← Repository README](../README.md)

This document describes the SWAS runtime architecture: physical components, ROS 2 software responsibilities, operating workflows, interfaces, simulation boundaries, and failure behavior.

## System purpose

SWAS demonstrates an automated EV battery exchange between a stationary Battery Receiving Unit (BRU) and an autonomous Battery Exchange Unit (BXU).

The BXU carries a Battery Unit (BU), aligns itself beneath the BRU, and transfers the BU using its lift plate and the BRU locking mechanism. The same mechanism supports installing a BU into the BRU and removing one from it.

SWAS is a prototype intended for practical demonstrations. It combines real hardware operation with a Gazebo simulation environment.

## Physical components

| Component | Role |
|---|---|
| **BU** | Removable EV battery pack transferred between BXU and BRU |
| **BRU** | Stationary receiving unit, retrofitted from a Tesla battery pack; locks and releases a BU |
| **BXU** | Autonomous mecanum-drive robot that carries and transfers the BU |
| **Operator laptop** | Runs the HMI and provides operator command and monitoring access |
| **BXU Raspberry Pi 5** | Main BXU compute platform for ROS 2, perception, control, and hardware interfaces |
| **BRU Raspberry Pi 5** | Main BRU compute platform for BRU-side control and locking |
| **Cameras** | Provide input for front approach, marker alignment, and docking |
| **Simulation environment** | Gazebo representation of the station, BXU, vehicle, cameras, and markers |

## High-level architecture

```text
+----------------------+
|   Operator laptop    |
|      swas_hmi        |
+----------+-----------+
           |
           | rosbridge / web video server
           v
+----------------------+
|    BXU ROS 2 system  |
|      swas_system     |
+----------+-----------+
           |
           +-----------------------------+
           |                             |
           v                             v
+---------------------+       +----------------------+
| Perception          |       | Motion and actuation |
| - Camera input      |       | - Mecanum drive      |
| - Marker detection  |       | - Lift plate         |
| - YOLO detection    |       | - Relays / tools     |
| - Pose estimation   |       +----------+-----------+
+----------+----------+                  |
           |                             v
           v                    +----------------------+
+---------------------+        | Embedded controllers |
| Alignment control   |        | - Motion Pico        |
| - Front approach    |        | - Relay Pico         |
| - Docking alignment |        +----------+-----------+
+----------+----------+                   |
           |                               v
           +------------------> BXU hardware or Gazebo
```

The BRU-side controller is responsible for locking and releasing the BU. The BXU-side system is responsible for approach, alignment, motion, lift-plate movement, and battery-transfer sequencing.

## ROS 2 packages

| Package | Responsibility |
|---|---|
| [`swas_system`](../ros2_ws/src/swas_system/README.md) | Active BXU runtime nodes, configuration, launch files, calibration data, models, and tests |
| [`swas_description`](../ros2_ws/src/swas_description/README.md) | BXU and BRU descriptions, meshes, camera frames, RViz configuration, and display launches |
| [`swas_msgs`](../ros2_ws/src/swas_msgs/README.md) | Custom messages for marker detections, alignment estimates, and YOLO observations |
| [`swas_simulation`](../ros2_ws/src/swas_simulation/README.md) | Gazebo worlds, simulated cameras and markers, bridge configuration, and simulation launches |
| [`swas_hmi`](../ros2_ws/src/swas_hmi/README.md) | Browser-based operator interface and rosbridge/video-server integration |

## Runtime responsibilities

### Supervisor

`bxu_supervisor_node` coordinates the complete battery-transfer mission.

Its state machine is:

```text
WAITING
  → FRONT_APPROACH
  → DOCKING_ALIGN
  → LIFTING
  → WAIT_USER_AFTER_LIFT
  → LOWERING
  → WAIT_USER_AFTER_LOWER
  → DRIVE_AWAY
  → DONE
```

A front-approach `FAULT` or docking `LOST` status transitions the mission to `FAULT`.

The supervisor enables only one controller stage at a time, publishes zero velocity between stages, commands the lift, and drives the BXU backward after the transfer.

In simulation, it exposes `/bxu/supervisor/start`, `/bxu/supervisor/confirm`, and `/bxu/supervisor/stop`. These services are created only when `use_sim_time` is true. On hardware, the supervisor currently waits for terminal confirmation after lift and lower operations.

### Perception

Perception converts camera images into alignment information for the controllers.

| Pipeline | Vision node | Estimation | Controller | Status |
|---|---|---|---|---|
| Marker-based front approach | `bxu_front_vision_node` | Marker geometry and PnP pose | `bxu_front_approach_controller_node` | Hardware and simulation |
| YOLO-based front approach | `bxu_front_yolo_vision_node` | `front_yolo_estimator.py` | `bxu_front_yolo_approach_controller_node` | Simulation only |
| Docking | `bxu_docking_vision_node` | Two-camera ArUco fusion | `bxu_docking_controller_node` | Hardware and simulation |

The marker-based front pipeline estimates metric distance, lateral error, and yaw from ArUco markers. The YOLO pipeline detects a vehicle side and wheels, then regulates normalized image-centre error and wheel-line image angle. It does not estimate metric pose.

The docking pipeline uses two upward-facing cameras to detect ArUco markers beneath the BRU. It fuses their observations into forward, lateral, and yaw errors in the BXU frame.

### Front approach and docking

**Front approach** brings the BXU close to the BRU and roughly centers it beneath it. It is not intended to achieve the precision required for battery transfer.

**Docking** performs the final precise alignment. It corrects the remaining forward, lateral, and yaw error so that the lift plate can engage the BU correctly.

The marker-based front controller uses search, coarse PD control, fine pulse-based alignment, forward approach, and a final blind-forward phase. The YOLO controller uses search, PD control on image-space features, and a final blind-forward phase.

Both controllers complete front approach when the docking vision reports valid docking markers.

### Motion and battery lift

`bxu_motion_node` converts `/bxu/cmd_vel` into `V vx vy omega` serial commands for the motion Pico. The Pico performs mecanum kinematics, stepper-pulse generation, driver timing, and alarm monitoring.

`bxu_battery_lift_node` controls the battery lift plate through the relay Pico. It accepts `LIFT` and `LOWER` commands and reports `IDLE`, `LIFTING`, `LIFTED`, `LOWERING`, or `LOWERED`.

The lift motion is time-based, not position-based.

## Operating workflows

### Marker-based workflow

```text
Start
  |
  v
Front-camera ArUco detection
  |
  v
Marker-based front approach
  |
  v
Rough alignment beneath BRU
  |
  v
Docking-marker detection
  |
  v
Two-camera docking alignment
  |
  v
Lift plate raises BU into BRU
  |
  v
BRU locks BU
  |
  v
Lift plate lowers
  |
  v
BXU drives away
```

### YOLO-based workflow

```text
Start
  |
  v
Front-camera image acquisition
  |
  v
YOLO vehicle/wheel detection
  |
  v
Image-space target and wheel-line estimation
  |
  v
YOLO front-approach control
  |
  v
Rough alignment beneath BRU
  |
  v
Docking-marker detection
  |
  v
Two-camera docking alignment
  |
  v
Battery-transfer sequence
```

The YOLO workflow has been tested in simulation only.

## Interfaces

### Main topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/bxu/cmd_vel` | `geometry_msgs/Twist` | Controllers → motion node | BXU velocity command |
| `/bxu/front_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Front vision → marker controller | Marker-based alignment |
| `/bxu/front_vision/raw_markers` | `swas_msgs/MarkerDetectionArray` | Front vision → monitoring | Raw front-marker detections |
| `/bxu/docking_vision/alignment` | `swas_msgs/BxuAlignmentOutput` | Docking vision → docking controller | Docking alignment |
| `/bxu/front_yolo_vision/observation` | `swas_msgs/BxuFrontYoloObservation` | YOLO vision → YOLO controller | Image-space YOLO observation |
| `/bxu/front_approach/controller_status` | `std_msgs/String` | Front controller → supervisor | Front-approach state |
| `/bxu/docking/controller_status` | `std_msgs/String` | Docking controller → supervisor | Docking state |
| `/bxu/supervisor/state` | `std_msgs/String` | Supervisor → HMI/monitoring | Mission state |
| `/bxu/motion/status` | `std_msgs/String` | Motion node → monitoring | Serial and command status |

### Custom messages

| Message | Purpose |
|---|---|
| `MarkerDetection` | One detected ArUco marker, its corners, and estimated pose |
| `MarkerDetectionArray` | Timestamped set of marker detections |
| `BxuAlignmentOutput` | Alignment error, visibility, confidence, and docking forward error |
| `BxuFrontYoloObservation` | Image-space YOLO observation; contains no metric pose |

See [`swas_msgs`](../ros2_ws/src/swas_msgs/README.md) for field definitions and conventions.

### Hardware boundary

ROS 2 nodes do not generate stepper pulses or relay signals directly. They communicate with Pico controllers over serial:

- The motion Pico generates step, direction, and enable signals for the HB860H drivers.
- The relay Pico controls lift and auxiliary relays.
- The HB860H drivers provide closed-loop position-following and alarm feedback.

See [Hardware](HARDWARE.md).

## HMI integration

The HMI runs on the operator laptop and connects through:

- `rosbridge_server` on port 9090 for ROS discovery, subscriptions, and service calls.
- `web_video_server` on port 8080 for JPEG camera snapshots.

The HMI can start, confirm, and stop the supervisor workflow in simulation. Hardware profiles are currently monitor-only because the supervisor services are created only when `use_sim_time` is true.

The optional simulation command gate routes autonomous velocity commands through `/bxu/auto/cmd_vel`, accepts HMI teleoperation on `/bxu/teleop/cmd_vel`, and publishes the final `/bxu/cmd_vel` command. It is not a safety controller and is not validated on hardware.

## Simulation and hardware

| Aspect | Simulation | Hardware |
|---|---|---|
| Camera input | Gazebo image and CameraInfo topics | USB / GStreamer cameras |
| Camera model | Ideal, zero-distortion simulated cameras | Calibrated physical cameras |
| BXU motion | Gazebo velocity-control plugin | Motion Pico and stepper drivers |
| Battery lift | Simulated acknowledgement node | Relay Pico and lift hardware |
| Supervisor services | Available | Not currently exposed |
| Marker-based workflow | Supported | Supported |
| YOLO workflow | Supported and tested | Not yet validated |

The simulation uses `use_sim_time: true`, ROS image inputs, and the Gazebo–ROS bridge. Hardware nodes use direct camera devices and serial-connected Pico controllers.

## Failure handling

Implemented failure behavior includes:

- Marker-based front approach reports `FAULT` after a blind-forward timeout on hardware.
- YOLO front approach reports faults for lost observations, invalid alignment features, blind-forward timeout, or no usable target.
- Docking control reports `LOST` after repeated missing-marker observations and publishes zero velocity.
- The supervisor enters `FAULT` on front-approach fault or docking loss, disables both controllers, and publishes zero velocity.
- The motion node reports serial-open and write failures on `/bxu/motion/status`.
- The simulation command gate stops on a stale clock, lost supervisor state, lost manual/auto authority interlocks, or detected conflicting final-velocity publishers.

The HMI distinguishes WebSocket connection, ROS discovery, subscription requests, bridge errors, and actual received-message counts. It does not treat a successful service call as proof of ongoing physical health.

## Design decisions

- Mission coordination is separated from real-time motion and relay control.
- Front approach and docking are separate stages: front approach provides rough positioning; docking provides final precise alignment.
- Marker-based docking provides a precise geometric reference for the final alignment.
- The YOLO pipeline reduces dependence on front markers during approach, but currently remains image-space only and simulation-tested.
- The same principal ROS topics and node responsibilities are used in simulation and hardware.
- The HMI is an operator interface, not a safety controller; robot-local watchdogs and physical stopping remain independent.

## Known integration issue

The supervisor publishes lift commands and subscribes to lift state on:

```text
/bxu_battery_lift/command
/bxu_battery_lift/state
```

The hardware battery-lift node uses:

```text
/bxubatteryliftcommand
/bxubatteryliftstate
```

These names must be aligned before the supervisor-driven lift sequence works on hardware. The simulation lift node already uses the supervisor’s topic names.