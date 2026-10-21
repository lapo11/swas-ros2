# `swas_description`

[← Repository README](../../../README.md) · [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)

`swas_description` contains the robot descriptions, visual meshes, RViz configuration, and display launch files for the SWAS Battery Exchange Unit (BXU) and Battery Receiving Unit (BRU).

## Purpose

The package provides the geometric and kinematic descriptions used to visualize the BXU and BRU and to spawn them in the Gazebo simulation environment.

It does not contain runtime control logic. The models define links, fixed transforms, camera frames, visual geometry, simplified collision geometry, and placeholder dynamics.

## Package contents

```text
swas_description/
├── launch/
│   ├── bxu_display.launch.py
│   └── bru_display.launch.py
├── meshes/
│   ├── bxu/
│   │   └── bxu_visual.dae
│   └── bru/
│       └── bru_visual.stl
├── rviz/
│   └── docking_sim.rviz
└── urdf/
    ├── bxu.urdf.xacro
    └── bru.urdf.xacro
```

## Current use

The descriptions are currently used for:

- RViz visualization.
- Gazebo simulation spawning.
- Camera and TF frame definitions.
- Simulation-only odometry and transform visualization.

The BXU odometry and TF output are simulation-oriented. Hardware odometry is not currently reliable enough to be used in the same way.

## BXU description

**File:** [`urdf/bxu.urdf.xacro`](urdf/bxu.urdf.xacro)

The BXU model represents the autonomous Battery Exchange Unit.

### Coordinate convention

| Axis | Meaning |
|---|---|
| `+X` | Forward, toward the front camera and BRU |
| `+Y` | Left |
| `+Z` | Up |

`bxu_base_link` is the root frame. Its origin is defined at the midpoint of the wheel-ground-contact footprint, with `z = 0` at the floor plane.

### Links and joints

| Link / joint | Type | Description |
|---|---|---|
| `bxu_base_link` | Root link | Massless reference frame at the wheel-contact footprint centre |
| `bxu_chassis_link` | Fixed to base | Main BXU body, visual mesh, collision envelope, and placeholder inertia |
| `front_camera_link` | Fixed to base | Front camera mounting frame |
| `front_camera_optical_frame` | Fixed to camera link | ROS camera optical frame |
| `docking_camera_0_link` | Fixed to base | Upward-facing docking camera 0 |
| `docking_camera_0_optical_frame` | Fixed to camera link | Docking camera 0 optical frame |
| `docking_camera_1_link` | Fixed to base | Upward-facing docking camera 1 |
| `docking_camera_1_optical_frame` | Fixed to camera link | Docking camera 1 optical frame |
| `bxu_docking_reference_frame` | Fixed to base | Nominal docking target/reference frame |

### Physical envelope

The chassis collision envelope is a simplified box:

| Dimension | Value |
|---|---:|
| Length, X | 1.650 m |
| Width, Y | 1.456 m |
| Height, Z | 0.114 m |

The visual mesh is offset and rotated to align the DAE asset with `bxu_base_link`. These offsets were tuned in RViz because the source mesh origin is not aligned with the desired footprint centre.

The collision geometry and inertial values are simplified placeholders. They are suitable for the current kinematic simulation but should be replaced before physics-based Gazebo tuning.

### Camera frames

| Camera | Position relative to `bxu_base_link` | Optical-frame convention |
|---|---|---|
| Front camera | `0.825 m`, `0 m`, `0.057 m` | ROS optical convention: `+Z` forward, `+X` right, `+Y` down |
| Docking camera 0 | `0.400 m`, `0.500 m`, `0.050 m` | Upward-facing |
| Docking camera 1 | `0.400 m`, `-0.500 m`, `0.050 m` | Upward-facing |

The docking-camera positions are preliminary measurements. Camera labels and the actual physical separation should be verified before relying on the configured baseline for docking yaw estimation.

### Locking mechanism

The physical BXU includes a patented locking mechanism that moves and secures the BU in place. It is an important part of the BXU design.

This mechanism is not currently represented as an actuated joint in the URDF and is not controlled by the ROS 2 software.

## BRU description

**File:** [`urdf/bru.urdf.xacro`](urdf/bru.urdf.xacro)

The BRU model represents the stationary Battery Receiving Unit.

### Coordinate convention

| Axis | Meaning |
|---|---|
| `+X` | BXU approach direction, out of the under-body opening |
| `+Y` | Left when looking along `+X` |
| `+Z` | Up |

`bru_base_link` is placed in the world by the simulation launch. `bru_center_link` is the centre of the under-body opening and battery receiver, and is the BXU docking target.

### Links and joints

| Link / joint | Type | Description |
|---|---|---|
| `bru_base_link` | Root link | BRU body and visual mesh |
| `bru_center_link` | Fixed to base | Centre of the under-body opening and battery receiver |
| `bru_lock_link` | Revolute about Z | Reference frame for the rotating locking mechanism |
| `bru_lock_joint` | Revolute | Lock rotation joint; no actuation or visual geometry is currently implemented |

The BRU is declared static in Gazebo. Gravity is disabled on its links in the current simulation setup.

### Mesh scaling

The BRU visual mesh is scaled by `0.001`. This assumes the STL was exported from FreeCAD in millimetres.

If the STL is already in metres, set `bru_mesh_scale` to `1.0`.

## Meshes

| Mesh | Used by | Notes |
|---|---|---|
| [`meshes/bxu/bxu_visual.dae`](meshes/bxu/bxu_visual.dae) | BXU visual model | Textured visual asset |
| [`meshes/bru/bru_visual.stl`](meshes/bru/bru_visual.stl) | BRU visual model | Millimetre-scale STL; scaled in the Xacro |

The current meshes are not fully up to date with the latest mechanical design, but they remain useful for simulation, visualization, camera-frame verification, and demonstration scenarios.

## RViz configuration

[`rviz/docking_sim.rviz`](rviz/docking_sim.rviz) is the simulation-oriented RViz configuration. It displays:

- BXU and BRU robot models.
- TF frames.
- Front-camera vision overlay.
- Docking-camera overlays.
- YOLO vision overlay.
- Raw camera images, disabled by default.

The fixed frame is `world`.

## Launch files

### BXU display

**File:** [`launch/bxu_display.launch.py`](launch/bxu_display.launch.py)

Starts the BXU robot-state publisher and RViz.

```bash
ros2 launch swas_description bxu_display.launch.py
```

The launch file publishes:

- `/bxu/robot_description`
- `/bxu/joint_states`

TODO: The launch file references `rviz/bxu.rviz`, which was not present in the provided package tree. Add the file or update the launch configuration.

### BRU display

**File:** [`launch/bru_display.launch.py`](launch/bru_display.launch.py)

Starts the BRU robot-state publisher, joint-state GUI, and RViz.

```bash
ros2 launch swas_description bru_display.launch.py
```

The launch file publishes:

- `/bru/robot_description`
- `/bru/joint_states`

TODO: The launch file references `rviz/bru.rviz`, which was not present in the provided package tree. Add the file or update the launch configuration.

## Simulation integration

[`swas_simulation`](../swas_simulation/README.md) processes these Xacro files, converts `package://swas_description/` mesh paths to absolute `file://` paths, adds simulated camera sensors and Gazebo plugins, and spawns the models into Gazebo.

The simulation adds:

- A velocity-control plugin mapped to `/model/bxu/cmd_vel`.
- An odometry publisher for `odom` and `bxu_base_link`.
- Front and docking camera sensors.
- A static `world` to `odom` transform.
- A static `world` to `bru_base_link` transform.

## Package metadata

TODO: Complete `package.xml`:

- Replace the package description.
- Add the intended license.
- Add explicit dependencies where required by the launch files, especially `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.

## Related documentation

- [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)
- [`swas_simulation`](../swas_simulation/README.md)
- [`swas_system`](../swas_system/README.md)
- [Hardware](../../../docs/HARDWARE.md)