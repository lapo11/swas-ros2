# SWAS — Automated EV Battery Swap System

SWAS is a prototype automated battery-swap system developed by **RU Technologies**.  
It demonstrates the exchange of a Battery Unit (BU) between a stationary **Battery Receiving Unit (BRU)** and an autonomous **Battery Exchange Unit (BXU)**.

The repository contains the ROS 2 software, Gazebo simulation environment, robot descriptions, operator HMI, YOLO model and dataset material, CAD references, and development scripts.

**Software implementation:** Lapo Veca.

[← Back to top](#swas--automated-ev-battery-swap-system)

## System overview

SWAS is a hardware-focused prototype intended for working demonstrations of an automated EV battery exchange.

- The **BU** is the removable EV battery pack.
- The **BRU** is the stationary Battery Receiving Unit, based on a retrofitted Tesla battery pack. It receives, locks, and releases a BU.
- The **BXU** is an autonomous mecanum-drive Battery Exchange Unit. It carries a BU beneath the BRU, aligns itself, transfers the BU with a lift plate, and drives away.
- The **operator laptop** runs the HMI and connects to the BXU and BRU Raspberry Pi 5 units over local Wi-Fi.
- Both the BXU and BRU use a Raspberry Pi 5 as their main compute platform.

The system supports real-hardware operation and a Gazebo simulation environment. The marker-based front approach, docking, supervisor, and motion-control nodes are used in both. The YOLO-based front-approach pipeline has been tested in simulation only.

[← Back to top](#swas--automated-ev-battery-swap-system)

## Battery exchange concept

The BXU first performs front approach: it detects and roughly centers itself beneath the BRU. It then performs docking: precise final alignment before battery transfer.

### Installing a BU into the BRU

1. The BXU approaches and roughly centers itself beneath the BRU.
2. The BXU performs precise docking alignment.
3. The BXU lift plate rises, pushing the BU into the BRU.
4. The BRU locking mechanism engages and secures the BU.
5. The BXU lift plate lowers.
6. The BXU drives away, leaving the BU installed in the BRU.

### Removing a BU from the BRU

1. The BXU approaches and aligns itself beneath the BRU.
2. The BXU lift plate rises into position beneath the BU.
3. The BRU locking mechanism releases the BU.
4. The BXU lift plate lowers, carrying the BU on the BXU.
5. The BXU drives away with the BU.

The BXU also contains a patented locking mechanism that moves and secures the BU on the BXU. It is a key mechanical design element but is not currently actuated or controlled by the ROS 2 software.

[← Back to top](#swas--automated-ev-battery-swap-system)

## Software architecture

The ROS 2 workspace is in [`ros2_ws/src`](ros2_ws/src).

| Package | Purpose |
|---|---|
| [`swas_system`](ros2_ws/src/swas_system/README.md) | BXU runtime software: supervisor, vision, alignment, docking, motion, and battery-lift control |
| [`swas_description`](ros2_ws/src/swas_description/README.md) | BXU and BRU URDF/Xacro descriptions, meshes, camera frames, RViz configuration, and display launches |
| [`swas_msgs`](ros2_ws/src/swas_msgs/README.md) | Custom messages for marker detections, alignment estimates, and YOLO observations |
| [`swas_simulation`](ros2_ws/src/swas_simulation/README.md) | Gazebo worlds, simulated cameras and markers, Gazebo–ROS bridge, and simulation launch files |
| [`swas_hmi`](ros2_ws/src/swas_hmi/README.md) | Browser-based operator interface for monitoring, supervisor control, camera views, and optional simulated teleoperation |

```text
Operator laptop / swas_hmi
            |
            | rosbridge and web video server
            v
BXU ROS 2 system / swas_system
            |
            +--> Perception
            |      |
            |      v
            +--> Front approach / docking control
            |      |
            |      v
            +--> Motion and battery-lift control
                   |
                   v
             BXU hardware or Gazebo
```

### Front-approach implementations

| Pipeline | Vision | Controller | Status |
|---|---|---|---|
| Marker-based | `bxu_front_vision_node` | `bxu_front_approach_controller_node` | Hardware and simulation |
| YOLO-based | `bxu_front_yolo_vision_node` | `bxu_front_yolo_approach_controller_node` | Simulation only |

Only one front-approach controller is active in a given configuration. Both use `/bxu/cmd_vel` and `/bxu/front_controller/enabled`, so they must not run simultaneously.

After either front-approach pipeline, `bxu_docking_vision_node` and `bxu_docking_controller_node` perform the final precise alignment.

[← Back to top](#swas--automated-ev-battery-swap-system)

## Repository structure

| Path | Contents |
|---|---|
| [`ros2_ws/src`](ros2_ws/src) | ROS 2 source packages |
| [`cad_drawings`](cad_drawings) | CAD models, drawings, and mechanical references |
| [`datasets`](datasets) | Dataset preparation and YOLO training data |
| [`models`](models) | Trained ONNX model artifacts |
| [`scripts`](scripts) | Standalone vision, dataset, and testing scripts |
| [`docs`](docs) | System-level documentation |

`ros2_ws/build`, `ros2_ws/install`, and `ros2_ws/log` are generated artifacts and are not part of the documented source architecture.

[← Back to top](#swas--automated-ev-battery-swap-system)

## Documentation map

### System-level documentation

- [System architecture](docs/SYSTEM_ARCHITECTURE.md)
- [Hardware](docs/HARDWARE.md)
- [Operation](docs/OPERATION.md)
- [Documentation guidelines](docs/DOCUMENTATION_GUIDELINES.md)

### Package documentation

- [`swas_system`](ros2_ws/src/swas_system/README.md)
- [`swas_description`](ros2_ws/src/swas_description/README.md)
- [`swas_msgs`](ros2_ws/src/swas_msgs/README.md)
- [`swas_simulation`](ros2_ws/src/swas_simulation/README.md)
- [`swas_hmi`](ros2_ws/src/swas_hmi/README.md)

[← Back to top](#swas--automated-ev-battery-swap-system)

## Quick start

The system targets ROS 2 Jazzy. Build and source the workspace before launching any nodes:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### Marker-based simulation demonstration

```bash
ros2 launch swas_simulation demo_docking_bru_markers_sim.launch.py
```

In a second sourced terminal:

```bash
ros2 service call /bxu/supervisor/start std_srvs/srv/Trigger
```

When the supervisor reaches `WAIT_USER_AFTER_LIFT` or `WAIT_USER_AFTER_LOWER`:

```bash
ros2 service call /bxu/supervisor/confirm std_srvs/srv/Trigger
```

### YOLO-based simulation demonstration

```bash
ros2 launch swas_simulation docking_tesla_yolo_approach_sim.launch.py
```

Start the YOLO approach controller:

```bash
ros2 service call /bxu/front_yolo_approach/start std_srvs/srv/Trigger
```

### Hardware demonstration

```bash
ros2 launch swas_system bxu_demo.launch.py
```

`bxu_demo.launch.py` is the main marker-based hardware demonstration. See [Operation](docs/OPERATION.md) for startup, monitoring, and shutdown procedures.

[← Back to top](#swas--automated-ev-battery-swap-system)

## Safety

The BXU contains powered motion, lifting, and actuation hardware. Before operating the physical system:

- Ensure the operating area is clear of people and obstacles.
- Verify that emergency-stop and fault-recovery procedures are understood.
- Confirm motor drivers, serial connections, cameras, and lift hardware are correctly connected.
- Do not place hands or tools near the lift plate, BU, BRU locking mechanism, or drive wheels during operation.
- Use the startup and shutdown procedures in [Operation](docs/OPERATION.md).

The HMI stop function and simulation command gate are not emergency stops. They do not cancel lift actuation or guarantee that another velocity publisher will stop sending commands.

TODO: Add the final hardware-specific emergency-stop procedure, driver-fault recovery procedure, and safe power-up/power-down sequence.

[← Back to top](#swas--automated-ev-battery-swap-system)