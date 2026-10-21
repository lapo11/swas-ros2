# Launch Files

[← `swas_simulation`](../README.md) · [← Repository README](../../../../README.md)

## Main demonstrations

| Launch file | Pipeline | Use |
|---|---|---|
| [`demo_docking_bru_markers_sim.launch.py`](../launch/demo_docking_bru_markers_sim.launch.py) | Marker-based front approach and docking | Main marker-based simulation demonstration |
| [`docking_tesla_yolo_approach_sim.launch.py`](../launch/docking_tesla_yolo_approach_sim.launch.py) | YOLO-based front approach and docking | Main YOLO-based simulation demonstration |
| [`bxu_gate_demo_docking_bru_markers_sim.launch.py`](../launch/bxu_gate_demo_docking_bru_markers_sim.launch.py) | Marker-based front approach and docking with command gating | HMI/teleop integration; under development |

## `demo_docking_bru_markers_sim.launch.py`

**File:** [`launch/demo_docking_bru_markers_sim.launch.py`](../launch/demo_docking_bru_markers_sim.launch.py)

This is the main marker-based simulation demonstration. It starts:

- The combined BRU marker scene.
- `bxu_front_vision_node`
- `bxu_docking_vision_node`
- `bxu_front_approach_controller_node`
- `bxu_docking_controller_node`
- `bxu_battery_lift_sim_node`
- `bxu_supervisor_node`, when `launch_supervisor=true`

All application nodes use `use_sim_time: true` and ROS image inputs.

### Run

```bash
ros2 launch swas_simulation demo_docking_bru_markers_sim.launch.py
```

### Start and control the demo

The supervisor starts in `WAITING`. In another sourced terminal:

```bash
ros2 service call /bxu/supervisor/start std_srvs/srv/Trigger
```

When the supervisor reaches `WAIT_USER_AFTER_LIFT` or `WAIT_USER_AFTER_LOWER`:

```bash
ros2 service call /bxu/supervisor/confirm std_srvs/srv/Trigger
```

To stop the demonstration:

```bash
ros2 service call /bxu/supervisor/stop std_srvs/srv/Trigger
```

To monitor the mission state:

```bash
ros2 topic echo /bxu/supervisor/state
```

### Key arguments

| Argument | Default | Description |
|---|---|---|
| `preset` | `manual` | BRU pose preset |
| `launch_supervisor` | `true` | Start the supervisor |
| `drive_away_speed` | `-0.15` m/s | Supervisor drive-away velocity |
| `drive_away_duration` | `15.0` s | Supervisor drive-away duration |
| `front_vision_params` | `swas_system` front-vision YAML | Front-vision parameter file |
| `front_controller_params` | `swas_system` front-controller YAML | Front-controller parameter file |
| `docking_vision_params` | `swas_system` docking-vision YAML | Docking-vision parameter file |
| `docking_controller_params` | `swas_system` docking-controller YAML | Docking-controller parameter file |

The launch also forwards scene arguments such as BXU pose, BRU pose, front-marker pose, and docking-marker poses to `docking_bru_markers_sim.launch.py`.

If any application node exits, the launch description shuts down the remaining demo processes.

## `docking_tesla_yolo_approach_sim.launch.py`

**File:** [`launch/docking_tesla_yolo_approach_sim.launch.py`](../launch/docking_tesla_yolo_approach_sim.launch.py)

This is the main YOLO-based simulation demonstration. It includes [`docking_tesla_yolo_sim.launch.py`](../launch/docking_tesla_yolo_sim.launch.py), which provides the Tesla scene, Gazebo integration, and YOLO vision node, then adds:

```text
bxu_front_yolo_approach_controller_node
```

### Run

```bash
ros2 launch swas_simulation docking_tesla_yolo_approach_sim.launch.py
```

### Start and stop the controller

The YOLO controller does not start enabled. Start it with:

```bash
ros2 service call /bxu/front_yolo_approach/start std_srvs/srv/Trigger
```

Stop it with:

```bash
ros2 service call /bxu/front_yolo_approach/stop std_srvs/srv/Trigger
```

### Key arguments

| Argument | Default | Description |
|---|---|---|
| `controller_params` | `bxu_front_yolo_approach_controller.yaml` | YOLO controller parameters |
| `world` | `empty` | Gazebo world |
| `preset` | `manual` | BRU pose preset |
| `bxu_x`, `bxu_y`, `bxu_yaw` | `0.0`, `0.0`, `0.0` | BXU spawn pose |
| `bru_x`, `bru_y`, `bru_z`, `bru_yaw` | `5.0`, `0.0`, `0.15`, `0.0` | BRU pose |
| `tesla_x`, `tesla_y`, `tesla_z` | `0.0`, `-0.11`, `-0.15` | Vehicle-model pose |
| `yolo_model` | `models/first_yolov8_24092026.onnx` | YOLO ONNX model |
| `yolo_confidence` | `0.30` | YOLO confidence threshold |
| `yolo_publish_rate_hz` | `15.0` | YOLO output rate |
| `yolo_max_frame_age_s` | `1.0` | Maximum accepted frame age |

The controller is launched with:

```python
use_sim_time = True
enable_control_services = True
```

## `bxu_gate_demo_docking_bru_markers_sim.launch.py`

**File:** [`launch/bxu_gate_demo_docking_bru_markers_sim.launch.py`](../launch/bxu_gate_demo_docking_bru_markers_sim.launch.py)

This launch file is based on the marker-based simulation demonstration and adds command gating for HMI/teleoperation integration.

It remaps autonomous controller and supervisor velocity output:

```text
/bxu/cmd_vel -> /bxu/auto/cmd_vel
```

This separates autonomous BXU commands from operator or gated commands. The exact HMI/gate integration is still under development.

TODO: Document the final HMI command-gate topic flow once the `swas_hmi` integration is stabilized.

## Base scene launch

[`docking_sim.launch.py`](../launch/docking_sim.launch.py) is the common Gazebo scene launcher. It:

- Processes the BXU and BRU Xacro descriptions.
- Converts `package://swas_description/` mesh paths to absolute `file://` paths for Gazebo.
- Adds the front and docking camera sensor configurations to the BXU description.
- Adds Gazebo velocity-control and odometry plugins to the BXU.
- Disables gravity on BXU links for the kinematic prototype.
- Spawns the BXU and BRU.
- Publishes world-to-odom and world-to-BRU-base transforms.
- Starts the Gazebo–ROS bridge.
- Starts RViz with [`swas_description/rviz/docking_sim.rviz`](../../swas_description/rviz/docking_sim.rviz).

### World argument

| Value | World |
|---|---|
| `empty` | `empty_docking_world.sdf` |
| `station` | `swas_station_world.sdf` |
| `station_w_ceiling` | `swas_station_with_ceiling_world.sdf` |

### BRU pose presets

| Preset | BRU offset from reference | Meaning |
|---|---|---|
| `front` | `0 m`, `0 m`, `0 deg` | BRU directly ahead |
| `lateral` | `0 m`, `-0.80 m`, `0 deg` | BRU displaced laterally |
| `rotated` | `0 m`, `0 m`, `-35 deg` | BRU rotated |
| `combined` | `0 m`, `-0.80 m`, `-35 deg` | Lateral and rotational offset |
| `away` | `-7 m`, `0 m`, `180 deg` | BRU placed away from the BXU |
| `manual` | Uses explicit pose arguments | No preset offset |

## Related documentation

- [`swas_system`](../../swas_system/README.md)
- [YOLO-based front approach](../../swas_system/docs/front_yolo_approach.md)
- [Marker-based front approach](../../swas_system/docs/front_marker_approach.md)
- [Docking vision and control](../../swas_system/docs/docking.md)
- [Supervisor](../../swas_system/docs/supervisor.md)
- [`swas_hmi`](../../swas_hmi/README.md)