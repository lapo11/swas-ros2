# Launch Files

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

The launch files in this package configure hardware operation. Simulation launch files live in [`swas_simulation`](../../swas_simulation/README.md) and reuse these nodes with ROS image inputs and `use_sim_time` enabled.

`bxu_demo.launch.py` is the main hardware demonstration entry point. The other files launch subsets of the system for isolated testing.

| Launch file | Purpose |
|---|---|
| [`bxu_demo.launch.py`](../launch/bxu_demo.launch.py) | Complete marker-based hardware demonstration: front approach, docking, supervisor, motion, and battery lift |
| [`bxu_docking.launch.py`](../launch/bxu_docking.launch.py) | Docking-only hardware test |
| [`bxu_front_alignment.launch.py`](../launch/bxu_front_alignment.launch.py) | Marker-based front-approach hardware test |
| [`bxu_full_alignment.launch.py`](../launch/bxu_full_alignment.launch.py) | Full marker-based alignment hardware test without battery lift |

## `bxu_demo.launch.py`

**File:** [`launch/bxu_demo.launch.py`](../launch/bxu_demo.launch.py)

Starts the complete marker-based hardware workflow:

```text
Front vision
Front approach controller
Docking vision
Docking controller
Supervisor
Motion node
Battery-lift node
```

### Nodes and configuration

| Node | Configuration |
|---|---|
| `bxu_front_vision_node` | [`config/bxu_front_vision.yaml`](../config/bxu_front_vision.yaml) |
| `bxu_docking_vision_node` | [`config/bxu_docking_vision.yaml`](../config/bxu_docking_vision.yaml) |
| `bxu_front_approach_controller_node` | [`config/bxu_front_approach_controller.yaml`](../config/bxu_front_approach_controller.yaml) |
| `bxu_docking_controller_node` | [`config/bxu_docking_controller.yaml`](../config/bxu_docking_controller.yaml) |
| `bxu_supervisor_node` | Inline launch parameters |
| `bxu_motion_node` | [`config/bxu_motion.yaml`](../config/bxu_motion.yaml) |
| `bxu_battery_lift_node` | Inline launch parameters |

### Supervisor parameters

| Parameter | Value |
|---|---:|
| `auto_start` | `true` |
| `drive_away_speed` | `-0.15` m/s |
| `drive_away_duration` | `15.0` s |

### Battery-lift parameters

| Parameter | Value |
|---|---:|
| Relay port | Relay Pico serial-by-id path |
| `battery_up_duration` | `1.5` s |
| `battery_down_duration` | `5.0` s |

### Run

```bash
ros2 launch swas_system bxu_demo.launch.py
```

## `bxu_docking.launch.py`

**File:** [`launch/bxu_docking.launch.py`](../launch/bxu_docking.launch.py)

Starts docking vision, docking control, supervisor, and motion. It tests precise docking when the BXU is already close to the BRU.

The supervisor is configured with:

```python
auto_start = True
start_state = 'DOCKING_ALIGN'
```

### Run

```bash
ros2 launch swas_system bxu_docking.launch.py
```

## `bxu_front_alignment.launch.py`

**File:** [`launch/bxu_front_alignment.launch.py`](../launch/bxu_front_alignment.launch.py)

Starts front vision, docking vision, marker-based front-approach control, supervisor, and motion. It tests the marker-based front-approach stage without the docking controller or battery lift.

### Run

```bash
ros2 launch swas_system bxu_front_alignment.launch.py
```

## `bxu_full_alignment.launch.py`

**File:** [`launch/bxu_full_alignment.launch.py`](../launch/bxu_full_alignment.launch.py)

Starts front vision, docking vision, marker-based front approach, docking control, supervisor, and motion. It tests the complete alignment workflow without the battery-lift node.

### Run

```bash
ros2 launch swas_system bxu_full_alignment.launch.py
```

## YOLO-based operation

The YOLO-based front-approach pipeline has been tested in simulation only. No dedicated hardware launch file is currently provided.

A YOLO launch configuration must start:

- `bxu_front_yolo_vision_node`
- `bxu_front_yolo_approach_controller_node`
- `bxu_docking_vision_node`
- `bxu_docking_controller_node`
- `bxu_supervisor_node`
- `bxu_motion_node`

It must not start `bxu_front_vision_node` or `bxu_front_approach_controller_node`, because both front controllers publish to `/bxu/cmd_vel` and use the same enable topic.

The YOLO controller is designed as a drop-in replacement for the marker-based front controller:

- It subscribes to `/bxu/front_controller/enabled`.
- It publishes status to `/bxu/front_approach/controller_status`.
- It publishes velocity commands to `/bxu/cmd_vel`.
- It uses `/bxu/docking_vision/alignment` to detect docking markers during blind forward.

See the simulation launch [`docking_tesla_yolo_approach_sim.launch.py`](../../swas_simulation/launch/docking_tesla_yolo_approach_sim.launch.py) for the current simulation-only YOLO configuration.

## Environment configuration

The hardware launch files currently set:

```text
GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0
LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/usr/local/lib/aarch64-linux-gnu:...
```

TODO: Remove the user-specific `LD_LIBRARY_PATH` entries and rely on sourcing the workspace:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch swas_system bxu_demo.launch.py
```

If the BXU requires a non-standard GStreamer plugin path, retain it as an explicit deployment setting or expose it as a launch argument.

## Related documentation

- [Supervisor](supervisor.md)
- [Motion control](motion.md)
- [Battery lift](battery_lift.md)
- [Docking vision and control](docking.md)
- [Marker-based front approach](front_marker_approach.md)
- [YOLO-based front approach](front_yolo_approach.md)
- [Configuration reference](configuration.md)