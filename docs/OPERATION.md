# Operation

[← Repository README](../README.md) · [System architecture](SYSTEM_ARCHITECTURE.md)

This document describes how to build, launch, operate, monitor, and troubleshoot SWAS.

## Prerequisites

- Ubuntu with ROS 2 Jazzy.
- Gazebo and `ros_gz` packages.
- Python 3 with `rclpy`, NumPy, OpenCV, ONNX Runtime, Ultralytics, and pyserial.
- Node.js 22.12 or newer and npm for the HMI.
- Camera and serial-device access on the BXU.
- rosbridge and `web_video_server` for HMI operation.

TODO: Add the exact apt, pip, and ROS dependency installation commands.

## Workspace setup

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

For iterative development, use:

```bash
colcon build --symlink-install
source install/setup.bash
```

TODO: Confirm whether `rosdep` is used and whether any package-specific build flags are required.

## Simulation

### Marker-based demonstration

```bash
ros2 launch swas_simulation demo_docking_bru_markers_sim.launch.py
```

This starts the Gazebo scene, marker-based front vision and controller, docking vision and controller, simulated battery lift, supervisor, and RViz.

The supervisor starts in `WAITING`. In another sourced terminal:

```bash
ros2 service call /bxu/supervisor/start std_srvs/srv/Trigger
```

When the supervisor reaches `WAIT_USER_AFTER_LIFT` or `WAIT_USER_AFTER_LOWER`:

```bash
ros2 service call /bxu/supervisor/confirm std_srvs/srv/Trigger
```

To stop:

```bash
ros2 service call /bxu/supervisor/stop std_srvs/srv/Trigger
```

To monitor the mission:

```bash
ros2 topic echo /bxu/supervisor/state
```

### YOLO-based demonstration

```bash
ros2 launch swas_simulation docking_tesla_yolo_approach_sim.launch.py
```

Start the YOLO front-approach controller:

```bash
ros2 service call /bxu/front_yolo_approach/start std_srvs/srv/Trigger
```

Stop it with:

```bash
ros2 service call /bxu/front_yolo_approach/stop std_srvs/srv/Trigger
```

The YOLO workflow has been tested in simulation only.

## Hardware operation

### Startup checklist

1. Power the BXU and BRU.
2. Confirm network connectivity to both Raspberry Pi 5 units.
3. Confirm the motion and relay Picos appear at their configured serial-by-id paths.
4. Confirm cameras are detected and publishing.
5. Confirm the operating area is clear.
6. Build and source the workspace.
7. Launch the required hardware configuration.
8. Start rosbridge and `web_video_server` if using the HMI.
9. Verify status, camera streams, and safety indicators before commanding motion.

### Marker-based hardware demonstration

```bash
ros2 launch swas_system bxu_demo.launch.py
```

This is the main hardware demonstration. It starts marker-based front approach, docking, supervisor, motion, and battery lift.

### Other hardware configurations

| Purpose | Command |
|---|---|
| Docking-only test | `ros2 launch swas_system bxu_docking.launch.py` |
| Marker-based front-approach test | `ros2 launch swas_system bxu_front_alignment.launch.py` |
| Full alignment test without lift | `ros2 launch swas_system bxu_full_alignment.launch.py` |

There is currently no dedicated hardware launch file for the YOLO-based front approach.

### HMI operation

In separate sourced terminals:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
ros2 run web_video_server web_video_server
```

Then start the HMI:

```bash
cd ros2_ws/src/swas_hmi
npm run dev
```

Open `http://localhost:5173`, select the appropriate profile, and connect.

On another device on the same trusted Wi-Fi network, use the operator laptop’s IP address instead of `localhost`.

## Shutdown

1. Stop active motion or transfer commands.
2. Confirm the lift and actuation are in a safe state.
3. Stop the ROS 2 launch processes.
4. Disconnect or shut down the HMI.
5. Stop rosbridge and the video server.
6. Power down hardware in the documented order.

TODO: Add the final hardware power-down sequence and lift/actuation safe-state procedure.

## Monitoring

Useful commands:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /bxu/supervisor/state
ros2 topic echo /bxu/front_approach/controller_status
ros2 topic echo /bxu/docking/controller_status
ros2 topic echo /bxu/motion/status
ros2 topic info /bxu/cmd_vel --verbose
ros2 param list /bxu_supervisor_node
```

For camera inspection:

```bash
ros2 topic echo /bxu/front_camera/camera_info --once
ros2 topic hz /bxu/front_camera/image_raw
```

## Launch reference

| Launch file | Package | Purpose |
|---|---|---|
| `bxu_demo.launch.py` | `swas_system` | Main marker-based hardware demonstration |
| `bxu_docking.launch.py` | `swas_system` | Docking-only hardware test |
| `bxu_front_alignment.launch.py` | `swas_system` | Marker-based front-approach hardware test |
| `bxu_full_alignment.launch.py` | `swas_system` | Full marker-based alignment hardware test without lift |
| `demo_docking_bru_markers_sim.launch.py` | `swas_simulation` | Main marker-based simulation demonstration |
| `docking_tesla_yolo_approach_sim.launch.py` | `swas_simulation` | Main YOLO-based simulation demonstration |
| `bxu_gate_demo_docking_bru_markers_sim.launch.py` | `swas_simulation` | Marker-based simulation with HMI/teleop command gating; under development |

## Troubleshooting

| Symptom | Likely cause | Check | Recovery |
|---|---|---|---|
| No camera image | Camera device, driver, or topic mismatch | `ros2 topic list`, `ros2 topic hz` | Correct device/topic configuration and restart the vision node |
| Marker detection unstable | Lighting, exposure, marker visibility, or calibration issue | Inspect camera overlay and marker visibility | Improve lighting/marker visibility; verify calibration |
| YOLO model fails to load | Missing ONNX Runtime/Ultralytics or invalid model metadata | Check node log and model path | Install dependencies and verify ONNX model metadata |
| Serial device missing | Pico disconnected or device path changed | `ls /dev/serial/by-id/` | Reconnect device or update configured serial-by-id path |
| BXU does not move | Controller disabled, zero command, serial fault, or conflicting publisher | Check `/bxu/cmd_vel`, controller status, and motion status | Enable the intended controller; resolve publisher conflicts |
| Motion-driver alarm | Stall, position-following error, or EMI | Check HB860H status and ALM wiring | Stop motion, clear fault according to hardware procedure, and retry |
| Docking does not converge | Marker loss, incorrect calibration, or control gain issue | Check docking alignment topic and overlay | Restore marker visibility, verify alignment-zero calibration, retune if necessary |
| HMI cannot connect | rosbridge/video server not running or wrong host/port | Check browser System page and service terminals | Start services and correct endpoints |
| HMI telemetry is silent | QoS mismatch, wrong domain, or missing custom message support | Use `ros2 topic echo` in the same sourced environment | Align ROS environment and inspect bridge logs |
| Supervisor Start disabled | Supervisor not in `WAITING`, services absent, or simulation clock paused | Check supervisor state and service list | Reset supervisor; unpause Gazebo; ensure `use_sim_time` is enabled |

## Known limitation

The supervisor and hardware battery-lift node currently use different lift topic names. The supervisor-driven lift sequence will not reach the hardware lift node until these topics are aligned.

See [System architecture](SYSTEM_ARCHITECTURE.md#known-integration-issue).