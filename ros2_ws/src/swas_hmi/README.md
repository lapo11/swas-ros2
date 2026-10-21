# `swas_hmi`

[← Repository README](../../../README.md) · [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)

`swas_hmi` is the SWAS operator station: a browser-based interface for monitoring the BXU/BRU system, controlling the supervisor workflow, viewing camera streams, and optionally jogging the BXU in simulation.

It runs on the operator laptop and connects to the BXU and BRU Raspberry Pi 5 units over the local Wi-Fi network.

## Purpose

The HMI provides a single operator-facing view of the system. It does not replace ROS 2 control nodes, embedded watchdogs, or physical safety functions.

Current capabilities:

- Connect to rosbridge and the video server.
- Monitor supervisor state and selected system topics.
- Start, confirm, and stop the supervisor workflow.
- View camera snapshot previews.
- Inspect ROS discovery, subscription, and bridge status.
- Optionally request manual jog authority through the simulation command gate.

## Current status

The HMI is under development.

| Capability | Status |
|---|---|
| rosbridge telemetry and service calls | Implemented |
| Supervisor start / confirm / stop | Implemented for simulation-enabled supervisor |
| Camera snapshot previews | Implemented |
| BRU lock and battery-seating monitoring | Monitoring only; topics are optional and not synthesized |
| Manual BXU jogging | Optional, simulation-only through the command gate |
| Hardware supervisor control | Not available in this release |
| Emergency stop or lift cancellation | Not implemented |

The supervisor creates `/bxu/supervisor/start`, `/bxu/supervisor/confirm`, and `/bxu/supervisor/stop` only when `use_sim_time` is true. Therefore, the HMI can control the supplied simulation workflow, but hardware profiles are monitor-only until the supervisor exposes the same service API on hardware.

## System architecture

```text
+---------------------+
|   Operator laptop   |
|      swas_hmi       |
+----------+----------+
           |
           | WebSocket / HTTP
           v
+---------------------+        +----------------------+
|   rosbridge_server  |        |  web_video_server    |
|     port 9090       |        |      port 8080       |
+----------+----------+        +----------+-----------+
           |                              |
           v                              v
+------------------------------------------------------+
|                  ROS 2 system                        |
|        BXU / BRU nodes and simulation nodes          |
+------------------------------------------------------+
```

Default endpoints:

| Service | Default URL |
|---|---|
| rosbridge WebSocket | `ws://<page-host>:9090` |
| Video server | `http://<page-host>:8080` |

The UI defaults to the page hostname. On the laptop, this is normally `localhost`. On a phone or another computer, use the operator laptop’s Wi-Fi IP address.

## Required services

Start the SWAS ROS 2 demonstration first, then keep the following services running:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
ros2 run web_video_server web_video_server
```

Source the same workspace and use the same `ROS_DOMAIN_ID` and middleware configuration as the running simulation or hardware system.

The HMI expects:

- rosbridge on port `9090`
- `web_video_server` on port `8080`

## Camera previews

Camera previews use sequential JPEG snapshots from `web_video_server`; they are not full-rate video streams.

Example snapshot URL:

```text
http://localhost:8080/snapshot?topic=/bxu/front_vision/image_raw&type=jpeg&qos_profile=sensor_data
```

The HMI preserves literal topic slashes. Earlier versions encoded `/` as `%2F`, which could be interpreted incorrectly by the video server.

Preview limits:

- Default maximum rate: 4 snapshots per second.
- Maximum simultaneous views: 3.
- Hidden pages stop preview requests.
- A hidden browser document pauses previews.

Camera topics currently mapped by the UI include:

| Camera | Topic |
|---|---|
| Front marker vision | `/bxu/front_vision/image_raw` |
| Front YOLO vision | `/bxu/front_yolo_vision/image_raw` |
| Docking camera 0 overlay | `/bxu/docking_camera_0/overlay` |
| Docking camera 1 overlay | `/bxu/docking_camera_1/overlay` |

The marker-based alignment topic is preferred when both marker and YOLO alignment topics are discovered.

## Supervisor workflow

The default workflow uses the supervisor interfaces:

| Interface | Type | Purpose |
|---|---|---|
| `/bxu/supervisor/start` | `std_srvs/srv/Trigger` | Start the battery-exchange workflow |
| `/bxu/supervisor/confirm` | `std_srvs/srv/Trigger` | Continue after lift or lower |
| `/bxu/supervisor/stop` | `std_srvs/srv/Trigger` | Stop the workflow |
| `/bxu/supervisor/state` | `std_msgs/msg/String` | Supervisor state |
| `/bxu_battery_lift/state` | `std_msgs/msg/String` | Battery-lift state |

The HMI enables **Start** only when:

- The start service is discovered.
- The supervisor reports a recent `WAITING` state.
- The simulation clock is advancing.

**Continue** appears in `WAIT_USER_AFTER_LIFT` and `WAIT_USER_AFTER_LOWER`. Both Start and Continue require operator confirmation.

**Stop** does not request confirmation. It calls the supervisor stop service and, if present, requests the command gate to stop.

A service timeout is shown as `UNKNOWN OUTCOME` and is not automatically retried.

### Stop behavior

Stop is not an emergency stop and does not cancel lift actuation. It requests:

- Supervisor stop.
- Controller disable.
- Zero velocity.
- Command-gate stop, when available.

The supplied simulation demo uses a drive-away speed of `-0.15 m/s` for `15 s`. Review these launch arguments before running the demonstration.

## BXU and BRU monitoring

The overview separates BXU reports from BRU lock and battery-seating reports.

Optional BRU topics:

| Topic | Meaning |
|---|---|
| `/bru/lock/status` | BRU lock status |
| `/bru/battery_seated` | Whether a BU is seated in the BRU |
| `/bru/joint_states` | BRU joint-model state |

These topics are shown as unavailable when they are not present. The HMI does not synthesize BRU state from simulation data.

No BRU lock or unlock commands are implemented in this release.

## Manual movement and command gate

The normal demonstration has multiple publishers on `/bxu/cmd_vel`: the front controller, docking controller, and supervisor. Direct teleoperation on the same topic would conflict with those publishers.

The optional simulation gate routes autonomous commands through an intermediate topic:

```text
Front controller ---+
Docking controller -+--> /bxu/auto/cmd_vel --> command gate --> /bxu/cmd_vel --> Gazebo
Supervisor ---------+
                         HMI --> /bxu/teleop/cmd_vel --+
```

The gate is started separately:

```bash
python3 tools/bxu_command_gate.py
```

It starts in `STOP` and publishes to `/bxu/cmd_vel`.

### Gate services

| Service | Purpose |
|---|---|
| `/bxu/control/auto` | Select automatic authority |
| `/bxu/control/manual` | Select manual authority |
| `/bxu/control/stop` | Stop the gate |

### Gate interlocks

Manual or automatic authority requires:

- An advancing simulation clock.
- Supervisor state `WAITING`.
- Both front and docking controllers disabled.
- Battery-lift state `IDLE`, `LIFTED`, or `LOWERED`.

Manual input expires after `0.35 s` and does not fall back to automatic control. A stale input produces zero velocity.

### Gate limits

| Mode | Linear limit per axis | Yaw-rate limit |
|---|---:|---:|
| Manual | 0.20 m/s | 0.15 rad/s |
| Automatic | 0.15 m/s | 0.40 rad/s |

The gate is simulation-only. It is not a safety controller, does not authenticate operators, and does not verify the physical system identity.

## Development

Requirements:

- Node.js 22.12 or newer.
- npm.
- ROS 2 Jazzy environment.
- Running rosbridge and video server.

```bash
cd ros2_ws/src/swas_hmi
npm install
npm test
npm run build
npm run dev
```

Open:

```text
http://localhost:5173
```

Select **Simulation**, then **Connect**.

The development server is intended for local development. For deployment, use a maintained local static server or reverse proxy rather than the Vite development server.

## Tests

Run the JavaScript tests with:

```bash
npm test
```

Run the gate logic tests with:

```bash
python3 tools/test_gate_core.py
```

The tests cover:

- rosbridge discovery, subscription, and message routing.
- Preservation of literal camera-topic slashes.
- Service response semantics.
- Refusal of motion when offline.
- Simulation command-gate arbitration logic.

## Security and safety boundaries

The HMI and command gate do not implement:

- User authentication.
- Exclusive multi-operator ownership.
- Replay protection.
- Hardware safety certification.
- Motor-driver watchdogs.
- Physical emergency stopping.
- Lift cancellation.
- Deployment network security.

Keep robot-local watchdogs and physical stopping independent of the browser, Wi-Fi connection, and HMI.

Do not expose the unauthenticated development services to the internet.

## Related documentation

- [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)
- [`swas_system`](../swas_system/README.md)
- [`swas_simulation`](../swas_simulation/README.md)
- [Supervisor](../swas_system/docs/supervisor.md)
- [Battery lift](../swas_system/docs/battery_lift.md)