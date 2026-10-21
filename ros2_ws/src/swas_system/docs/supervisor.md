# Supervisor

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

## Purpose

`bxu_supervisor_node` coordinates the complete BXU battery-transfer mission. It enables and disables the front-approach and docking controllers, commands the battery lift, publishes zero velocity between stages, and drives the BXU away after the transfer.

Source: [`swas_system/bxu/supervisor_node.py`](../swas_system/bxu/supervisor_node.py)

## Summary

The supervisor runs a fixed state machine:

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

Any front-approach fault or loss of docking markers transitions the mission to `FAULT`.

## Interfaces

### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `/bxu/front_approach/controller_status` | `std_msgs/String` | Front-approach controller state |
| `/bxu/docking/controller_status` | `std_msgs/String` | Docking-controller state |
| `/bxu_battery_lift/state` | `std_msgs/String` | Battery-lift state |

### Publishers

| Topic | Type | Purpose |
|---|---|---|
| `/bxu/supervisor/state` | `std_msgs/String` | Current supervisor state |
| `/bxu/front_controller/enabled` | `std_msgs/Bool` | Enables or disables the active front-approach controller |
| `/bxu/docking_controller/enabled` | `std_msgs/Bool` | Enables or disables the docking controller |
| `/bxu/cmd_vel` | `geometry_msgs/Twist` | Publishes zero velocity between stages and during drive-away |
| `/bxu_battery_lift/command` | `std_msgs/String` | Publishes `LIFT` or `LOWER` |

### Services

These services are created only when `use_sim_time` is true:

| Service | Type | Purpose |
|---|---|---|
| `/bxu/supervisor/start` | `std_srvs/srv/Trigger` | Start a new demo run from `WAITING` |
| `/bxu/supervisor/stop` | `std_srvs/srv/Trigger` | Stop the demo, disable controllers, and return to `WAITING` |
| `/bxu/supervisor/confirm` | `std_srvs/srv/Trigger` | Confirm continuation after lift or lower |

## State machine

| State | Enabled controllers | Action | Next state |
|---|---|---|---|
| `WAITING` | None | Publishes zero velocity and disable commands | `FRONT_APPROACH` when auto-start is enabled or a simulation start is accepted |
| `FRONT_APPROACH` | Front approach | Waits for `DONE` or `FAULT` | `DOCKING_ALIGN` on `DONE`; `FAULT` on `FAULT` |
| `DOCKING_ALIGN` | Docking | Waits for `DOCKED` or `LOST` | `LIFTING` on `DOCKED`; `FAULT` on `LOST` |
| `LIFTING` | None | Sends `LIFT` once and waits for `LIFTED` | `WAIT_USER_AFTER_LIFT` |
| `WAIT_USER_AFTER_LIFT` | None | Waits for operator confirmation | `LOWERING` |
| `LOWERING` | None | Sends `LOWER` once and waits for `LOWERED` | `WAIT_USER_AFTER_LOWER` |
| `WAIT_USER_AFTER_LOWER` | None | Waits for operator confirmation | `DRIVE_AWAY` |
| `DRIVE_AWAY` | None | Drives backward at `drive_away_speed` for `drive_away_duration` | `DONE` |
| `DONE` | None | Publishes zero velocity and disables controllers | Terminal |
| `FAULT` | None | Publishes zero velocity and disables controllers | Terminal until restart or stop |

## Hardware and simulation behavior

On hardware, the supervisor waits for terminal input after lifting and lowering the BU.

In simulation, it does not read terminal input. Instead, it expects the following service calls:

```bash
ros2 service call /bxu/supervisor/start std_srvs/srv/Trigger
ros2 service call /bxu/supervisor/confirm std_srvs/srv/Trigger
ros2 service call /bxu/supervisor/stop std_srvs/srv/Trigger
```

The `confirm` service is valid only in `WAIT_USER_AFTER_LIFT` or `WAIT_USER_AFTER_LOWER`.

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `front_status_topic` | `/bxu/front_approach/controller_status` | Front-controller status topic |
| `dock_status_topic` | `/bxu/docking/controller_status` | Docking-controller status topic |
| `supervisor_state_topic` | `/bxu/supervisor/state` | Supervisor state topic |
| `front_enable_topic` | `/bxu/front_controller/enabled` | Front-controller enable topic |
| `dock_enable_topic` | `/bxu/docking_controller/enabled` | Docking-controller enable topic |
| `cmd_vel_topic` | `/bxu/cmd_vel` | Velocity command topic |
| `lift_command_topic` | `/bxu_battery_lift/command` | Lift command topic |
| `lift_state_topic` | `/bxu_battery_lift/state` | Lift state topic |
| `auto_start` | `true` | Start in `FRONT_APPROACH` instead of `WAITING` |
| `drive_away_speed` | `-0.15` m/s | Backward drive-away velocity |
| `drive_away_duration` | `2.0` s | Drive-away duration |

## Required integration fix

The supervisor and battery-lift node currently use different topic names:

| Signal | Supervisor topic | Battery-lift topic |
|---|---|---|
| Lift command | `/bxu_battery_lift/command` | `/bxubatteryliftcommand` |
| Lift state | `/bxu_battery_lift/state` | `/bxubatteryliftstate` |

These names must be aligned before the supervisor-driven lift sequence can operate. Use one consistent pair of topic names in both nodes and launch files.

## Related documentation

- [Motion control](motion.md)
- [Battery lift](battery_lift.md)
- [Docking vision and control](docking.md)
- [Marker-based front approach](front_marker_approach.md)
- [YOLO-based front approach](front_yolo_approach.md)