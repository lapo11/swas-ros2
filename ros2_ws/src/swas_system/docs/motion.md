# Motion Control

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

## Purpose

`bxu_motion_node` converts ROS 2 velocity commands into serial commands for the BXU motion Pico. The Pico is responsible for mecanum kinematics, stepper-pulse generation, and motor-driver timing.

Source: [`swas_system/bxu/motion_node.py`](../swas_system/bxu/motion_node.py)

## Summary

The node subscribes to `/bxu/cmd_vel`, applies optional inversion, velocity limits, and minimum nonzero-command thresholds, then sends a `V vx vy omega` command to the motion Pico.

It does not perform docking control, path planning, or localization. It is a hardware abstraction layer between ROS 2 controllers and the motion Pico.

## Interfaces

### Subscription

| Topic | Type | Description |
|---|---|---|
| `/bxu/cmd_vel` | `geometry_msgs/Twist` | Desired BXU velocity |

### Publisher

| Topic | Type | Description |
|---|---|---|
| `/bxu/motion/status` | `std_msgs/String` | Connection, command, and error status |

### Serial interface

| Parameter | Default |
|---|---|
| Port | `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_31c5dd4d04cc2c72-if00` |
| Baud rate | `115200` |
| Command format | `V <vx> <vy> <omega>` |
| Decimal places | 3 |

Example serialized command:

```text
V 0.050 -0.020 0.100
```

## Command processing

For each incoming `Twist`:

1. Apply `invert_vx`, `invert_vy`, and `invert_omega` if enabled.
2. Clamp `vx`, `vy`, and `omega` to their configured maximum values.
3. Apply configured minimum nonzero magnitudes.
4. If all components are zero, send the zero command and enter heartbeat mode.
5. Otherwise, format and transmit the velocity command.

The velocity convention is:

| Field | Meaning |
|---|---|
| `linear.x` | BXU forward/backward velocity |
| `linear.y` | BXU lateral velocity |
| `angular.z` | BXU yaw rate |

## Heartbeat and safety

When the latest command is zero, the node periodically sends the configured heartbeat command:

```text
V 0 0 0
```

This keeps the motion Pico informed that the ROS 2 node is alive. On node shutdown, the heartbeat stops and the motors are expected to relax through the Pico watchdog.

Serial-write failures are reported on `/bxu/motion/status` using:

```text
ERROR:WRITE_FAILED:<exception>
```

A failed serial-port open terminates the node and publishes:

```text
ERROR:OPEN_FAILED:<exception>
```

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `move_port` | Motion Pico serial-by-id path | Serial device for the motion Pico |
| `baud_rate` | `115200` | Serial baud rate |
| `serial_timeout_s` | `0.05` | Serial read timeout |
| `startup_delay_s` | `2.0` | Delay after opening the serial port |
| `cmd_vel_topic` | `/bxu/cmd_vel` | Input velocity topic |
| `status_topic` | `/bxu/motion/status` | Status topic |
| `heartbeat_interval_s` | `0.2` | Heartbeat period |
| `enable_heartbeat` | `true` | Enable zero-velocity heartbeat |
| `heartbeat_command` | `V 0 0 0` | Command sent while holding |
| `zero_command` | `V 0 0 0` | Command sent for zero velocity |
| `max_vx` | `0.30` m/s | Forward/backward velocity limit |
| `max_vy` | `0.30` m/s | Lateral velocity limit |
| `max_omega` | `0.80` rad/s | Yaw-rate limit |
| `min_nonzero_vx` | `0.0` m/s | Minimum nonzero forward/backward command |
| `min_nonzero_vy` | `0.0` m/s | Minimum nonzero lateral command |
| `min_nonzero_omega` | `0.0` rad/s | Minimum nonzero yaw command |
| `invert_vx` | `false` | Invert forward/backward direction |
| `invert_vy` | `false` | Invert lateral direction |
| `invert_omega` | `false` | Invert yaw direction |
| `command_prefix` | `V` | Serial command prefix |
| `command_decimal_places` | `3` | Decimal precision in serial command |
| `flush_each_write` | `true` | Flush serial output after every command |
| `log_commands` | `false` | Log every transmitted command |
| `publish_status` | `true` | Publish motion status |

## Related documentation

- [Supervisor](supervisor.md)
- [Hardware](../../../docs/HARDWARE.md)