# Battery Lift

[← `swas_system`](../README.md) · [← Repository README](../../../../README.md)

## Purpose

`bxu_battery_lift_node` controls the BXU battery lift plate through the relay Pico. It raises the plate to push a BU into the BRU, or lowers it to receive a BU released by the BRU.

Source: [`swas_system/bxu/battery_lift_node.py`](../swas_system/bxu/battery_lift_node.py)

## Summary

The node receives `LIFT` or `LOWER` commands, activates the corresponding relay for a fixed duration, and publishes its state. Lift motion is time-based rather than position-based.

## Interfaces

### Subscription

| Topic | Resolved topic | Accepted values |
|---|---|---|
| `bxubatteryliftcommand` | `/bxubatteryliftcommand` | `LIFT`, `LOWER` |

### Publisher

| Topic | Resolved topic | Values |
|---|---|---|
| `bxubatteryliftstate` | `/bxubatteryliftstate` | `IDLE`, `LIFTING`, `LIFTED`, `LOWERING`, `LOWERED` |

### Serial interface

| Parameter | Default |
|---|---|
| Port | `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_af1400e815e1417b-if00` |
| Baud rate | `115200` |

## Relay mapping

| Relay label | Pico relay | Function |
|---|---|---|
| `vacuum` | `R1` | Vacuum control |
| `bxu_down` | `R2` | BXU plate down |
| `bxu_up` | `R3` | BXU plate up |
| `battery_up` | `R4` | Battery lift plate up |
| `battery_down` | `R5` | Battery lift plate down |

The battery-lift node uses only `battery_up` and `battery_down`. The other relay labels are retained in the relay mapping.

## State behavior

| Command | Initial state | Relay action | Final state |
|---|---|---|---|
| `LIFT` | `IDLE`, `LIFTED`, or `LOWERED` | `battery_up` enabled for `battery_up_duration` | `LIFTED` |
| `LOWER` | `IDLE`, `LIFTED`, or `LOWERED` | `battery_down` enabled for `battery_down_duration` | `LOWERED` |

If a lift or lower motion is already active, new commands are ignored.

The state is republished every 0.5 seconds.

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `relay_port` | Relay Pico serial-by-id path | Serial device for the relay Pico |
| `battery_up_duration` | `1.5` s | Duration for which `battery_up` remains enabled |
| `battery_down_duration` | `5.0` s | Duration for which `battery_down` remains enabled |

## Manual use

```bash
ros2 topic pub --once /bxubatteryliftcommand std_msgs/msg/String "{data: 'LIFT'}"
ros2 topic pub --once /bxubatteryliftcommand std_msgs/msg/String "{data: 'LOWER'}"
```

## Required integration fix

The battery-lift node uses `/bxubatteryliftcommand` and `/bxubatteryliftstate`, while the supervisor uses `/bxu_battery_lift/command` and `/bxu_battery_lift/state`.

These topic names must be aligned before the supervisor can command the battery lift. Choose one naming convention and apply it consistently to the node, supervisor, launch files, and documentation.

## Related documentation

- [Supervisor](supervisor.md)
- [Hardware](../../../docs/HARDWARE.md)