# Hardware

[← Repository README](../README.md) · [System architecture](SYSTEM_ARCHITECTURE.md)

This document describes the physical SWAS hardware, its main interfaces, and the boundary between ROS 2 software and embedded control hardware.

## System components

| Component | Description |
|---|---|
| **BU** | Removable EV battery pack transferred by the system |
| **BRU** | Stationary Battery Receiving Unit, retrofitted from a Tesla battery pack |
| **BXU** | Autonomous mecanum-drive Battery Exchange Unit |
| **BXU Raspberry Pi 5** | Main BXU compute platform |
| **BRU Raspberry Pi 5** | Main BRU compute platform |
| **Operator laptop** | Runs the HMI and provides operator access |
| **Motion Pico** | Real-time motion controller for the BXU drive system |
| **Relay Pico** | Controls lift, vacuum, and other relay-driven functions |
| **HB860H drivers** | Closed-loop stepper drivers for the BXU drive motors |
| **NEMA 34 motors** | High-torque drive motors |
| **Encoders / ALM signals** | Provide position-following and fault feedback |
| **Lift plate** | Raises or lowers the BU between BXU and BRU |
| **BXU locking mechanism** | Moves and secures the BU on the BXU; patented mechanical design |
| **BRU locking mechanism** | Secures or releases the BU in the BRU |
| **Cameras** | Provide perception input for approach and docking |

## Compute and network architecture

```text
+------------------+        +----------------------+
| Operator laptop  |        |   BRU Raspberry Pi 5 |
|    swas_hmi      |        |   BRU-side control   |
+--------+---------+        +----------+-----------+
         |                             |
         | local Wi-Fi network         |
         v                             |
+----------------------+               |
|  BXU Raspberry Pi 5  |<--------------+
|      ROS 2 system    |
+----------+-----------+
           |
           +--> Motion Pico
           |       |
           |       +--> 4x HB860H --> 4x NEMA 34
           |       +--> ALM feedback
           |
           +--> Relay Pico
                   |
                   +--> Lift / vacuum / actuation relays
```

The operator laptop, BXU Raspberry Pi 5, and BRU Raspberry Pi 5 are on the same trusted local Wi-Fi network. The HMI connects to rosbridge on port 9090 and `web_video_server` on port 8080.

TODO: Add device hostnames or IP addresses, SSH access procedure, ROS domain ID, DDS configuration, firewall requirements, and Wi-Fi client-isolation constraints.

## BXU drive system

The BXU uses a four-wheel mecanum drive with closed-loop stepper drivers.

### Motion controller

The motion Pico receives high-level velocity commands from the BXU Raspberry Pi 5 and generates the required step, direction, enable, and timing signals for the HB860H drivers.

The ROS 2 motion node uses:

| Item | Value |
|---|---|
| Serial device | `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_31c5dd4d04cc2c72-if00` |
| Baud rate | 115200 |
| Command format | `V <vx> <vy> <omega>` |
| Zero/heartbeat command | `V 0 0 0` |
| Heartbeat interval | 0.2 s |
| Maximum `vx` | 0.30 m/s |
| Maximum `vy` | 0.30 m/s |
| Maximum yaw rate | 0.80 rad/s |

The velocity convention is:

| Field | Meaning |
|---|---|
| `linear.x` | BXU forward/backward velocity |
| `linear.y` | BXU lateral velocity |
| `angular.z` | BXU yaw rate |

When the commanded velocity is zero, the motion node periodically sends the zero command as a heartbeat. On shutdown, the heartbeat stops and the motors are expected to relax through the Pico watchdog.

### Motor mapping

The following mapping comes from the earlier custom control implementation and must be verified against the current hardware before being treated as authoritative.

| Motor | Position | STEP | DIR | ENA | ALM input | Inverted |
|---|---|---:|---:|---:|---:|---|
| 1 | Front left | 2 | 3 | 4 | 22 | Yes |
| 2 | Front right | 6 | 7 | 8 | 15 | No |
| 3 | Rear left | 10 | 11 | 12 | 13 | Yes |
| 4 | Rear right | 18 | 19 | 20 | 16 | No |

TODO: Confirm driver DIP-switch settings, steps per revolution, wheel radius, gearing, maximum safe acceleration, and current motor mapping.

## Actuation and relays

The relay Pico controls the lift and auxiliary functions.

The following mapping comes from the earlier custom control implementation and must be verified against the current hardware.

| GPIO | Relay | Function |
|---:|---|---|
| 2 | R1 | Vacuum |
| 4 | R2 | BXU plate down |
| 7 | R3 | BXU plate up |
| 9 | R4 | Battery lift plate up |
| 10 | R5 | Battery lift plate down |

The current ROS battery-lift node uses `battery_up` (`R4`) and `battery_down` (`R5`). Lift motion is time-based: 1.5 s up and 5.0 s down by default.

TODO: Confirm relay-module ratings, power supply, wiring, mechanical limits, and whether vacuum and BXU-plate relays are still used.

## Cameras and calibration

Cameras provide input for front approach, marker detection, and docking alignment.

| Camera | Resolution | Purpose | Calibration file |
|---|---|---|---|
| Front camera | 1280 × 720 | Marker-based or YOLO-based front approach | `front_cam_1280x720_calib.npz` |
| Docking camera 0 | 640 × 480 | Docking marker 0 | `camera_0_640x480_calibration.npz` |
| Docking camera 1 | 640 × 480 | Docking marker 1 | `camera_1_640x480_calibration.npz` |

Calibration files are in [`swas_system/calib`](../ros2_ws/src/swas_system/calib).

`alignment_zero.npz` defines the pixel offsets at which the BRU docking markers appear in the BXU upward-camera images when the BXU and BRU are aligned. It is required for the docking phase.

TODO: Document camera models, mounting positions, orientations, topic names, recalibration procedure, and the final docking-camera geometry.

## Safety and fault handling

The BXU combines high-torque drive motors, lifting hardware, and relay-controlled actuation. Hardware faults must stop motion and require deliberate recovery.

Known fault-related behavior includes:

- HB860H driver alarms indicate a position-following or stall-related fault.
- The motion Pico can report alarm IDs to the host.
- Driver enable pins can be toggled to clear certain driver faults without a full power cycle.
- Shielded alarm wiring reduces false alarms caused by motor EMI.

The HMI stop function requests supervisor stop, controller disable, zero velocity, and command-gate stop. It does not cancel lift actuation and is not an emergency stop.

TODO: Document the current emergency-stop path, conditions that halt motion immediately, lift interlocks, alarm recovery, watchdog behavior, and safe power-up/power-down sequence.

## Hardware–software boundary

| Layer | Responsibility |
|---|---|
| ROS 2 nodes | Mission logic, perception, alignment decisions, high-level motion and lift commands |
| Raspberry Pi 5 | ROS 2 runtime, camera acquisition, serial communication, HMI connectivity |
| Motion Pico | Step generation, mecanum kinematics, timing, alarm monitoring |
| Relay Pico | Relay actuation for lift and auxiliary functions |
| HB860H drivers | Motor current control, closed-loop position following, alarm generation |
| BRU controller | BRU locking and release control |

TODO: Add the exact motion- and relay-Pico firmware protocol, including all command and feedback formats.