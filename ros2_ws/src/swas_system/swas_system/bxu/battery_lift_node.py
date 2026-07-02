import time
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RelayController:
    """
    Minimal copy of your existing RelayController, used ONLY
    for controlling the battery lift via the relay Pico.

    Relay mapping (same as mecanum_controller.py):

      vacuum       -> R1
      bxu_down     -> R2
      bxu_up       -> R3
      battery_up   -> R4  (battery plate up)
      battery_down -> R5  (battery plate down)
    """

    def __init__(self, port: str):
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(1.0)  # give Pico time to reset
            print(f"[bxu_battery_liftnode] ✅ Relay Pico connected on {port}")
        except Exception as e:
            print(f"[bxu_battery_liftnode] ⚠️ Relay Pico not found on {port}: {e}")
            self.ser = None

        self._mapping = {
            "vacuum": "R1",
            "bxu_down": "R2",
            "bxu_up": "R3",
            "battery_up": "R4",
            "battery_down": "R5",
        }

    def set_relay(self, label: str, state: bool):
        if not self.ser:
            return
        if label not in self._mapping:
            print(f"[bxu_battery_liftnode] ⚠️ Unknown relay label: {label}")
            return
        cmd = f"{self._mapping[label]} {1 if state else 0}\n"
        try:
            self.ser.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[bxu_battery_liftnode] ⚠️ Relay serial error: {e}")
            self.ser = None

    def timed_relay(self, label: str, duration: float):
        """Turn relay on for `duration` seconds, then off."""
        self.set_relay(label, True)
        time.sleep(duration)
        self.set_relay(label, False)


class BxuBatteryLiftNode(Node):
    """
    bxu_battery_liftnode: ROS 2 wrapper for the BXU BATTERY plate lift.
    Uses relay Pico R4/R5 to move the plate up/down.

    Subscribes:
      - /bxubatteryliftcommand (std_msgs/String): "LIFT" or "LOWER"

    Publishes:
      - /bxubatteryliftstate   (std_msgs/String):
          "IDLE", "LIFTING", "LIFTED", "LOWERING", "LOWERED"
    """

    def __init__(self):
        super().__init__("bxu_battery_lift_node")

        # Parameters (later move to config/bxuparams.yaml)
        self.declare_parameter(
            "relay_port",
            "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_af1400e815e1417b-if00",
        )
        # Durations copied from mecanum_controller CLI: 1.5s up, 5s down
        self.declare_parameter("battery_up_duration", 1.5)
        self.declare_parameter("battery_down_duration", 5.0)

        relay_port = self.get_parameter("relay_port").value
        self._dur_up = float(self.get_parameter("battery_up_duration").value)
        self._dur_down = float(self.get_parameter("battery_down_duration").value)

        # Hardware interface
        self._relays = RelayController(relay_port)

        # Internal state
        self._state = "IDLE"
        self._state_lock = threading.Lock()
        self._motion_thread = None

        # ROS interfaces
        self._cmd_sub = self.create_subscription(
            String, "bxubatteryliftcommand", self._cmd_callback, 10
        )
        self._state_pub = self.create_publisher(String, "bxubatteryliftstate", 10)

        # Periodic state publisher
        self._state_timer = self.create_timer(0.5, self._publish_state)

        self.get_logger().info("bxu_battery_lift_node initialized")

    # --- ROS callbacks ------------------------------------------------------

    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"Received battery lift command: {cmd}")

        if cmd == "LIFT":
            self._start_motion(direction="UP")
        elif cmd == "LOWER":
            self._start_motion(direction="DOWN")
        else:
            self.get_logger().warn(f"Unknown battery lift command: {cmd}")

    def _publish_state(self):
        with self._state_lock:
            s = self._state
        out = String()
        out.data = s
        self._state_pub.publish(out)

    # --- Motion control -----------------------------------------------------

    def _set_state(self, new_state: str):
        with self._state_lock:
            if new_state != self._state:
                self.get_logger().info(
                    f"Battery lift state: {self._state} -> {new_state}"
                )
            self._state = new_state

    def _start_motion(self, direction: str):
        # Avoid overlapping motions
        with self._state_lock:
            if self._state in ("LIFTING", "LOWERING"):
                self.get_logger().warn(
                    "Battery lift already moving; ignoring new command"
                )
                return

        # Fire a new thread for the timed relay so we don't block ROS executor
        self._motion_thread = threading.Thread(
            target=self._do_motion, args=(direction,), daemon=True
        )
        self._motion_thread.start()

    def _do_motion(self, direction: str):
        if direction == "UP":
            self._set_state("LIFTING")
            # battery_up -> R4
            self._relays.timed_relay("battery_up", self._dur_up)
            self._set_state("LIFTED")
        else:
            self._set_state("LOWERING")
            # battery_down -> R5
            self._relays.timed_relay("battery_down", self._dur_down)
            self._set_state("LOWERED")

    # --- Shutdown -----------------------------------------------------------

    def destroy_node(self):
        # No special cleanup besides default
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BxuBatteryLiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
