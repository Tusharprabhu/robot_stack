from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import serial  # type: ignore
except Exception as exc:  # pragma: no cover
    serial = None  # type: ignore
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SharewaveSerialDriver(Node):
    def __init__(self) -> None:
        super().__init__("sharewave_serial_driver")

        self.declare_parameter("port", "/dev/serial0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("protocol", "T13")  # T13: {T:13,X,Z} ; T1: {T:1,L,R}
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 1.5)
        self.declare_parameter("wheel_separation", 0.30)  # only used for T1
        self.declare_parameter("wheel_speed_scale", 1.0)  # only used for T1

        self._port = str(self.get_parameter("port").value)
        self._baud = int(self.get_parameter("baud").value)
        self._protocol = str(self.get_parameter("protocol").value).upper()
        self._cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)

        self._max_linear = float(self.get_parameter("max_linear").value)
        self._max_angular = float(self.get_parameter("max_angular").value)

        self._wheel_separation = float(self.get_parameter("wheel_separation").value)
        self._wheel_speed_scale = float(self.get_parameter("wheel_speed_scale").value)

        self._ser_lock = threading.Lock()
        self._ser: Optional["serial.Serial"] = None

        self._last_cmd_time = 0.0
        self._last_twist = Twist()

        topic = str(self.get_parameter("cmd_vel_topic").value)
        self._sub = self.create_subscription(Twist, topic, self._on_cmd_vel, 10)

        # Watchdog timer: enforce stop when commands stop arriving.
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_tick)

        # Try to connect in background to avoid blocking constructor.
        self._connect_timer = self.create_timer(0.5, self._ensure_connected)

        self.get_logger().info(
            f"Sharewave rover driver starting (port={self._port}, baud={self._baud}, protocol={self._protocol})"
        )

        if _SERIAL_IMPORT_ERROR is not None:
            self.get_logger().error(
                "pyserial is not available. Install it (e.g. `sudo apt install python3-serial` or `pip install pyserial`)."
            )

    def destroy_node(self) -> bool:
        try:
            self._send_stop(best_effort=True)
        except Exception:
            pass
        self._close_serial()
        return super().destroy_node()

    def _ensure_connected(self) -> None:
        if _SERIAL_IMPORT_ERROR is not None:
            return
        if self._ser is not None:
            return
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.2, write_timeout=0.2)
            time.sleep(0.2)
            with self._ser_lock:
                self._ser = ser
            self.get_logger().info("Serial connected")
            self._send_stop(best_effort=True)
        except Exception as exc:
            self.get_logger().warn(f"Serial connect failed: {exc}")
            self._close_serial()

    def _close_serial(self) -> None:
        with self._ser_lock:
            ser = self._ser
            self._ser = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_time = time.monotonic()
        self._last_twist = msg

        x = _clamp(float(msg.linear.x), -self._max_linear, self._max_linear)
        z = _clamp(float(msg.angular.z), -self._max_angular, self._max_angular)

        try:
            if self._protocol == "T1":
                left, right = self._twist_to_lr(x, z)
                self._send_t1(left, right)
            else:
                self._send_t13(x, z)
        except Exception as exc:
            self.get_logger().warn(f"Send failed: {exc}")
            self._close_serial()

    def _watchdog_tick(self) -> None:
        if self._cmd_timeout_sec <= 0:
            return
        if self._ser is None:
            return
        if self._last_cmd_time == 0.0:
            return

        age = time.monotonic() - self._last_cmd_time
        if age > self._cmd_timeout_sec:
            self._send_stop(best_effort=True)

    def _twist_to_lr(self, x: float, z: float) -> tuple[float, float]:
        # Differential drive approx.
        # v_l = x - z * (W/2)
        # v_r = x + z * (W/2)
        half_w = self._wheel_separation / 2.0
        left = (x - z * half_w) * self._wheel_speed_scale
        right = (x + z * half_w) * self._wheel_speed_scale

        # Clamp to [-1, 1] since your existing scripts use that convention.
        left = _clamp(left, -1.0, 1.0)
        right = _clamp(right, -1.0, 1.0)
        return left, right

    def _send_stop(self, best_effort: bool = False) -> None:
        try:
            if self._protocol == "T1":
                self._send_t1(0.0, 0.0)
            else:
                self._send_t13(0.0, 0.0)
        except Exception:
            if not best_effort:
                raise

    def _send_t13(self, x: float, z: float) -> None:
        ser = self._ser
        if ser is None:
            return

        payload = {"T": 13, "X": round(float(x), 2), "Z": round(float(z), 2)}
        line = json.dumps(payload, separators=(",", ":")) + "\n"

        with self._ser_lock:
            ser.write(line.encode("utf-8"))

    def _send_t1(self, left: float, right: float) -> None:
        ser = self._ser
        if ser is None:
            return

        payload = {"T": 1, "L": round(float(left), 3), "R": round(float(right), 3)}
        line = json.dumps(payload, separators=(",", ":")) + "\n"

        with self._ser_lock:
            ser.write(line.encode("utf-8"))


def main() -> None:
    rclpy.init()
    node = SharewaveSerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
