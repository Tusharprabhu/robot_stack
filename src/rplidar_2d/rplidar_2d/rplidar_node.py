from __future__ import annotations

import math
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from rplidar import RPLidar  # type: ignore
except Exception as exc:  # pragma: no cover
    RPLidar = None  # type: ignore
    _RPLIDAR_IMPORT_ERROR = exc
else:
    _RPLIDAR_IMPORT_ERROR = None


class Rplidar2DNode(Node):
    def __init__(self) -> None:
        super().__init__("rplidar_2d")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrates", [256000, 115200])
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("topic", "scan")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("samples", 360)

        self._port = str(self.get_parameter("port").value)
        self._baudrates = [int(x) for x in self.get_parameter("baudrates").value]
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._topic = str(self.get_parameter("topic").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._samples = int(self.get_parameter("samples").value)

        self._pub = self.create_publisher(LaserScan, self._topic, 10)

        self._running = True
        self._thread: Optional[threading.Thread] = None

        if _RPLIDAR_IMPORT_ERROR is not None:
            self.get_logger().error(
                "Python 'rplidar' library is not available. Install it with `pip3 install rplidar`."
            )
            return

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"RPLidar2D node starting (port={self._port}, baudrates={self._baudrates}, topic={self._topic})"
        )

    def destroy_node(self) -> bool:
        self._running = False
        return super().destroy_node()

    def _connect(self) -> "RPLidar":
        last_exc: Optional[BaseException] = None
        for baud in self._baudrates:
            try:
                lidar = RPLidar(self._port, baudrate=baud, timeout=2)
                # Some devices need reset/clean before stable reads.
                try:
                    lidar.reset()
                    time.sleep(0.5)
                except Exception:
                    pass
                try:
                    lidar.clean_input()
                except Exception:
                    pass

                info = None
                try:
                    info = lidar.get_info()
                except Exception:
                    info = None

                if info:
                    self.get_logger().info(
                        f"Connected to RPLidar @ {baud} (model={info.get('model', '?')}, fw={info.get('firmware', '?')})"
                    )
                else:
                    self.get_logger().info(f"Connected to RPLidar @ {baud}")

                return lidar
            except Exception as exc:
                last_exc = exc
                self.get_logger().warn(f"Connect failed @ {baud}: {exc}")

        raise RuntimeError(f"Could not connect to RPLidar on {self._port}: {last_exc}")

    def _disconnect(self, lidar: Optional["RPLidar"]) -> None:
        if lidar is None:
            return
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass

    def _publish_scan(self, ranges: List[float], scan_time: float) -> None:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.range_min = float(self._range_min)
        msg.range_max = float(self._range_max)

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (2.0 * math.pi) / float(self._samples)

        # LaserScan defines time between measurements and time per scan; we only provide scan_time.
        msg.time_increment = 0.0
        msg.scan_time = float(scan_time)

        msg.ranges = ranges
        msg.intensities = []

        self._pub.publish(msg)

    def _run(self) -> None:
        angle_min = -math.pi
        angle_increment = (2.0 * math.pi) / float(self._samples)

        while self._running and rclpy.ok():
            lidar = None
            try:
                lidar = self._connect()

                ranges = [math.inf] * self._samples
                have_points = False
                scan_start = time.monotonic()

                for new_scan, quality, angle_deg, dist_mm in lidar.iter_measurements(max_buf_meas=3000):
                    if not self._running or not rclpy.ok():
                        break

                    if new_scan:
                        if have_points:
                            scan_time = max(0.0, time.monotonic() - scan_start)
                            self._publish_scan(ranges, scan_time)
                        ranges = [math.inf] * self._samples
                        have_points = False
                        scan_start = time.monotonic()

                    if dist_mm <= 0:
                        continue

                    d_m = float(dist_mm) / 1000.0
                    if d_m < self._range_min or d_m > self._range_max:
                        continue

                    rad = math.radians(float(angle_deg))
                    # Convert [0, 2pi) to [-pi, pi)
                    if rad >= math.pi:
                        rad -= 2.0 * math.pi

                    idx = int((rad - angle_min) / angle_increment)
                    if 0 <= idx < self._samples:
                        prev = ranges[idx]
                        if math.isinf(prev) or d_m < prev:
                            ranges[idx] = d_m
                            have_points = True

            except Exception as exc:
                self.get_logger().warn(f"LiDAR error: {exc}")
                time.sleep(0.5)
            finally:
                self._disconnect(lidar)


def main() -> None:
    rclpy.init()
    node = Rplidar2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
