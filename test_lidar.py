from rplidar import RPLidar
import time

LIDAR_PORT = "/dev/ttyUSB0"


def min_distance(scan, angle_ranges):
    vals = []
    for quality, angle, distance_mm in scan:
        if distance_mm <= 0:
            continue
        for a1, a2 in angle_ranges:
            if a1 <= a2:
                match = a1 <= angle <= a2
            else:
                match = angle >= a1 or angle <= a2
            if match:
                vals.append(distance_mm / 1000.0)
                break
    return min(vals) if vals else None


def main():
    lidar = RPLidar(LIDAR_PORT)

    try:
        print("LIDAR info:", lidar.get_info())
        print("LIDAR health:", lidar.get_health())

        for scan in lidar.iter_scans():
            front = min_distance(scan, [(330, 360), (0, 30)])
            left = min_distance(scan, [(30, 90)])
            right = min_distance(scan, [(270, 330)])

            print(
                f"Front: {front} m | Left: {left} m | Right: {right} m"
            )
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()


if __name__ == "__main__":
    main()
