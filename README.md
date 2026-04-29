# robot_stack (ROS 2 Humble)

This repo contains a ROS 2 Humble `colcon` workspace for a Sharewave rover (serial ESP32 JSON protocol) and an RPLidar 2D.

## Packages

- `sharewave_rover`: subscribes `cmd_vel` and sends serial JSON to the rover.
- `rplidar_2d`: publishes `sensor_msgs/LaserScan` on `scan` using the Python `rplidar` library.
- `robot_bringup`: launch + config.

## Install (Ubuntu / Raspberry Pi)

Assuming ROS 2 Humble is installed and sourced.

```bash
sudo apt update
sudo apt install -y python3-pip python3-serial
pip3 install -r requirements.txt
```

## Build

From repo root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Edit the device ports in `src/robot_bringup/config/robot.yaml` if needed (e.g. `/dev/ttyUSB0`, `/dev/serial0`).

```bash
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py config:=$(pwd)/src/robot_bringup/config/robot.yaml
```

### Quick test

Send a command (in another terminal):

```bash
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

View LiDAR:

```bash
source install/setup.bash
ros2 topic echo /scan --once
```

## Notes

- The rover protocol supports two formats used in your existing scripts:
  - `T13` (default): `{T:13, X:<linear>, Z:<angular>}`
  - `T1`: `{T:1, L:<left>, R:<right>}` (set `protocol: T1` in YAML)
- If you prefer the upstream ROS driver for Slamtec RPLidar, you can swap `rplidar_2d` out later; this repo keeps everything Python-only for fast setup.
