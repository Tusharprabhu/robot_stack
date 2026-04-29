from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robot.yaml"]
    )
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to a YAML file with parameters for sharewave_rover and rplidar_2d",
            ),
            Node(
                package="sharewave_rover",
                executable="sharewave_serial_driver",
                name="sharewave_rover",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rplidar_2d",
                executable="rplidar_node",
                name="rplidar",
                output="screen",
                parameters=[config],
            ),
        ]
    )
