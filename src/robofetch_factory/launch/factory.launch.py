"""The three live production sections.

    ros2 launch robofetch_factory factory.launch.py scenario:=fault_burst seed:=3
    ros2 launch robofetch_factory factory.launch.py use_sim_time:=false   # without Gazebo
"""
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_id = time.strftime("run_%Y%m%d_%H%M%S")
    sections = [
        Node(
            package="robofetch_factory",
            executable="section_node",
            name=f"section_{sid}",
            output="screen",
            parameters=[{
                "section_id": sid,
                "scenario": LaunchConfiguration("scenario"),
                "seed": LaunchConfiguration("seed"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "run_id": run_id,
            }],
        )
        for sid in ("A", "B", "C")
    ]
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="balanced"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description="-1 uses the scenario's own seed"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        *sections,
    ])
