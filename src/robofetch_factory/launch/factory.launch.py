"""The three live production sections (one process, per-section topics and services).

    ros2 launch robofetch_factory factory.launch.py scenario:=fault_burst seed:=3
    ros2 launch robofetch_factory factory.launch.py use_sim_time:=false   # without Gazebo
"""
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="balanced"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description="-1 uses the scenario's own seed"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("run_id", default_value=time.strftime("run_%Y%m%d_%H%M%S")),
        Node(
            package="robofetch_factory",
            executable="factory_node",
            name="factory",
            output="screen",
            parameters=[{
                "scenario": LaunchConfiguration("scenario"),
                "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "run_id": LaunchConfiguration("run_id"),
            }],
        ),
    ])
