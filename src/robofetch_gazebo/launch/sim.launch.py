"""Simulation bringup.

Starts Gazebo Harmonic with the factory maze world, publishes the robot description, spawns the
robot on the charging station, and runs the ros_gz parameter bridge so ROS 2 sees /cmd_vel,
/odom, /scan, /tf, /joint_states, /clock and the ground-truth pose.

The spawn pose is read from robofetch_factory/config/poi.yaml (generated from layout.yaml), so
moving the charger in the layout moves the spawn point too.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_spawn_pose():
    config = os.path.join(get_package_share_directory("robofetch_factory"), "config", "poi.yaml")
    with open(config) as fh:
        data = yaml.safe_load(fh)
    return data["poi"][data["spawn"]]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    # gz_extra lets us append flags, e.g. "-s --headless-rendering" for a GUI-less run.
    gz_extra = LaunchConfiguration("gz_extra")
    use_rviz = LaunchConfiguration("rviz")

    pkg_gazebo = FindPackageShare("robofetch_gazebo")
    pkg_description = FindPackageShare("robofetch_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    world_path = PathJoinSubstitution([pkg_gazebo, "worlds", world])
    bridge_config = PathJoinSubstitution([pkg_gazebo, "config", "bridge.yaml"])
    rviz_config = PathJoinSubstitution([pkg_description, "rviz", "robofetch.rviz"])

    # 1) Gazebo Harmonic with the factory world. "-r" runs immediately, "-v4" is verbose.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -r -v4 ", gz_extra]}.items(),
    )

    # 2) robot_state_publisher (expands xacro -> /robot_description + link TF)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_description, "launch", "rsp.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 3) Spawn the robot from the /robot_description topic on the charging station.
    spawn_pose = load_spawn_pose()
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "robofetch",
            "-x", str(spawn_pose["x"]), "-y", str(spawn_pose["y"]), "-z", "0.1",
            "-Y", str(spawn_pose["yaw"]),
        ],
    )

    # 4) ros_gz parameter bridge (gz topics <-> ROS 2 topics)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": use_sim_time}],
    )

    # 5) RViz (optional visual check)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Gazebo's GUI is Qt Quick (QML). On WSLg its Wayland window is created but never
        # registers with the compositor, so no window ever appears even though the process
        # runs normally. Forcing the X11/XWayland platform makes it show up. RViz uses Qt
        # Widgets and is unaffected either way. Harmless on native Linux.
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value="factory_maze.sdf"),
        DeclareLaunchArgument("gz_extra", default_value="",
                              description="Extra gz args, e.g. '-s --headless-rendering'."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Set false to skip RViz (e.g. headless)."),
        gz_sim,
        rsp,
        spawn,
        bridge,
        rviz,
    ])
