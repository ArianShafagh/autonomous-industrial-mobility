"""Simulation + Nav2 (map_server, AMCL, planner, MPPI controller, behaviours) + RViz, using the
factory map generated from layout.yaml. Set the initial pose with scripts/set_pose.sh (the robot
spawns on the charger), then send goals with scripts/goto.sh.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PLANNERS = ("NavfnDijkstra", "NavfnAStar", "Smac2D", "ThetaStar", "SmacLattice")


def params_with_initial_pose(params_path, planner="NavfnDijkstra"):
    """nav2_params.yaml with AMCL's initial pose set to the robot's spawn pose (poi.yaml).

    Nav2's planner/controller cannot ACTIVATE until the map->odom transform exists, and AMCL only
    publishes it once it has an initial pose. If nobody sends one within the activation timeout
    (~60 s) the whole navigation bringup aborts - which is what made launches flaky in WP3.
    Giving AMCL the spawn pose up front removes that race. The pose is read from the generated
    poi.yaml, so moving the charger in layout.yaml moves this too.
    """
    poi_file = os.path.join(get_package_share_directory("robofetch_factory"), "config", "poi.yaml")
    with open(poi_file) as fh:
        poi = yaml.safe_load(fh)
    spawn = poi["poi"][poi["spawn"]]
    with open(params_path) as fh:
        params = yaml.safe_load(fh)
    # The behaviour tree plans with the plugin called "GridBased"; make it the chosen algorithm.
    planners = params["planner_server"]["ros__parameters"]
    if planner not in PLANNERS:
        raise RuntimeError(f"unknown planner '{planner}', choose one of {', '.join(PLANNERS)}")
    planners["GridBased"] = dict(planners[planner])
    amcl = params["amcl"]["ros__parameters"]
    amcl["set_initial_pose"] = True
    amcl["initial_pose"] = {"x": float(spawn["x"]), "y": float(spawn["y"]), "z": 0.0,
                            "yaw": float(spawn["yaw"])}
    out = tempfile.NamedTemporaryFile("w", prefix="nav2_params_", suffix=".yaml", delete=False)
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("rviz")

    pkg_nav = FindPackageShare("robofetch_nav")
    pkg_gazebo = FindPackageShare("robofetch_gazebo")
    pkg_nav2_bringup = FindPackageShare("nav2_bringup")

    default_map = PathJoinSubstitution([pkg_nav, "maps", "factory_maze.yaml"])
    default_params = PathJoinSubstitution([pkg_nav, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([pkg_nav, "rviz", "nav.rviz"])

    # 1) Simulation (Gazebo + robot + ros_gz bridge). We start RViz ourselves below with
    #    the navigation config, so the sim's own RViz is disabled.
    #    NOTE: wrapped in a scoped GroupAction. Without scoping, the launch arguments
    #    passed here leak into this file's scope, so `rviz:=false` would also switch off
    #    OUR RViz node below.
    sim = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_gazebo, "launch", "sim.launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time, "rviz": "false"}.items(),
            )
        ],
        scoped=True,
        forwarding=True,
    )

    # 2) Nav2 stack (localization + navigation), using the stock bringup with our params plus
    #    AMCL's initial pose (see params_with_initial_pose).
    def nav2(context):
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2_bringup, "launch", "bringup_launch.py"])
            ),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": params_with_initial_pose(params_file.perform(context),
                                                        LaunchConfiguration("planner").perform(context)),
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        )]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("planner", default_value="NavfnDijkstra",
                              description="global planner used when driving: " + ", ".join(PLANNERS)),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Set false to run navigation headless."),
        sim,
        OpaqueFunction(function=nav2),
        rviz,
    ])
