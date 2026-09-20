"""Complete autonomous run: Gazebo + Nav2 + factory + robot condition model + AI + mission.

    ros2 launch robofetch_bringup mission.launch.py    # AI drives + dashboard on localhost:8000
    ros2 launch robofetch_bringup mission.launch.py model:=ppo scenario:=high_demand
    ros2 launch robofetch_bringup mission.launch.py model:="" plan:="PICKUP:B;DELIVER"   # scripted
    ros2 launch robofetch_bringup mission.launch.py ai:=false              # no AI: fallback rules

No operator input at any point: the robot localises on the charger, asks the decision model what
to do, and works the shift by itself. Stopping is manual (./scripts/stop.sh).
"""
import os
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def workspace_python():
    """The venv interpreter: FastAPI, torch and the models live there, rclpy comes from ROS."""
    # realpath, not abspath: with --symlink-install this file is a symlink from install/ into
    # src/, and only the resolved path leads back to the workspace root (and its venv).
    here = os.path.dirname(os.path.realpath(__file__))
    ws = os.path.normpath(os.path.join(here, "..", "..", ".."))
    candidate = os.path.join(ws, "venv", "bin", "python")
    return candidate if os.path.exists(candidate) else "python3"


def generate_launch_description():
    run_id = time.strftime("run_%Y%m%d_%H%M%S")
    scenario = LaunchConfiguration("scenario")
    sim_time = {"use_sim_time": True}

    def navigation(headless):
        return GroupAction(
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("robofetch_nav"), "launch", "navigation.launch.py"])),
                launch_arguments={
                    "rviz": "false" if headless else LaunchConfiguration("rviz"),
                    "gz_extra": "-s --headless-rendering" if headless else "",
                    "planner": LaunchConfiguration("planner"),
                }.items())],
            scoped=True, forwarding=True,
            condition=(IfCondition(LaunchConfiguration("headless")) if headless
                       else UnlessCondition(LaunchConfiguration("headless"))))

    factory = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("robofetch_factory"), "launch", "factory.launch.py"])),
            launch_arguments={"scenario": scenario, "seed": LaunchConfiguration("seed"),
                              "use_sim_time": "true", "run_id": run_id}.items())],
        scoped=True, forwarding=True)

    robot_state = Node(
        package="robofetch_core", executable="robot_state_node", name="robot_state_node",
        output="screen",
        parameters=[sim_time, {"scenario": scenario, "run_id": run_id}])

    # The decision service runs as its own process on port 8001, so "the AI is down" is a state
    # you can produce by stopping it - and the robot then falls back to its built-in rules.
    decision_service = ExecuteProcess(
        cmd=[workspace_python(), "-m", "uvicorn", "robofetch_ai.service:app",
             "--host", "0.0.0.0", "--port", "8001"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("ai")))

    # The read-only dashboard, so one command gives a complete, watchable system.
    dashboard = ExecuteProcess(
        cmd=[workspace_python(), "-m", "uvicorn", "robofetch_bridge.app:app",
             "--host", "0.0.0.0", "--port", "8000"],
        additional_env={"ROBOFETCH_WEB": PathJoinSubstitution(
            [FindPackageShare("robofetch_web"), "web"]),
                        # shown in the page header, so a planner/model comparison is readable live
                        "ROBOFETCH_MODEL": LaunchConfiguration("model"),
                        "ROBOFETCH_PLANNER": LaunchConfiguration("planner")},
        output="screen",
        condition=IfCondition(LaunchConfiguration("web")))

    mission = Node(
        package="robofetch_core", executable="mission_executor", name="mission_executor",
        output="screen",
        parameters=[sim_time, {"scenario": scenario, "run_id": run_id,
                               "plan": LaunchConfiguration("plan"),
                               "model": LaunchConfiguration("model"),
                               "planner": LaunchConfiguration("planner"),
                               "shift_duration_s": ParameterValue(
                                   LaunchConfiguration("shift_s"), value_type=float)}])

    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="balanced"),
        DeclareLaunchArgument("seed", default_value="-1"),
        DeclareLaunchArgument("plan", default_value="",
                              description="scripted actions, e.g. 'PICKUP:B;DELIVER;CHARGE:90'"),
        DeclareLaunchArgument("model", default_value="ns",
                              description="decision model: ns, ns_symbolic_only, ppo, rule, or "
                                          "'' to follow `plan` instead"),
        DeclareLaunchArgument("ai", default_value="true",
                              description="start the decision service (false = fallback rules)"),
        DeclareLaunchArgument("web", default_value="true",
                              description="start the read-only dashboard on http://localhost:8000"),
        DeclareLaunchArgument("shift_s", default_value="0.0",
                              description="shift length in seconds (0 = from params.yaml)"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("planner", default_value="NavfnDijkstra",
                              description="global path planner (WP8 fastest: ThetaStar; WP9 default for "
                                          "reliability: NavfnDijkstra): NavfnDijkstra, NavfnAStar, "
                                          "Smac2D, ThetaStar, SmacLattice"),
        DeclareLaunchArgument("rviz", default_value="true"),
        navigation(False),
        navigation(True),
        TimerAction(period=3.0, actions=[decision_service, dashboard]),
        TimerAction(period=5.0, actions=[factory, robot_state]),
        TimerAction(period=8.0, actions=[mission]),
    ])
