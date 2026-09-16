"""Complete autonomous run: Gazebo + Nav2 + factory sections + robot condition model + mission.

    ros2 launch robofetch_bringup mission.launch.py plan:="PICKUP:B;DELIVER;CHARGE:95"
    ros2 launch robofetch_bringup mission.launch.py scenario:=high_demand headless:=true plan:=...

No operator input: the executor localises the robot on the charger and starts by itself.
"""
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    mission = Node(
        package="robofetch_core", executable="mission_executor", name="mission_executor",
        output="screen",
        parameters=[sim_time, {"scenario": scenario, "run_id": run_id,
                               "plan": LaunchConfiguration("plan")}])

    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="balanced"),
        DeclareLaunchArgument("seed", default_value="-1"),
        DeclareLaunchArgument("plan", default_value="",
                              description="scripted actions, e.g. 'PICKUP:B;DELIVER;CHARGE:90'"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        navigation(False),
        navigation(True),
        TimerAction(period=5.0, actions=[factory, robot_state]),
        TimerAction(period=8.0, actions=[mission]),
    ])
