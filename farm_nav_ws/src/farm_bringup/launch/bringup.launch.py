"""
Top-level bring-up for the autonomous farm-lane navigation task.

Brings up, in order:
  1. Gazebo with the five-lane farm world (init + factory + state plugins).
  2. robot_state_publisher (Husky + RealSense URDF) -> /robot_description + TF.
  3. robot_spawner_node           -> spawns the Husky at the BLUE marker (runtime).
  4. RTAB-Map RGB-D SLAM          -> drift-corrected map->odom (camera only).
  5. Nav2 navigation stack        -> plans/executes to the goal on the perceived
                                     obstacle costmap.
  6. Perception + mission stack   -> lane/free-space cloud, plant counting,
                                     path recording, goal sending (GREEN marker).
  7. RViz                         -> live visualization.

Usage:
  ros2 launch farm_bringup bringup.launch.py
  ros2 launch farm_bringup bringup.launch.py headless:=true use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, GroupAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("farm_bringup")
    desc_share = get_package_share_directory("farm_description")

    default_world = os.path.join(bringup_share, "worlds", "five_lane_farm.world")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    rviz_cfg = os.path.join(desc_share, "rviz", "farm.rviz")

    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    use_slam = LaunchConfiguration("use_slam")
    headless = LaunchConfiguration("headless")
    out_path = LaunchConfiguration("out_path")

    # ---- Gazebo (server always; client unless headless) ----
    gz = get_package_share_directory("gazebo_ros")
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world, "verbose": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, "launch", "gzclient.launch.py")),
        condition=UnlessCondition(headless),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    spawner = Node(
        package="farm_perception", executable="robot_spawner_node",
        name="robot_spawner_node", output="screen",
        parameters=[{"use_sim_time": True}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "slam.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(use_slam),
    )
    # fallback localization when SLAM is disabled: static identity map->odom
    static_map_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=UnlessCondition(use_slam),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"),
                         "launch", "navigation_launch.py")),
        launch_arguments={"use_sim_time": "true",
                          "params_file": nav2_params,
                          "autostart": "true"}.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "perception.launch.py")),
        launch_arguments={"out_path": out_path,
                          "video_path": LaunchConfiguration("video_path")}.items(),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg], output="log",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_slam", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("out_path", default_value="/tmp/farm_run_path.png"),
        DeclareLaunchArgument("video_path", default_value="/tmp/farm_run_overlay.mp4"),

        gzserver,
        gzclient,
        rsp,
        TimerAction(period=4.0, actions=[spawner]),
        TimerAction(period=7.0, actions=[GroupAction([slam, static_map_odom])]),
        TimerAction(period=9.0, actions=[nav2]),
        TimerAction(period=7.0, actions=[perception]),
        TimerAction(period=6.0, actions=[rviz]),
    ])
