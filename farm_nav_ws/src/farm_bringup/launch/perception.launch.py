"""Perception + mission stack: lane/free-space, plant counting, path recording, goal."""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}
    out_path = LaunchConfiguration("out_path")
    video_path = LaunchConfiguration("video_path")

    return LaunchDescription([
        DeclareLaunchArgument("out_path", default_value="/tmp/farm_run_path.png"),
        DeclareLaunchArgument("video_path", default_value="/tmp/farm_run_overlay.mp4"),

        Node(package="farm_perception", executable="lane_perception_node",
             name="lane_perception_node", output="screen",
             parameters=[use_sim_time, {"global_frame": "map"}]),

        Node(package="farm_perception", executable="plant_counter_node",
             name="plant_counter_node", output="screen",
             parameters=[use_sim_time]),

        Node(package="farm_perception", executable="path_recorder_node",
             name="path_recorder_node", output="screen",
             parameters=[use_sim_time, {"out_path": out_path}]),

        Node(package="farm_perception", executable="goal_sender_node",
             name="goal_sender_node", output="screen",
             parameters=[use_sim_time, {"spawn_yaw": math.pi, "start_delay": 14.0}]),

        Node(package="farm_perception", executable="video_recorder_node",
             name="video_recorder_node", output="screen",
             parameters=[use_sim_time, {"out_path": video_path}]),
    ])
