"""Publish the Husky+RealSense URDF to /robot_description and TF (static joints)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_path = PathJoinSubstitution(
        [FindPackageShare("farm_description"), "urdf", "husky.urdf.xacro"]
    )
    # Wrap in ParameterValue(value_type=str) so launch does not try to parse the
    # xacro/URDF output as YAML (required on ROS 2 Humble).
    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),

        # Wheel joints are driven by Gazebo; publish zero joint states so the
        # TF tree is complete even before /joint_states arrives.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
