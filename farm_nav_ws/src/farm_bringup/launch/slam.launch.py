"""
RTAB-Map RGB-D SLAM using ONLY the RealSense camera.

Wheel odometry (odom->base_footprint TF from the diff-drive plugin) is used as
the motion prior; RTAB-Map refines it with visual registration + loop closure
on the RGB-D stream and publishes the drift-correcting map->odom transform plus
a consistent `map` frame for Nav2.

Constrained to 2D (Force3DoF / Slam2D) because the Husky drives on a planar
farm, which makes the estimate far more robust in this low-texture scene.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    rtabmap_params = {
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",          # use wheel-odom TF as the prior
        "map_frame_id": "map",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_scan": False,
        "approx_sync": True,
        "queue_size": 30,
        "use_sim_time": True,
        # planar farm -> constrain to 2D for robustness
        "Reg/Force3DoF": "true",
        "Optimizer/Slam2D": "true",
        "Reg/Strategy": "0",              # visual registration
        "Vis/MinInliers": "10",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "Grid/FromDepth": "false",        # Nav2 costmap comes from our own cloud
        "Mem/IncrementalMemory": "true",
    }

    remaps = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_rect_raw"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[rtabmap_params],
            remappings=remaps,
            arguments=["-d"],            # reset the database each run
        ),
    ])
