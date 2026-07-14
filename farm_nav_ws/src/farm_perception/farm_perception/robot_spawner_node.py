#!/usr/bin/env python3
"""
robot_spawner_node
==================

Spawns the Husky at the BLUE start marker, read from the environment at runtime
(via /gazebo/get_entity_state) rather than hardcoded coordinates. This satisfies
the brief's requirement that the start position be read from the environment.

Sequence:
  1. Wait for the robot URDF on the latched /robot_description topic.
  2. Query Gazebo for the blue-marker model pose.
  3. Spawn the Husky there, lifted to wheel height and given `spawn_yaw`
     (default: facing the field / goal so the front camera sees the lanes).

The map frame used by SLAM/Nav2 is anchored at this spawn pose, so the start is
implicitly the navigation origin -- never written as a literal in the nav logic.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetEntityState, SpawnEntity


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RobotSpawnerNode(Node):
    def __init__(self):
        super().__init__("robot_spawner_node")
        # The marker's pose is defined on its link, so we must query the link
        # ("model::link"); the bare model pose is (0,0,0) in this world.
        self.declare_parameter("start_marker",
                               "husky_footprint_start_reference::footprint_link")
        self.declare_parameter("robot_name", "husky")
        self.declare_parameter("spawn_yaw", math.pi)   # face -x toward the goal
        self.declare_parameter("spawn_z", 0.2)

        self.start_marker = self.get_parameter("start_marker").value
        self.robot_name = self.get_parameter("robot_name").value
        self.spawn_yaw = float(self.get_parameter("spawn_yaw").value)
        self.spawn_z = float(self.get_parameter("spawn_z").value)

        self.urdf = None
        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(String, "/robot_description", self._urdf_cb, qos)

        self.get_state = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self.done = False
        self.timer = self.create_timer(1.0, self._tick)

    def _urdf_cb(self, msg):
        self.urdf = msg.data

    def _tick(self):
        if self.done:
            return
        if self.urdf is None:
            self.get_logger().info("waiting for /robot_description ...", once=True)
            return
        if not self.get_state.service_is_ready() or not self.spawn.service_is_ready():
            self.get_logger().info("waiting for Gazebo services ...", once=True)
            return

        # read the blue start marker pose at runtime
        req = GetEntityState.Request()
        req.name = self.start_marker
        req.reference_frame = "world"
        fut = self.get_state.call_async(req)
        fut.add_done_callback(self._on_start_pose)
        self.done = True            # only attempt once

    def _on_start_pose(self, fut):
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().error(
                f"could not read start marker '{self.start_marker}'; retrying")
            self.done = False
            return
        p = res.state.pose.position
        self.get_logger().info(
            f"start marker at world ({p.x:.2f}, {p.y:.2f}); spawning Husky there.")

        pose = Pose()
        pose.position.x = p.x
        pose.position.y = p.y
        pose.position.z = self.spawn_z
        qx, qy, qz, qw = yaw_to_quat(self.spawn_yaw)
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw

        req = SpawnEntity.Request()
        req.name = self.robot_name
        req.xml = self.urdf
        req.initial_pose = pose
        req.reference_frame = "world"
        self.spawn.call_async(req).add_done_callback(self._on_spawn)

    def _on_spawn(self, fut):
        res = fut.result()
        if res is not None and res.success:
            self.get_logger().info("Husky spawned at start marker.")
        else:
            self.get_logger().error(f"spawn failed: {getattr(res, 'status_message', '?')}")


def main():
    rclpy.init()
    node = RobotSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
