#!/usr/bin/env python3
"""
goal_sender_node
================

Reads the BLUE (start) and GREEN (goal) marker poses from the environment at
runtime (/gazebo/get_entity_state) and commands Nav2 to drive to the goal.

Frame handling
--------------
The goal marker is read in Gazebo *world* coordinates, but Nav2 navigates in the
`map` frame, whose origin depends on the localization source (RTAB-Map anchors
it at the SLAM start; the static fallback leaves it world-aligned). Rather than
assume a particular anchoring, we recover the world->map transform at runtime
from a single fact we already know live: where the robot is *now*.

The robot spawned at the blue marker (read live) with the configured spawn yaw,
so its world pose is known; its `map` pose is read from TF (`map->base`). The
two describe the same body, which fixes world->map and lets us map any world
point into `map`:

    p_body = R(-yaw_spawn) * (p_world - t_start)        # goal in spawn body frame
    p_map  = t_robot_map  + R(yaw_robot_map) * p_body   # body frame -> map

Both marker positions are read live; only the spawn *orientation* (a known
configuration value, not a position) is supplied as a parameter. No start/goal
coordinate is hardcoded in the navigation logic.

The node waits for the Nav2 `navigate_to_pose` action, sends the goal, and
publishes a human-readable status on /perception/nav_status for the overlay.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetEntityState
from nav2_msgs.action import NavigateToPose

import tf2_ros


class GoalSenderNode(Node):
    def __init__(self):
        super().__init__("goal_sender_node")
        # Marker poses live on the link ("model::link"); the bare model pose is
        # (0,0,0) in this world, so query the link to read the true positions.
        self.declare_parameter("start_marker",
                               "husky_footprint_start_reference::footprint_link")
        self.declare_parameter("goal_marker",
                               "husky_footprint_end_reference::footprint_link")
        self.declare_parameter("spawn_yaw", math.pi)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("start_delay", 12.0)   # let SLAM/Nav2 come up

        self.start_marker = self.get_parameter("start_marker").value
        self.goal_marker = self.get_parameter("goal_marker").value
        self.spawn_yaw = float(self.get_parameter("spawn_yaw").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.status = "WAITING"
        self.start_pose = None
        self.goal_world = None
        self.sent = False

        self.status_pub = self.create_publisher(String, "/perception/nav_status", 10)
        self.create_timer(0.5, lambda: self.status_pub.publish(String(data=self.status)))

        self.get_state = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        delay = float(self.get_parameter("start_delay").value)
        self.create_timer(delay, self._begin)
        self._begun = False

    # ----------------------------------------------------------------------
    def _begin(self):
        if self._begun:
            return
        self._begun = True
        if not self.get_state.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/gazebo/get_entity_state unavailable")
            self.status = "NO_GAZEBO"
            return
        self._read_marker(self.start_marker, self._got_start)

    def _read_marker(self, name, cb):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = "world"
        self.get_state.call_async(req).add_done_callback(cb)

    def _got_start(self, fut):
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().error(f"cannot read start marker '{self.start_marker}'")
            self.status = "NO_START"
            return
        self.start_pose = res.state.pose.position
        self._read_marker(self.goal_marker, self._got_goal)

    def _got_goal(self, fut):
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().error(f"cannot read goal marker '{self.goal_marker}'")
            self.status = "NO_GOAL"
            return
        self.goal_world = res.state.pose.position
        self._send_goal()

    # ----------------------------------------------------------------------
    def _send_goal(self):
        if self.sent:
            return
        # Both Nav2 (lifecycle activation, esp. under load) and the map->base TF
        # may not be ready immediately, so poll with a retry timer rather than
        # giving up after a single fixed wait.
        self._wait_logged = False
        self._dispatch_timer = self.create_timer(2.0, self._try_dispatch)

    def _try_dispatch(self):
        if self.sent:
            return
        if not self.nav.server_is_ready():
            if not self._wait_logged:
                self.get_logger().info(
                    "waiting for Nav2 navigate_to_pose action to come up ...")
                self._wait_logged = True
            self.status = "WAIT_NAV2"
            return

        # Recover the robot's current pose in the map frame; this fixes the
        # world->map transform regardless of how `map` happens to be anchored.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:                       # noqa: BLE001
            self.get_logger().info(
                f"waiting for {self.map_frame}->{self.base_frame} TF ... ({e})",
                throttle_duration_sec=3.0)
            self.status = "WAIT_TF"
            return

        mx = tf.transform.translation.x
        my = tf.transform.translation.y
        q = tf.transform.rotation
        m_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # goal expressed in the robot's spawn body frame (start marker world
        # pose + spawn yaw, both known live)...
        dx = self.goal_world.x - self.start_pose.x
        dy = self.goal_world.y - self.start_pose.y
        cs, ss = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
        bx = cs * dx + ss * dy
        by = -ss * dx + cs * dy
        # ...then mapped into `map` via the robot's actual map pose.
        cm, sm = math.cos(m_yaw), math.sin(m_yaw)
        gx = mx + cm * bx - sm * by
        gy = my + sm * bx + cm * by

        self.get_logger().info(
            f"goal world=({self.goal_world.x:.2f},{self.goal_world.y:.2f}) "
            f"robot_map=({mx:.2f},{my:.2f},{m_yaw:.2f}) "
            f"-> {self.map_frame}=({gx:.2f},{gy:.2f})")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.w = 1.0

        self.sent = True
        self.status = "NAVIGATING"
        self.nav.send_goal_async(goal).add_done_callback(self._goal_response)
        self._dispatch_timer.cancel()

    def _goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.status = "REJECTED"
            self.get_logger().error("goal rejected by Nav2")
            return
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, fut):
        status = fut.result().status
        # 4 == STATUS_SUCCEEDED in action_msgs
        self.status = "SUCCEEDED" if status == 4 else f"ENDED({status})"
        self.get_logger().info(f"navigation result: {self.status}")


def main():
    rclpy.init()
    node = GoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
