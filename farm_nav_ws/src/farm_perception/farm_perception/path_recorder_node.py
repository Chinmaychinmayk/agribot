#!/usr/bin/env python3
"""
path_recorder_node
==================

Records the run for the deliverables:

  * Accumulates the ACTUAL trajectory by sampling the map->base_footprint TF,
    republished as nav_msgs/Path on /actual_path (drawn in RViz).
  * Caches the latest Nav2 global PLAN from /plan.
  * On shutdown (Ctrl-C) writes a matplotlib figure overlaying the planned path
    and the actual trajectory, plus the detected plant positions, to `out_path`.
    This is the "path visualization" deliverable.

It also prints simple path-efficiency numbers (actual vs planned length).
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import tf2_ros


class PathRecorderNode(Node):
    def __init__(self):
        super().__init__("path_recorder_node")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("out_path", "/tmp/farm_run_path.png")
        self.declare_parameter("sample_dist", 0.05)   # [m] min spacing to log

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.out_path = self.get_parameter("out_path").value
        self.sample_dist = float(self.get_parameter("sample_dist").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.actual = Path()
        self.actual.header.frame_id = self.map_frame
        self.planned = None
        self.plants = []

        self.actual_pub = self.create_publisher(Path, "/actual_path", 10)
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(MarkerArray, "/perception/plant_markers",
                                 self._plants_cb, 10)
        self.create_timer(0.2, self._sample)

    def _plan_cb(self, msg):
        self.planned = msg

    def _plants_cb(self, msg):
        self.plants = [(m.pose.position.x, m.pose.position.y) for m in msg.markers]

    def _sample(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:                       # noqa: BLE001
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        if self.actual.poses:
            last = self.actual.poses[-1].pose.position
            if math.hypot(x - last.x, y - last.y) < self.sample_dist:
                return
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        self.actual.poses.append(ps)
        self.actual.header.stamp = ps.header.stamp
        self.actual_pub.publish(self.actual)

    @staticmethod
    def _length(poses):
        d = 0.0
        for a, b in zip(poses[:-1], poses[1:]):
            d += math.hypot(b.pose.position.x - a.pose.position.x,
                            b.pose.position.y - a.pose.position.y)
        return d

    def save(self):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:                  # noqa: BLE001
            self.get_logger().error(f"matplotlib unavailable: {e}")
            return

        fig, ax = plt.subplots(figsize=(8, 6))
        if self.planned and self.planned.poses:
            px = [p.pose.position.x for p in self.planned.poses]
            py = [p.pose.position.y for p in self.planned.poses]
            ax.plot(px, py, "b--", lw=2, label="planned (last)")
        if self.actual.poses:
            ax_ = [p.pose.position.x for p in self.actual.poses]
            ay = [p.pose.position.y for p in self.actual.poses]
            ax.plot(ax_, ay, "r-", lw=2, label="actual")
            ax.plot(ax_[0], ay[0], "go", ms=10, label="start")
            ax.plot(ax_[-1], ay[-1], "ks", ms=10, label="end")
        if self.plants:
            ax.scatter([p[0] for p in self.plants], [p[1] for p in self.plants],
                       c="magenta", marker="^", s=60, label=f"plants ({len(self.plants)})")

        actual_len = self._length(self.actual.poses)
        plan_len = self._length(self.planned.poses) if self.planned else 0.0
        ax.set_title(f"Run path  |  actual={actual_len:.1f} m  planned={plan_len:.1f} m")
        ax.set_xlabel("x [m] (map)"); ax.set_ylabel("y [m] (map)")
        ax.axis("equal"); ax.grid(True); ax.legend()
        fig.savefig(self.out_path, dpi=120, bbox_inches="tight")
        self.get_logger().info(
            f"saved path plot -> {self.out_path} "
            f"(actual {actual_len:.1f} m, planned {plan_len:.1f} m, "
            f"{len(self.plants)} plants)")


def main():
    rclpy.init()
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
