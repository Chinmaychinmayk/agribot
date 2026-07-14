#!/usr/bin/env python3
"""
plant_counter_node
==================

Detects and counts plants from the geometric obstacle cloud produced by
`lane_perception_node`, taking care never to double-count a plant seen from
multiple viewpoints.

Pipeline (all geometric, no colour):

  1. Subscribe to /perception/obstacles (points above the ground plane, already
     expressed in the drift-corrected global frame, `map`).
  2. Keep only points in the FOLIAGE height band. The raised plant beds sit at
     ~0.10 m and are excluded; plant foliage spheres peak around 0.42 m, so the
     band cleanly isolates plant canopies from the beds.
  3. Cluster the surviving points on the ground (XY) plane with a lightweight
     grid + connected-components clusterer (no external deps).
  4. Each cluster centroid is a plant *observation* in the world frame. Because
     observations live in the fixed `map` frame, the same physical plant lands
     at (almost) the same XY every time it is seen. We associate each new
     observation with the nearest tracked plant within PLANT_ASSOC_RADIUS; if
     none is close enough, a new track is created. A track is only *counted*
     once confirmed in >= MIN_OBSERVATIONS frames (rejects one-frame flicker).
  5. Publish the running count (/perception/plant_count) and visualization
     markers (/perception/plant_markers) for RViz and the overlay.

This node is entirely HAND-ENGINEERED. The constants below are the only tuned
parameters and are reproduced in WRITEUP.md.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray


# ------------------------- tuned constants (hand-engineered) ----------------
FOLIAGE_MIN_Z = 0.28        # [m] bottom of the canopy band (above bed height)
FOLIAGE_MAX_Z = 0.70        # [m] top of the canopy band
GRID_CELL = 0.15            # [m] clustering resolution on the XY plane
MIN_CLUSTER_PTS = 4         # reject specks
MAX_CLUSTER_EXTENT = 0.8    # [m] reject anything bigger than a plant (e.g. bed)
PLANT_ASSOC_RADIUS = 0.45   # [m] same-plant association distance (dedup)
MIN_OBSERVATIONS = 2        # frames a track must persist before being counted


class PlantTrack:
    __slots__ = ("x", "y", "obs", "counted")

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.obs = 1
        self.counted = False

    def update(self, x, y):
        # running average keeps the estimate stable as viewpoints change
        a = 1.0 / (self.obs + 1)
        self.x = (1 - a) * self.x + a * x
        self.y = (1 - a) * self.y + a * y
        self.obs += 1


class PlantCounterNode(Node):
    def __init__(self):
        super().__init__("plant_counter_node")
        self.tracks = []
        self.frame_id = "map"

        self.count_pub = self.create_publisher(Int32, "/perception/plant_count", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/plant_markers", 10)
        self.create_subscription(PointCloud2, "/perception/obstacles",
                                 self.cloud_cb, 10)

        self.get_logger().info("plant_counter_node ready (geometric clustering).")

    # ----------------------------------------------------------------------
    def cloud_cb(self, msg):
        self.frame_id = msg.header.frame_id or "map"
        # read_points returns a numpy *structured* array on Humble
        raw = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if raw is None or len(raw) == 0:
            return
        pts = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float32)

        canopy = pts[(pts[:, 2] > FOLIAGE_MIN_Z) & (pts[:, 2] < FOLIAGE_MAX_Z)]
        if canopy.shape[0] >= MIN_CLUSTER_PTS:
            for cx, cy in self._cluster(canopy[:, :2]):
                self._associate(cx, cy)

        self._publish()

    # ----------------------------------------------------------------------
    def _cluster(self, xy):
        """Grid + connected-components clustering; yields (cx, cy) centroids."""
        cells = np.floor(xy / GRID_CELL).astype(np.int64)
        cell_pts = {}
        for (ci, cj), p in zip(map(tuple, cells), xy):
            cell_pts.setdefault((ci, cj), []).append(p)

        seen = set()
        for start in cell_pts:
            if start in seen:
                continue
            stack = [start]
            seen.add(start)
            members = []
            while stack:
                c = stack.pop()
                members.extend(cell_pts[c])
                ci, cj = c
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        nb = (ci + di, cj + dj)
                        if nb in cell_pts and nb not in seen:
                            seen.add(nb)
                            stack.append(nb)
            members = np.asarray(members)
            if members.shape[0] < MIN_CLUSTER_PTS:
                continue
            extent = members.max(axis=0) - members.min(axis=0)
            if max(extent) > MAX_CLUSTER_EXTENT:
                continue                      # too large to be a single plant
            yield float(members[:, 0].mean()), float(members[:, 1].mean())

    # ----------------------------------------------------------------------
    def _associate(self, x, y):
        best, best_d = None, PLANT_ASSOC_RADIUS
        for t in self.tracks:
            d = np.hypot(t.x - x, t.y - y)
            if d < best_d:
                best, best_d = t, d
        if best is None:
            self.tracks.append(PlantTrack(x, y))
        else:
            best.update(x, y)

    # ----------------------------------------------------------------------
    def _publish(self):
        count = 0
        markers = MarkerArray()
        for i, t in enumerate(self.tracks):
            if t.obs >= MIN_OBSERVATIONS:
                t.counted = True
            if not t.counted:
                continue
            count += 1
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "plants"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = t.x
            m.pose.position.y = t.y
            m.pose.position.z = 0.42
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.30
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.8
            markers.markers.append(m)

        self.count_pub.publish(Int32(data=count))
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = PlantCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
