#!/usr/bin/env python3
"""
video_recorder_node
===================

Writes the annotated perception overlay (/perception/overlay/image_raw -- RGB
feed + drivable region + obstacles + plant count + nav status) straight to an
MP4 file. This is the "demonstration video" deliverable, produced deterministically
from the perception output rather than by screen-grabbing.

Combine with `record_video.sh` if you also want a screen capture of Gazebo/RViz.
"""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__("video_recorder_node")
        self.declare_parameter("topic", "/perception/overlay/image_raw")
        self.declare_parameter("out_path", "/tmp/farm_run_overlay.mp4")
        self.declare_parameter("fps", 15.0)

        self.out_path = self.get_parameter("out_path").value
        self.fps = float(self.get_parameter("fps").value)
        self.bridge = CvBridge()
        self.writer = None

        self.create_subscription(Image, self.get_parameter("topic").value,
                                 self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"recording overlay -> {self.out_path}")

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
        self.writer.write(frame)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"saved video -> {self.out_path}")


def main():
    rclpy.init()
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
