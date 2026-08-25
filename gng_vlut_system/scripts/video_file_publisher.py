#!/usr/bin/env python3

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VideoFilePublisher(Node):
    def __init__(self) -> None:
        super().__init__("video_file_publisher")

        self.declare_parameter("video_path", "")
        self.declare_parameter("image_topic", "/video/image_raw")
        self.declare_parameter("frame_id", "video_optical_frame")
        self.declare_parameter("enable_loop", True)
        self.declare_parameter("publish_hz", 0.0)

        video_path = str(self.get_parameter("video_path").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.enable_loop = bool(self.get_parameter("enable_loop").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        if not video_path:
            raise ValueError("video_pathは空にできません")
        if not os.path.isfile(video_path):
            raise FileNotFoundError(f"動画ファイルがありません: {video_path}")
        if not image_topic:
            raise ValueError("image_topicは空にできません")
        if not self.frame_id:
            raise ValueError("frame_idは空にできません")
        if publish_hz < 0.0:
            raise ValueError("publish_hzは0以上が必要です")

        self.capture = cv2.VideoCapture(video_path)
        if not self.capture.isOpened():
            raise RuntimeError(f"動画ファイルを開けません: {video_path}")

        source_hz = float(self.capture.get(cv2.CAP_PROP_FPS))
        if publish_hz == 0.0:
            publish_hz = source_hz if source_hz > 0.0 else 30.0
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, image_topic, qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_frame)
        self.has_finished = False
        self.get_logger().info(
            f"動画配信を開始: path={video_path}, topic={image_topic}, "
            f"publish_hz={publish_hz:.3f}, loop={self.enable_loop}"
        )

    def publish_frame(self) -> None:
        has_frame, frame = self.capture.read()
        if not has_frame and self.enable_loop:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            has_frame, frame = self.capture.read()
        if not has_frame:
            if not self.has_finished:
                self.get_logger().info("動画終端へ到達")
                self.has_finished = True
                self.timer.cancel()
            return

        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def destroy_node(self):
        self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoFilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
