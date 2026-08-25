#!/usr/bin/env python3

import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloPersonDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_person_detector")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/perception/person/detections_2d")
        self.declare_parameter("annotated_image_topic", "/perception/person/annotated_image")
        self.declare_parameter("person_detected_topic", "/perception/person/is_detected")
        self.declare_parameter(
            "inference_healthy_topic", "/perception/person/is_inference_healthy"
        )
        self.declare_parameter("model_path", "yolo11n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("confidence_th", 0.20)
        self.declare_parameter("iou_th", 0.70)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("max_inference_hz", 5.0)
        self.declare_parameter("enable_annotated_image", True)

        image_topic = str(self.get_parameter("image_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        person_detected_topic = str(self.get_parameter("person_detected_topic").value)
        inference_healthy_topic = str(
            self.get_parameter("inference_healthy_topic").value
        )
        model_path = str(self.get_parameter("model_path").value)
        self.device = str(self.get_parameter("device").value)
        self.confidence_th = float(self.get_parameter("confidence_th").value)
        self.iou_th = float(self.get_parameter("iou_th").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.max_inference_hz = float(self.get_parameter("max_inference_hz").value)
        self.enable_annotated_image = bool(
            self.get_parameter("enable_annotated_image").value
        )

        if not 0.0 <= self.confidence_th <= 1.0:
            raise ValueError("confidence_thは0から1の範囲が必要です")
        if not 0.0 <= self.iou_th <= 1.0:
            raise ValueError("iou_thは0から1の範囲が必要です")
        if self.image_size < 1:
            raise ValueError("image_sizeは1以上が必要です")
        if self.max_inference_hz <= 0.0:
            raise ValueError("max_inference_hzは正数が必要です")

        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                "Ultralyticsがありません。"
                "Docker imageを再buildするかpipで導入してください"
            ) from error

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.min_inference_interval = 1.0 / self.max_inference_hz
        self.last_inference_time = 0.0
        self.detections_publisher = self.create_publisher(
            Detection2DArray, detections_topic, 10
        )
        self.person_detected_publisher = self.create_publisher(
            Bool, person_detected_topic, 10
        )
        self.inference_healthy_publisher = self.create_publisher(
            Bool, inference_healthy_topic, 10
        )
        self.annotated_image_publisher = self.create_publisher(
            Image, annotated_image_topic, qos_profile_sensor_data
        )
        self.image_subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            "YOLO人物検出を開始: "
            f"image={image_topic}, model={model_path}, device={self.device}"
        )

    def image_callback(self, message: Image) -> None:
        current_time = time.monotonic()
        if current_time - self.last_inference_time < self.min_inference_interval:
            return
        self.last_inference_time = current_time

        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = self.model.predict(
                source=image,
                classes=[0],
                conf=self.confidence_th,
                iou=self.iou_th,
                imgsz=self.image_size,
                device=self.device,
                verbose=False,
            )[0]
        except Exception as error:
            self.get_logger().error(f"YOLO推論失敗: {error}")
            self.inference_healthy_publisher.publish(Bool(data=False))
            self.person_detected_publisher.publish(Bool(data=True))
            return

        detections_message = Detection2DArray()
        detections_message.header = message.header
        boxes = result.boxes
        if boxes is not None:
            coordinates = boxes.xyxy.detach().cpu().numpy()
            scores = boxes.conf.detach().cpu().numpy()
            for detection_idx, (coordinate, score) in enumerate(zip(coordinates, scores)):
                min_x, min_y, max_x, max_y = (float(value) for value in coordinate)
                detection = Detection2D()
                detection.header = message.header
                detection.id = str(detection_idx)
                detection.bbox.center.position.x = (min_x + max_x) * 0.5
                detection.bbox.center.position.y = (min_y + max_y) * 0.5
                detection.bbox.size_x = max_x - min_x
                detection.bbox.size_y = max_y - min_y
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "person"
                hypothesis.hypothesis.score = float(score)
                detection.results.append(hypothesis)
                detections_message.detections.append(detection)

        is_person_detected = bool(detections_message.detections)
        self.detections_publisher.publish(detections_message)
        self.inference_healthy_publisher.publish(Bool(data=True))
        self.person_detected_publisher.publish(Bool(data=is_person_detected))

        if self.enable_annotated_image:
            annotated_image = result.plot()
            annotated_message = self.bridge.cv2_to_imgmsg(
                annotated_image, encoding="bgr8"
            )
            annotated_message.header = message.header
            self.annotated_image_publisher.publish(annotated_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
