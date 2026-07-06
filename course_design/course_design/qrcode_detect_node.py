import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from course_design.config_utils import load_config_from_node


class QRCodeDetectNode(Node):
    def __init__(self):
        super().__init__('qrcode_detect_node')
        self.config, self.config_path = load_config_from_node(self)
        self.qrcode_config = self.config.get('qrcode', {})
        self.image_topic = self.qrcode_config.get(
            'image_topic', '/depth_cam/rgb0/image_raw')
        self.target_topic = self.qrcode_config.get('target_topic', '/qrcode/target')
        self.image_result_topic = self.qrcode_config.get(
            'image_result_topic', '/qrcode/image_result')
        self.detection_period_sec = float(
            self.qrcode_config.get('detection_period_sec', 0.2))
        self.allowed_targets = set(
            self.qrcode_config.get(
                'allowed_targets',
                ['pick_area', 'goal_red', 'goal_green', 'goal_blue'],
            )
        )
        self.last_detection_time = 0.0
        self.last_logged_target = ''

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.target_pub = self.create_publisher(String, self.target_topic, 10)
        self.image_pub = self.create_publisher(Image, self.image_result_topic, 1)
        self.create_subscription(Image, self.image_topic, self.image_callback, 1)

        self.get_logger().info(
            f'QR code detector started. config={self.config_path} '
            f'image_topic={self.image_topic} target_topic={self.target_topic} '
            f'allowed={sorted(self.allowed_targets)}'
        )

    def image_callback(self, ros_image):
        now = time.monotonic()
        if now - self.last_detection_time < self.detection_period_sec:
            return
        self.last_detection_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to convert camera image: {exc}')
            return

        image = np.array(image, dtype=np.uint8)
        targets, points = self.detect_targets(image)
        self.publish_targets(targets)
        self.publish_debug_image(image, points, ros_image.header)

    def detect_targets(self, image):
        try:
            ret_qr, decoded_info, points, _ = self.detector.detectAndDecodeMulti(image)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'QR decode failed: {exc}')
            return [], None

        if not ret_qr:
            return [], None

        targets = []
        for raw_text in decoded_info:
            target = (raw_text or '').strip()
            if not target:
                continue
            if target not in self.allowed_targets:
                self.get_logger().warn(
                    f'Ignoring QR text "{target}". '
                    f'Allowed targets: {sorted(self.allowed_targets)}'
                )
                continue
            targets.append(target)
        return targets, points

    def publish_targets(self, targets):
        for target in targets:
            msg = String()
            msg.data = target
            self.target_pub.publish(msg)
            if target != self.last_logged_target:
                self.get_logger().info(f'QR target detected: {target}')
                self.last_logged_target = target

    def publish_debug_image(self, image, points, header):
        if points is not None:
            for point_set in points:
                if point_set is not None:
                    cv2.polylines(
                        image,
                        [point_set.astype(int)],
                        True,
                        (0, 255, 0),
                        3,
                    )
        try:
            result = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            result.header = header
            self.image_pub.publish(result)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to publish QR debug image: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
