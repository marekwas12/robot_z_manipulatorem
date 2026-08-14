import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class DualCameraNode(Node):
    def __init__(self):
        super().__init__("dual_camera_node")

        self.left_publisher = self.create_publisher(
            CompressedImage,
            "/camera_left/image_raw/compressed",
            5,
        )

        self.right_publisher = self.create_publisher(
            CompressedImage,
            "/camera_right/image_raw/compressed",
            5,
        )

        # Najczęściej kamera 0 i kamera 1.
        # Jeśli będą zamienione, zamień liczby miejscami.
        self.left_camera = cv2.VideoCapture(0)
        self.right_camera = cv2.VideoCapture(1)

        self.configure_camera(self.left_camera)
        self.configure_camera(self.right_camera)

        if not self.left_camera.isOpened():
            self.get_logger().error(
                "Nie można otworzyć lewej kamery /dev/video0"
            )

        if not self.right_camera.isOpened():
            self.get_logger().error(
                "Nie można otworzyć prawej kamery /dev/video1"
            )

        # Około 10 klatek na sekundę.
        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.get_logger().info(
            "Dual Camera Node uruchomiony"
        )

    def configure_camera(self, camera):
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 10)

    def create_compressed_message(self, frame):
        message = CompressedImage()

        message.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        message.format = "jpeg"

        success, compressed = cv2.imencode(
            ".jpg",
            frame,
            [
                int(cv2.IMWRITE_JPEG_QUALITY),
                70,
            ],
        )

        if not success:
            return None

        message.data = np.asarray(
            compressed,
            dtype=np.uint8,
        ).tobytes()

        return message

    def timer_callback(self):
        left_success, left_frame = self.left_camera.read()
        right_success, right_frame = self.right_camera.read()

        if left_success:
            left_message = self.create_compressed_message(
                left_frame
            )

            if left_message is not None:
                self.left_publisher.publish(
                    left_message
                )

        if right_success:
            right_message = self.create_compressed_message(
                right_frame
            )

            if right_message is not None:
                self.right_publisher.publish(
                    right_message
                )

    def destroy_node(self):
        self.left_camera.release()
        self.right_camera.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DualCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()