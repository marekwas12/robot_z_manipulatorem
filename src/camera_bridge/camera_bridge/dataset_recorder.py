import os
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String


IMAGE_TOPIC = "/stereo/left_rectified"
COMMAND_TOPIC = "/dataset_recorder/command"

DATASET_DIR = os.path.expanduser(
    "~/robot_ws/datasets/robot_objects_raw"
)

IMAGE_DIR = os.path.join(
    DATASET_DIR,
    "images"
)

AUTO_INTERVAL_SEC = 0.5
AUTO_DURATION_SEC = 30.0


class DatasetRecorder(Node):

    def __init__(self):

        super().__init__(
            "dataset_recorder"
        )

        os.makedirs(
            IMAGE_DIR,
            exist_ok=True,
        )

        self.latest_image = None

        self.saved_count = (
            self.count_existing_images()
        )

        self.auto_active = False
        self.auto_started_at = None
        self.auto_saved = 0

        self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            10,
        )

        self.auto_timer = (
            self.create_timer(
                AUTO_INTERVAL_SEC,
                self.auto_timer_callback,
            )
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "DATASET RECORDER"
        )

        self.get_logger().info(
            f"Obraz: {IMAGE_TOPIC}"
        )

        self.get_logger().info(
            f"Folder: {IMAGE_DIR}"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "Komendy:"
        )

        self.get_logger().info(
            "save  - zapisz 1 klatke"
        )

        self.get_logger().info(
            "burst - zapisz 10 klatek"
        )

        self.get_logger().info(
            "auto  - zapis co 0.5 s przez 30 s"
        )

        self.get_logger().info(
            "stop  - zatrzymaj auto"
        )

        self.get_logger().info(
            "info  - status"
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # COUNT
    # ========================================================

    def count_existing_images(self):

        try:

            files = [
                name
                for name in os.listdir(
                    IMAGE_DIR
                )
                if name.lower().endswith(
                    (
                        ".jpg",
                        ".jpeg",
                        ".png",
                    )
                )
            ]

            return len(files)

        except OSError:

            return 0


    # ========================================================
    # IMAGE
    # ========================================================

    def image_callback(
        self,
        msg,
    ):

        if msg.encoding != "bgr8":

            self.get_logger().warn(
                f"Nieobslugiwany encoding: "
                f"{msg.encoding}"
            )

            return

        expected_size = (
            msg.height
            *
            msg.width
            *
            3
        )

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        if raw.size < expected_size:

            return

        image = raw[
            :expected_size
        ].reshape(
            msg.height,
            msg.width,
            3,
        )

        self.latest_image = (
            np.ascontiguousarray(
                image.copy()
            )
        )


    # ========================================================
    # COMMAND
    # ========================================================

    def command_callback(
        self,
        msg,
    ):

        command = (
            msg.data
            .strip()
            .lower()
        )

        if command == "save":

            self.save_image()

        elif command == "burst":

            self.save_burst()

        elif command == "auto":

            self.start_auto()

        elif command == "stop":

            self.stop_auto()

        elif command == "info":

            self.print_info()

        else:

            self.get_logger().warn(
                f"Nieznana komenda: "
                f"{command}"
            )


    # ========================================================
    # FILENAME
    # ========================================================

    def make_filename(
        self,
    ):

        now = datetime.now()

        stamp = now.strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        return os.path.join(
            IMAGE_DIR,
            f"frame_{stamp}.jpg",
        )


    # ========================================================
    # SAVE
    # ========================================================

    def save_image(
        self,
    ):

        if self.latest_image is None:

            self.get_logger().error(
                "Nie mam jeszcze obrazu."
            )

            return False

        filename = (
            self.make_filename()
        )

        ok = cv2.imwrite(
            filename,
            self.latest_image,
            [
                int(
                    cv2.IMWRITE_JPEG_QUALITY
                ),
                95,
            ],
        )

        if not ok:

            self.get_logger().error(
                "Nie udalo sie zapisac obrazu."
            )

            return False

        self.saved_count += 1

        self.get_logger().info(
            f"Zapisano #{self.saved_count}: "
            f"{os.path.basename(filename)}"
        )

        return True


    # ========================================================
    # BURST
    # ========================================================

    def save_burst(
        self,
    ):

        if self.latest_image is None:

            self.get_logger().error(
                "Nie mam jeszcze obrazu."
            )

            return

        self.get_logger().info(
            "BURST: zapisuje 10 klatek."
        )

        for _ in range(10):

            self.save_image()


    # ========================================================
    # AUTO
    # ========================================================

    def start_auto(
        self,
    ):

        if self.auto_active:

            self.get_logger().warn(
                "AUTO juz dziala."
            )

            return

        if self.latest_image is None:

            self.get_logger().error(
                "Nie mam jeszcze obrazu."
            )

            return

        self.auto_active = True

        self.auto_started_at = (
            self.get_clock().now()
        )

        self.auto_saved = 0

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "AUTO START"
        )

        self.get_logger().info(
            f"Interwal: "
            f"{AUTO_INTERVAL_SEC:.1f} s"
        )

        self.get_logger().info(
            f"Czas: "
            f"{AUTO_DURATION_SEC:.1f} s"
        )

        self.get_logger().info(
            "Oczekiwane zdjecia: "
            f"{int(AUTO_DURATION_SEC / AUTO_INTERVAL_SEC)}"
        )

        self.get_logger().info(
            "=========================================="
        )


    def stop_auto(
        self,
    ):

        if not self.auto_active:

            self.get_logger().info(
                "AUTO nie jest aktywne."
            )

            return

        self.auto_active = False

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "AUTO STOP"
        )

        self.get_logger().info(
            f"Zapisano w tej serii: "
            f"{self.auto_saved}"
        )

        self.get_logger().info(
            "=========================================="
        )


    def auto_timer_callback(
        self,
    ):

        if not self.auto_active:

            return

        if self.auto_started_at is None:

            return

        now = self.get_clock().now()

        elapsed_sec = (
            now
            -
            self.auto_started_at
        ).nanoseconds / 1e9

        if elapsed_sec >= AUTO_DURATION_SEC:

            self.stop_auto()

            return

        ok = (
            self.save_image()
        )

        if ok:

            self.auto_saved += 1


    # ========================================================
    # INFO
    # ========================================================

    def print_info(
        self,
    ):

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"Wszystkich zdjec: "
            f"{self.saved_count}"
        )

        self.get_logger().info(
            f"AUTO: "
            f"{'ON' if self.auto_active else 'OFF'}"
        )

        if self.auto_active:

            self.get_logger().info(
                f"Zapisano w tej serii: "
                f"{self.auto_saved}"
            )

        self.get_logger().info(
            "=========================================="
        )


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = DatasetRecorder()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()