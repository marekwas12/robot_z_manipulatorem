import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
from serial import SerialException


class SerialBridgeNode(Node):

    ALLOWED_COMMANDS = {
        "w", "s",
        "a", "d",
        "f", "g",
        "x", "z",
        "t", "y",
        "u", "i",
        "o", "p",
        "h", "j",
        "k", "l",
        "0", "1", "2", "3", "4",
        "5", "6", "7", "8",
    }

    def __init__(self):

        super().__init__("serial_bridge_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port = (
            self.get_parameter("port")
            .get_parameter_value()
            .string_value
        )

        self.baudrate = (
            self.get_parameter("baudrate")
            .get_parameter_value()
            .integer_value
        )

        self.serial_connection = None
        self.serial_lock = threading.Lock()

        #
        # ROS
        #

        self.subscription = self.create_subscription(
            String,
            "/manipulator/command",
            self.command_callback,
            10,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/robot/status",
            10,
        )

        #
        # Timery
        #

        self.reconnect_timer = self.create_timer(
            2.0,
            self.ensure_serial_connection,
        )

        self.read_timer = self.create_timer(
            0.05,
            self.read_serial_data,
        )

        self.ensure_serial_connection()

        self.get_logger().info(
            "Serial Bridge uruchomiony."
        )

    # =====================================================
    # POŁĄCZENIE Z ARDUINO
    # =====================================================

    def ensure_serial_connection(self):

        if (
            self.serial_connection is not None
            and self.serial_connection.is_open
        ):
            return

        try:

            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=1.0,
            )

            #
            # Arduino resetuje się po otwarciu portu
            #

            time.sleep(2)

            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()

            self.get_logger().info(
                f"Połączono z {self.port}"
            )

        except (SerialException, OSError) as e:

            self.serial_connection = None

            self.get_logger().warn(
                f"Brak Arduino: {e}"
            )

    # =====================================================
    # WYSYŁANIE DO ARDUINO
    # =====================================================

    def command_callback(self, msg: String):

        command = msg.data.strip().lower()

        if len(command) != 1:
            return

        if command not in self.ALLOWED_COMMANDS:
            return

        if (
            self.serial_connection is None
            or not self.serial_connection.is_open
        ):
            return

        try:

            with self.serial_lock:

                self.serial_connection.write(
                    command.encode("ascii")
                )

                self.serial_connection.flush()

        except (SerialException, OSError):

            self.close_serial()

    # =====================================================
    # ODBIERANIE Z ARDUINO
    # =====================================================

    def read_serial_data(self):

        if (
            self.serial_connection is None
            or not self.serial_connection.is_open
        ):
            return

        try:

            while self.serial_connection.in_waiting > 0:

                line = (
                    self.serial_connection
                    .readline()
                    .decode(
                        "utf-8",
                        errors="ignore"
                    )
                    .strip()
                )

                if not line:
                    continue

                #
                # JSON statusu robota
                #

                if (
                    line.startswith("{")
                    and line.endswith("}")
                ):

                    msg = String()
                    msg.data = line

                    self.status_publisher.publish(msg)

                else:

                    #
                    # zwykłe logi Arduino
                    #

                    self.get_logger().info(
                        f"Arduino: {line}"
                    )

        except (SerialException, OSError):

            self.close_serial()

    # =====================================================
    # ZAMKNIĘCIE
    # =====================================================

    def close_serial(self):

        if self.serial_connection is None:
            return

        try:

            if self.serial_connection.is_open:
                self.serial_connection.close()

        except SerialException:
            pass

        self.serial_connection = None

    def destroy_node(self):

        self.close_serial()

        return super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = SerialBridgeNode()

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