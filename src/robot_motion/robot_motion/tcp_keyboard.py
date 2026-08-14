import sys
import termios
import tty

import rclpy

from rclpy.node import Node
from std_msgs.msg import String


KEY_MAP = {
    "w": "x+",
    "s": "x-",
    "a": "y+",
    "d": "y-",
    "q": "z+",
    "e": "z-",
}


class TCPKeyboard(Node):

    def __init__(self):

        super().__init__("tcp_keyboard")

        self.publisher = self.create_publisher(
            String,
            "/tcp_jog",
            10,
        )

        self.get_logger().info(
            "Sterowanie TCP:"
        )

        self.get_logger().info(
            "W/S = +X/-X"
        )

        self.get_logger().info(
            "A/D = +Y/-Y"
        )

        self.get_logger().info(
            "Q/E = +Z/-Z"
        )

        self.get_logger().info(
            "X = wyjscie"
        )


    def send_command(
        self,
        command,
    ):

        msg = String()

        msg.data = command

        self.publisher.publish(msg)

        self.get_logger().info(
            f"TX: {command}"
        )


def get_key():

    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)

    try:

        tty.setraw(fd)

        key = sys.stdin.read(1)

    finally:

        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings,
        )

    return key


def main(args=None):

    rclpy.init(args=args)

    node = TCPKeyboard()

    print()
    print("============================")
    print("TCP KEYBOARD")
    print("============================")
    print("W/S : +X / -X")
    print("A/D : +Y / -Y")
    print("Q/E : +Z / -Z")
    print("X   : wyjscie")
    print("============================")
    print()

    try:

        while rclpy.ok():

            key = get_key().lower()

            if key == "x":

                print()
                print("Koniec.")

                break


            if key in KEY_MAP:

                command = KEY_MAP[key]

                node.send_command(
                    command
                )

                rclpy.spin_once(
                    node,
                    timeout_sec=0.05
                )

            else:

                print(
                    f"\nNieznany klawisz: {repr(key)}"
                )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
