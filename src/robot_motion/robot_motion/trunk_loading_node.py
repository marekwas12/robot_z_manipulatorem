import rclpy

from rclpy.node import Node
from std_msgs.msg import String


COMMAND_TOPIC = "/trunk_loading/command"
STATUS_TOPIC = "/trunk_loading/status"

PICK_COMMAND_TOPIC = "/cube_pick/command"
PICK_STATUS_TOPIC = "/cube_pick/status"

PLACE_COMMAND_TOPIC = "/cube_place/command"
PLACE_STATUS_TOPIC = "/cube_place/status"

PLACE_DELAY_S = 0.50
PLACE_TIMEOUT_S = 45.0
RETRY_PICK_DELAY_S = 0.50

IDLE = 0
WAIT_PICK = 1
WAIT_PLACE_DELAY = 2
WAIT_PLACE = 3
WAIT_PICK_CHECK_DECISION = 4
WAIT_ABORT_READY = 5
WAIT_RETRY_DELAY = 6


class TrunkLoadingNode(Node):

    def __init__(self):
        super().__init__("trunk_loading_node")

        self.state = IDLE
        self.place_after = None
        self.place_wait_started = None
        self.retry_after = None

        self.command_sub = self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            10,
        )

        self.pick_status_sub = self.create_subscription(
            String,
            PICK_STATUS_TOPIC,
            self.pick_status_callback,
            10,
        )

        self.place_status_sub = self.create_subscription(
            String,
            PLACE_STATUS_TOPIC,
            self.place_status_callback,
            10,
        )

        self.pick_pub = self.create_publisher(
            String,
            PICK_COMMAND_TOPIC,
            10,
        )

        self.place_pub = self.create_publisher(
            String,
            PLACE_COMMAND_TOPIC,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.update,
        )

        self.get_logger().info("==========================================")
        self.get_logger().info("TRUNK LOADING NODE")
        self.get_logger().info("start = CUBE PICK FULL -> CUBE PLACE START")
        self.get_logger().info("place = TYLKO odlozenie juz podniesionej kostki do bagaznika")
        self.get_logger().info(f"COMMAND: {COMMAND_TOPIC}")
        self.get_logger().info(f"STATUS : {STATUS_TOPIC}")
        self.get_logger().info("==========================================")

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)

        self.status_pub.publish(msg)

        self.get_logger().info(
            f"TRUNK STATUS -> {msg.data}"
        )

    def publish_string(self, publisher, data):
        msg = String()
        msg.data = str(data)
        publisher.publish(msg)

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd == "start":
            if self.state != IDLE:
                self.get_logger().warn(
                    "Sekwencja zaladunku juz trwa."
                )
                return

            self.get_logger().info(
                "START ZALADUNKU: najpierw cube_pick full."
            )

            self.publish_status("running:pick")

            self.publish_string(
                self.pick_pub,
                "full",
            )

            self.state = WAIT_PICK
            return

        if cmd in ("place", "to_trunk", "bagaznik"):
            if self.state != IDLE:
                self.get_logger().warn(
                    "Sekwencja zaladunku juz trwa."
                )
                return

            self.get_logger().info(
                "PLACE-ONLY: kostka jest juz w chwytaku -> pomijam cube_pick."
            )
            self.publish_status("running:place")
            self.publish_string(
                self.place_pub,
                "place",
            )
            self.state = WAIT_PLACE
            self.place_wait_started = self.get_clock().now()
            return

        if cmd == "pickup_ok":
            if self.state != WAIT_PICK_CHECK_DECISION:
                self.get_logger().warn(
                    "pickup_ok ignorowane - nie czekam na decyzje."
                )
                return

            self.get_logger().info(
                "Cleanup potwierdzil: kostki NIE MA na ziemi -> kontynuuje PLACE."
            )

            self.publish_string(
                self.place_pub,
                "continue",
            )

            self.publish_status(
                "running:place"
            )

            self.state = WAIT_PLACE
            self.place_wait_started = self.get_clock().now()
            return

        if cmd == "pickup_failed":
            if self.state != WAIT_PICK_CHECK_DECISION:
                self.get_logger().warn(
                    "pickup_failed ignorowane - nie czekam na decyzje."
                )
                return

            self.get_logger().warn(
                "Cleanup widzi kostke nadal na ziemi -> PLACE abort -> READY -> RETRY PICK."
            )

            self.publish_string(
                self.place_pub,
                "abort",
            )

            self.publish_status(
                "running:abort_to_ready"
            )

            self.state = WAIT_ABORT_READY
            return

        if cmd == "reset":
            self.state = IDLE
            self.place_after = None
            self.place_wait_started = None
            self.retry_after = None
            self.publish_status("idle")
            self.get_logger().warn("RESET -> IDLE")
            return

        if cmd == "status":
            names = {
                IDLE: "idle",
                WAIT_PICK: "wait_pick",
                WAIT_PLACE_DELAY: "wait_place_delay",
                WAIT_PLACE: "wait_place",
                WAIT_PICK_CHECK_DECISION: "wait_pick_check_decision",
                WAIT_ABORT_READY: "wait_abort_ready",
                WAIT_RETRY_DELAY: "wait_retry_delay",
            }

            self.publish_status(
                names.get(
                    self.state,
                    f"state:{self.state}",
                )
            )
            return

        self.get_logger().warn(
            f"Nieznana komenda: {cmd}"
        )

    def pick_status_callback(self, msg):
        status = msg.data.strip().lower()

        if self.state != WAIT_PICK:
            return

        if status == "done:full":
            self.get_logger().info(
                "CUBE PICK zakonczony poprawnie -> za chwile PLACE."
            )

            self.publish_status("pick_done")

            self.place_after = (
                self.get_clock().now()
                +
                rclpy.duration.Duration(
                    seconds=PLACE_DELAY_S
                )
            )

            self.state = WAIT_PLACE_DELAY
            return

        if status == "error:full":
            self.get_logger().error(
                "CUBE PICK zakonczyl sie bledem. PLACE NIE zostanie uruchomiony."
            )

            self.publish_status("error:pick")
            self.state = IDLE
            return

    def place_status_callback(self, msg):
        status = msg.data.strip().lower()

        if status == "check:pickup":
            if self.state != WAIT_PLACE:
                return

            self.get_logger().info(
                "CUBE PLACE jest w P2 -> prosze cleanup o sprawdzenie kostki."
            )

            self.publish_status(
                "check:pickup"
            )

            self.state = WAIT_PICK_CHECK_DECISION
            self.place_wait_started = None
            return

        if status == "aborted:pickup_check":
            if self.state != WAIT_ABORT_READY:
                return

            self.get_logger().warn(
                "CUBE PLACE wrocil do READY -> za chwile ponawiam CUBE PICK."
            )

            self.publish_status(
                "retry:ready"
            )

            self.retry_after = (
                self.get_clock().now()
                +
                rclpy.duration.Duration(
                    seconds=RETRY_PICK_DELAY_S
                )
            )

            self.state = WAIT_RETRY_DELAY
            return

        if self.state != WAIT_PLACE:
            return

        if status in ("done:start", "done:place"):
            self.get_logger().info(
                "CUBE PLACE zakonczony poprawnie -> ZALADUNEK DONE."
            )
            self.publish_status("done")
            self.state = IDLE
            self.place_after = None
            self.place_wait_started = None
            self.retry_after = None
            return

        if status == "error:start":
            self.get_logger().error(
                "CUBE PLACE zakonczyl sie bledem."
            )
            self.publish_status("error:place")
            self.state = IDLE
            self.place_after = None
            self.place_wait_started = None
            self.retry_after = None
            return

    def update(self):
        if self.state == WAIT_RETRY_DELAY:
            if self.retry_after is None:
                self.retry_after = self.get_clock().now()

            if self.get_clock().now() < self.retry_after:
                return

            self.get_logger().warn(
                "RETRY PICK -> /cube_pick/command = full"
            )

            self.publish_status(
                "running:pick_retry"
            )

            self.publish_string(
                self.pick_pub,
                "full",
            )

            self.retry_after = None
            self.state = WAIT_PICK
            return

        # Timeout oczekiwania na cube_place.
        if self.state == WAIT_PLACE:
            if self.place_wait_started is None:
                self.place_wait_started = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.place_wait_started
            ).nanoseconds / 1e9

            if elapsed > PLACE_TIMEOUT_S:
                self.get_logger().error(
                    "TIMEOUT oczekiwania na cube_place/status."
                )
                self.publish_status("error:place_timeout")
                self.state = IDLE
                self.place_wait_started = None
            return

        if self.state != WAIT_PLACE_DELAY:
            return

        if self.place_after is None:
            return

        if self.get_clock().now() < self.place_after:
            return

        self.get_logger().info(
            "URUCHAMIAM CUBE PLACE."
        )

        self.publish_string(
            self.place_pub,
            "start",
        )

        self.publish_status("running:place")

        # Teraz NIE wracamy od razu do IDLE.
        # Czekamy na /cube_place/status = done:start, aby misja wiedziala,
        # ze kostka faktycznie wpadla do bagaznika i ramie wrocilo do READY.
        self.state = WAIT_PLACE
        self.place_after = None
        self.place_wait_started = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    node = TrunkLoadingNode()

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