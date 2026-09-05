#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String


# ============================================================
# JAZDA PÓŁAUTONOMICZNA
# ============================================================
#
# Telefon steruje robotem dokładnie jak w trybie manualnym.
# Ten node jest tylko warstwą bezpieczeństwa:
#
#   /autonomous_drive/command:
#       start
#       stop
#       0 / 1 / 2
#       i / u       - skręt jak w manualu
#       t / y       - kamera jak w manualu
#       pick        - cube_pick full
#
# Komendy trafiają dalej na:
#   /manipulator/command
#
# Bezpieczeństwo:
#   - HC-SR04 < 10 cm -> STOP + 2 piknięcia + blokada 2 s
#   - depth center < 10 cm -> STOP + 2 piknięcia
#   - przeszkoda z przodu/depth -> po piknięciu cofanie 1 s
#   - przeszkoda z tyłu -> po piknięciu jazda do przodu 1 s
#   - przeszkoda tylko z boku -> STOP, bez automatycznego ruchu
#   - brak/inf/stary HC-SR04 -> ignorowany
#
# Podczas cube_pick automatyczne uniki są wyłączone, żeby platforma
# nie ruszyła pod pracującym manipulatorem.
# ============================================================


COMMAND_TOPIC = "/autonomous_drive/command"
STATUS_TOPIC = "/autonomous_drive/status"

ARDUINO_COMMAND_TOPIC = "/manipulator/command"

CUBE_PICK_COMMAND_TOPIC = "/cube_pick/command"
CUBE_PICK_STATUS_TOPIC = "/cube_pick/status"

DEPTH_TOPIC = "/stereo/center_distance"

ULTRASONIC_TOPICS = {
    "front_left": "/ultrasonic/front_left",
    "front_center": "/ultrasonic/front_center",
    "front_right": "/ultrasonic/front_right",
    "left": "/ultrasonic/left",
    "right": "/ultrasonic/right",
    "rear": "/ultrasonic/rear",
}

FRONT_SENSORS = {
    "front_left",
    "front_center",
    "front_right",
}

SIDE_SENSORS = {
    "left",
    "right",
}


# ============================================================
# PROGI
# ============================================================

STOP_DISTANCE_M = 0.10
REAR_STOP_DISTANCE_M = 0.30
CLEAR_DISTANCE_M = 0.12

ULTRASONIC_FRESH_S = 1.00
DEPTH_FRESH_S = 0.80

AUTO_ESCAPE_TIME_S = 1.00
REAR_ESCAPE_TIME_S = 0.60
SAFETY_BLOCK_TIME_S = 2.00

BUZZER_PIP_S = 0.10
BUZZER_GAP_S = 0.10
BUZZER_TOTAL_S = (
    2 * BUZZER_PIP_S
    +
    BUZZER_GAP_S
)

STEERING_CENTER_DEG = 95.0


# ============================================================
# KOMENDY
# ============================================================

CMD_MANUAL_MODE = "M"
CMD_STOP = "0"
CMD_FORWARD = "1"
CMD_REVERSE = "2"

# Dokładnie manualne komendy, które mogą być przesłane z aplikacji.
PASSTHROUGH_COMMANDS = {
    "0",
    "1",
    "2",
    "i",
    "u",
    "t",
    "y",
}


IDLE = 0
READY = 1
BEEPING = 2
ESCAPING = 3
PICKING = 4


class SemiAutonomousDriveNode(Node):

    def __init__(self):
        super().__init__(
            "autonomous_drive_node"
        )

        self.enabled = False
        self.state = IDLE
        self.state_started = self.get_clock().now()

        self.escape_direction = None
        self.escape_source = None

        self.hazard_latched = False
        self.hazard_started = None
        self.current_drive_command = None

        self.ultrasonic = {
            key: {
                "value": None,
                "stamp": None,
            }
            for key in ULTRASONIC_TOPICS
        }

        self.depth_m = None
        self.depth_stamp = None

        self.arduino_pub = self.create_publisher(
            String,
            ARDUINO_COMMAND_TOPIC,
            20,
        )

        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            20,
        )

        self.pick_pub = self.create_publisher(
            String,
            CUBE_PICK_COMMAND_TOPIC,
            10,
        )

        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            20,
        )

        self.create_subscription(
            Float32,
            DEPTH_TOPIC,
            self.depth_callback,
            20,
        )

        self.create_subscription(
            String,
            CUBE_PICK_STATUS_TOPIC,
            self.pick_status_callback,
            20,
        )

        for key, topic in ULTRASONIC_TOPICS.items():
            self.create_subscription(
                Range,
                topic,
                lambda msg, sensor_key=key:
                    self.ultrasonic_callback(
                        sensor_key,
                        msg,
                    ),
                20,
            )

        self.timer = self.create_timer(
            0.05,
            self.update,
        )

        self.get_logger().info(
            "=========================================="
        )
        self.get_logger().info(
            "JAZDA PÓŁAUTONOMICZNA"
        )
        self.get_logger().info(
            "Sterowanie ręczne + bezpieczeństwo HC-SR04/depth"
        )
        self.get_logger().info(
            "PRÓG STOP = 10 cm | blokada = 2 s | brak pomiaru HC = IGNORE"
        )
        self.get_logger().info(
            "FRONT/depth -> STOP + pip pip + reverse 1 s"
        )
        self.get_logger().info(
            "REAR -> STOP + pip pip + forward 1 s"
        )
        self.get_logger().info(
            "SIDE -> STOP + pip pip"
        )
        self.get_logger().info(
            "=========================================="
        )

    # ========================================================
    # HELPERS
    # ========================================================

    def now(self):
        return self.get_clock().now()

    def elapsed(self):
        return (
            self.now() - self.state_started
        ).nanoseconds / 1e9

    def set_state(self, state):
        self.state = state
        self.state_started = self.now()

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)

        self.get_logger().info(
            f"SEMI STATUS -> {msg.data}"
        )

    def send_arduino(self, command):
        msg = String()
        msg.data = str(command)
        self.arduino_pub.publish(msg)

    def stop_robot(self):
        self.send_arduino(
            CMD_STOP
        )
        self.current_drive_command = None

    def set_steering_center(self):
        self.send_arduino(
            f"@S2:{STEERING_CENTER_DEG:.0f}"
        )

    def fresh(self, stamp, timeout_s):
        if stamp is None:
            return False

        return (
            (
                self.now() - stamp
            ).nanoseconds / 1e9
            <=
            timeout_s
        )

    # ========================================================
    # SENSOR CALLBACKS
    # ========================================================

    def ultrasonic_callback(
        self,
        key,
        msg,
    ):
        value = float(
            msg.range
        )

        # Brak echa / rozłączony przewód:
        # serial bridge publikuje inf. Ignorujemy.
        if (
            not math.isfinite(value)
            or
            value <= 0.0
        ):
            self.ultrasonic[key]["value"] = None
            self.ultrasonic[key]["stamp"] = None
            return

        self.ultrasonic[key]["value"] = value
        self.ultrasonic[key]["stamp"] = self.now()

    def depth_callback(
        self,
        msg,
    ):
        value = float(
            msg.data
        )

        if (
            not math.isfinite(value)
            or
            value <= 0.0
        ):
            self.depth_m = None
            self.depth_stamp = None
            return

        self.depth_m = value
        self.depth_stamp = self.now()

    # ========================================================
    # SENSOR READ
    # ========================================================

    def ultrasonic_value(
        self,
        key,
    ):
        item = self.ultrasonic[key]

        if (
            item["value"] is None
            or
            not self.fresh(
                item["stamp"],
                ULTRASONIC_FRESH_S,
            )
        ):
            return None

        return float(
            item["value"]
        )

    def depth_value(self):
        if (
            self.depth_m is None
            or
            not self.fresh(
                self.depth_stamp,
                DEPTH_FRESH_S,
            )
        ):
            return None

        return float(
            self.depth_m
        )

    def hazards(self):
        result = []

        for key in ULTRASONIC_TOPICS:
            value = self.ultrasonic_value(
                key
            )

            stop_distance = (
                REAR_STOP_DISTANCE_M
                if key == "rear"
                else STOP_DISTANCE_M
            )

            if (
                value is not None
                and
                value < stop_distance
            ):
                result.append(
                    (
                        key,
                        value,
                    )
                )

        depth = self.depth_value()

        if (
            depth is not None
            and
            depth < STOP_DISTANCE_M
        ):
            result.append(
                (
                    "depth",
                    depth,
                )
            )

        result.sort(
            key=lambda item: item[1]
        )

        return result

    def all_hazards_clear(self):
        # Histereza. Brak pomiaru nie blokuje odblokowania,
        # bo użytkownik wyraźnie chce ignorować znikające HC-SR04.
        for key in ULTRASONIC_TOPICS:
            value = self.ultrasonic_value(
                key
            )

            if (
                value is not None
                and
                value < CLEAR_DISTANCE_M
            ):
                return False

        depth = self.depth_value()

        if (
            depth is not None
            and
            depth < CLEAR_DISTANCE_M
        ):
            return False

        return True

    def front_clear_for_escape(self):
        for key in FRONT_SENSORS:
            value = self.ultrasonic_value(
                key
            )

            if (
                value is not None
                and
                value < STOP_DISTANCE_M
            ):
                return False

        depth = self.depth_value()

        if (
            depth is not None
            and
            depth < STOP_DISTANCE_M
        ):
            return False

        return True

    def rear_clear_for_escape(self):
        value = self.ultrasonic_value(
            "rear"
        )

        return (
            value is None
            or
            value >= STOP_DISTANCE_M
        )

    # ========================================================
    # BEEP / SAFETY
    # ========================================================

    def buzzer_on(self):
        self.send_arduino(
            "7"
        )

    def buzzer_off(self):
        self.send_arduino(
            "8"
        )

    def begin_hazard_response(
        self,
        source,
        distance_m,
    ):
        if (
            not self.enabled
            or
            self.state in (
                BEEPING,
                ESCAPING,
                PICKING,
            )
        ):
            return

        self.hazard_latched = True
        self.hazard_started = self.now()

        self.stop_robot()
        self.set_steering_center()

        if (
            source in FRONT_SENSORS
            or
            source == "depth"
        ):
            # Przód -> cofnięcie, ale tylko jeśli tył nie ma
            # poprawnego pomiaru < 10 cm.
            if self.rear_clear_for_escape():
                self.escape_direction = "reverse"
            else:
                self.escape_direction = None

        elif source == "rear":
            # Tył -> do przodu, ale tylko jeśli przód/depth jest wolny.
            if self.front_clear_for_escape():
                self.escape_direction = "forward"
            else:
                self.escape_direction = None

        else:
            # Czujnik boczny: STOP + 2 piknięcia.
            self.escape_direction = None

        self.escape_source = source

        self.buzzer_on()
        self.set_state(
            BEEPING
        )

        self.publish_status(
            (
                "STOP_PRZESZKODA;"
                f"SRC={source};"
                f"D={distance_m*100.0:.1f}cm"
            )
        )

    def update_beeper(self):
        t = self.elapsed()

        if t < BUZZER_PIP_S:
            self.buzzer_on()
            return

        if t < (
            BUZZER_PIP_S
            +
            BUZZER_GAP_S
        ):
            self.buzzer_off()
            return

        if t < BUZZER_TOTAL_S:
            self.buzzer_on()
            return

        self.buzzer_off()

        if self.escape_direction is None:
            self.stop_robot()
            self.set_state(
                ESCAPING
            )

            self.publish_status(
                "BLOKADA_2S_PO_PRZESZKODZIE"
            )
            return

        if self.escape_direction == "reverse":
            self.send_arduino(
                CMD_REVERSE
            )
        else:
            self.send_arduino(
                CMD_FORWARD
            )

        self.set_state(
            ESCAPING
        )

        self.publish_status(
            (
                "UNIK_1S;"
                f"DIR={self.escape_direction};"
                f"SRC={self.escape_source}"
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

        if command == "start":
            self.enabled = True
            self.hazard_latched = False
            self.hazard_started = None
            self.current_drive_command = None
            self.escape_direction = None
            self.escape_source = None

            self.stop_robot()

            # Platforma w ręcznym trybie sterowania.
            self.send_arduino(
                CMD_MANUAL_MODE
            )

            # Włączenie pomiarów HC-SR04.
            self.send_arduino(
                "@U:1"
            )

            self.set_state(
                READY
            )

            self.publish_status(
                "GOTOWA_PÓŁAUTONOMICZNA"
            )
            return

        if command == "stop":
            self.enabled = False

            self.stop_robot()
            self.buzzer_off()

            self.send_arduino(
                "@U:0"
            )

            self.set_state(
                IDLE
            )

            self.publish_status(
                "STOP"
            )
            return

        if not self.enabled:
            self.publish_status(
                "NAJPIERW_START"
            )
            return

        # W trakcie pracy manipulatora platforma nie może ruszać.
        if self.state == PICKING:
            if command == "0":
                self.stop_robot()

            self.publish_status(
                "PICK_W_TRAKCIE"
            )
            return

        if command == "pick":
            self.stop_robot()

            pick = String()
            pick.data = "full"

            self.pick_pub.publish(
                pick
            )

            self.set_state(
                PICKING
            )

            self.publish_status(
                "PODNOSZENIE_KOSTKI"
            )
            return

        # Safety ma pierwszeństwo.
        if self.state in (
            BEEPING,
            ESCAPING,
        ):
            if command == "0":
                self.stop_robot()

            return

        # Blokada po alarmie trwa tylko 2 sekundy.
        if (
            self.hazard_latched
            and
            command in (
                CMD_FORWARD,
                CMD_REVERSE,
            )
        ):
            if (
                self.hazard_started is not None
                and
                (
                    self.now() - self.hazard_started
                ).nanoseconds / 1e9
                <
                SAFETY_BLOCK_TIME_S
            ):
                self.stop_robot()
                self.publish_status(
                    "BLOKADA_2S"
                )
                return

            self.hazard_latched = False
            self.hazard_started = None

        if command not in PASSTHROUGH_COMMANDS:
            self.get_logger().warn(
                f"Nieznana komenda półautonomiczna: {command}"
            )
            return

        # Przed rozpoczęciem ruchu sprawdzamy aktualny kierunek.
        if command == CMD_FORWARD:
            front_hazard = [
                item
                for item in self.hazards()
                if (
                    item[0] in FRONT_SENSORS
                    or
                    item[0] == "depth"
                )
            ]

            if front_hazard:
                source, distance_m = front_hazard[0]
                self.begin_hazard_response(
                    source,
                    distance_m,
                )
                return

        if command == CMD_REVERSE:
            rear = self.ultrasonic_value(
                "rear"
            )

            if (
                rear is not None
                and
                rear < STOP_DISTANCE_M
            ):
                self.begin_hazard_response(
                    "rear",
                    rear,
                )
                return

        self.send_arduino(
            command
        )

        if command in (
            CMD_FORWARD,
            CMD_REVERSE,
        ):
            self.current_drive_command = command
        elif command == CMD_STOP:
            self.current_drive_command = None

    # ========================================================
    # PICK STATUS
    # ========================================================

    def pick_status_callback(
        self,
        msg,
    ):
        if self.state != PICKING:
            return

        status = (
            msg.data
            .strip()
            .lower()
        )

        if status == "done:full":
            self.stop_robot()

            self.set_state(
                READY
            )

            self.publish_status(
                "PICK_DONE"
            )
            return

        if status == "error:full":
            self.stop_robot()

            self.set_state(
                READY
            )

            self.publish_status(
                "PICK_ERROR"
            )
            return

    # ========================================================
    # UPDATE
    # ========================================================

    def update(self):
        if not self.enabled:
            return

        if self.state == PICKING:
            return

        if self.state == BEEPING:
            self.update_beeper()
            return

        if self.state == ESCAPING:
            # Unik automatyczny trwa maksymalnie 1 sekundę.
            escape_time_s = (
                REAR_ESCAPE_TIME_S
                if self.escape_source == "rear"
                else AUTO_ESCAPE_TIME_S
            )

            if (
                self.escape_direction is not None
                and
                self.elapsed() >= escape_time_s
            ):
                self.stop_robot()

            # Łączna blokada bezpieczeństwa trwa 2 sekundy.
            if (
                self.hazard_started is not None
                and
                (
                    self.now() - self.hazard_started
                ).nanoseconds / 1e9
                >=
                SAFETY_BLOCK_TIME_S
            ):
                self.stop_robot()
                self.hazard_latched = False
                self.hazard_started = None
                self.escape_direction = None
                self.escape_source = None

                self.set_state(
                    READY
                )

                self.publish_status(
                    "ODBLOKOWANO_PO_2S"
                )

            return

        # Automatyczny alarm odpalamy tylko, gdy robot faktycznie jedzie.
        # Dzięki temu stojąc przy przeszkodzie nie pika w pętli co 2 sekundy.
        if self.state == READY:
            hazard_list = self.hazards()

            if self.current_drive_command == CMD_FORWARD:
                hazard_list = [
                    item
                    for item in hazard_list
                    if (
                        item[0] in FRONT_SENSORS
                        or
                        item[0] == "depth"
                    )
                ]

            elif self.current_drive_command == CMD_REVERSE:
                hazard_list = [
                    item
                    for item in hazard_list
                    if item[0] == "rear"
                ]

            else:
                hazard_list = []

            if hazard_list:
                source, distance_m = hazard_list[0]

                self.begin_hazard_response(
                    source,
                    distance_m,
                )


def main(args=None):
    rclpy.init(
        args=args
    )

    node = SemiAutonomousDriveNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.buzzer_off()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()