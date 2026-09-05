#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


# ============================================================
# CLEANUP MISSION
# ============================================================
#
# TEN NODE NIE LICZY POZYCJI CHWYTU I NIE STERUJE RAMIENIEM.
#
# JAZDA NIE KORZYSTA Z ARUCO.
#
# Podczas jazdy platforma korzysta z:
#
#   /ai_cube/detected
#   /ai_cube/position_camera_fast
#   /stereo/center_distance
#
# ArUco jest używane dopiero BLISKO kostki, tuż przed podnoszeniem,
# jako finalne źródło dla gotowego cube_pick_node:
#
#   /cube_aruco/detected
#   /cube_aruco/position_camera
#   /cube_aruco/id
#   /cube_aruco/yaw_deg
#   /cube_aruco/quality
#
# Po dojechaniu cleanup wysyła TYLKO:
#
#   /trunk_loading/command = "start"
#
# Dalej istniejące nody robią wszystko same:
#
#   trunk_loading_node
#       -> cube_pick_node  "full"
#       -> cube_place_node "start"
#
# Cleanup NIE publikuje:
#   /manipulator/cartesian_target
#   /manipulator/cartesian_jog
#   /cube_pick/command
#   /cube_place/command
#   żadnych jointów
#   żadnych własnych offsetów XYZ
#
# ============================================================


# ============================================================
# ZAKRES STARTU PODNOSZENIA
# ============================================================
#
# Zachowane z przesłanego cleanup_mission_node.
#
# Z to DOKŁADNIE:
#   msg.point.z z /cube_aruco/position_camera
#
# ============================================================

PICK_START_MIN_Z_M = 0.0
PICK_START_MAX_Z_M = 0.33

# AI prowadzi robota aż do właściwego zakresu startu podnoszenia.
#
# Cleanup NIE czeka już na /cube_aruco/position_camera.
# Gdy AI Z wejdzie w zakres PICK, platforma staje i uruchamia
# gotowy trunk_loading_node.
#
# To cube_pick_node ma pełną odpowiedzialność za:
#   - ustawienie kamery,
#   - świeże ArUco XYZ,
#   - yaw,
#   - quality,
#   - chwyt.
#
# Dzięki temu cleanup nie blokuje się na starym ArUco.
ARUCO_HANDOFF_Z_M = PICK_START_MAX_Z_M


# ============================================================
# KAMERA
# ============================================================

CAMERA_SERVO = 1

CAMERA_SWITCH_Z_M = 0.45

CAMERA_FAR_ANGLE_DEG = 140.0
CAMERA_NEAR_ANGLE_DEG = 152.0

CAMERA_COMMAND_SETTLE_S = 0.35


# ============================================================
# CZASY JAZDY
# ============================================================
#
# Zachowane DOKŁADNIE z przesłanego przez Ciebie pliku:
#
#   >= 0.60 m       -> 1.05 s
#   0.55 .. 0.60 m  -> 0.95 s
#   < 0.55 m        -> 0.85 s
#
# ============================================================

DRIVE_BURST_FAR_S = 1.05
DRIVE_BURST_NEAR_S = 0.95
DRIVE_BURST_CLOSE_S = 0.85

DRIVE_CLOSE_Z_M = 0.55

MEASURE_WAIT_TIME_S = 3.5


# ============================================================
# SKRĘT KÓŁ
# ============================================================
#
# Kąt do środka jest liczony WYŁĄCZNIE z publikowanego
# /cube_aruco/position_camera:
#
#   bearing = atan2(-point.x, point.z)
#
# WAŻNE:
# /cube_aruco/position_camera zostaje BEZ ZMIAN dla cube_pick.
# Odwracamy znak X TYLKO tutaj, przy interpretacji lewo/prawo
# dla platformy, ponieważ fizyczna kamera jest obrócona o 180°.
#
# Nie jest to yaw kostki.
#
# /cube_aruco/yaw_deg to ORIENTACJA kostki używana przez J5
# w cube_pick_node i cleanup jej do jazdy NIE używa.
#
# ============================================================

STEERING_SERVO = 2

STEERING_RIGHT_MAX_DEG = 80.0
STEERING_RIGHT_HALF_DEG = 85.0

STEERING_CENTER_DEG = 95.0

STEERING_LEFT_HALF_DEG = 110.0
STEERING_LEFT_MAX_DEG = 115.0

STEERING_CENTER_LIMIT_DEG = 6.5
STEERING_MAX_LIMIT_DEG = 10.5

STEERING_SETTLE_TIME_S = 0.35


# ============================================================
# NAPĘD
# ============================================================

DRIVE_PWM = 255

TRAPEZOID_RAMP_FRACTION = 0.22
TRAPEZOID_PWM_LOW = 120
TRAPEZOID_PWM_MID = 190
TRAPEZOID_PWM_MAX = 255


# ============================================================
# ŚWIEŻOŚĆ DANYCH ARUCO
# ============================================================
#
# position_camera z ArUco jest już stabilizowane przez detector.
# Cleanup dodatkowo pilnuje tylko, żeby nie jechać na STARYM
# komunikacie po utracie kostki.
#
# ============================================================

ARUCO_POSITION_TIMEOUT_S = 0.90


# ============================================================
# PO ZATRZYMANIU PRZED PICK
# ============================================================
#
# Po wejściu w 0.285..0.315:
#   STOP
#   kamera 148
#   krótki postój
#   ponowne sprawdzenie ŚWIEŻEGO ArUco Z
#   trunk_loading start
#
# cube_pick po otrzymaniu "full" i tak ponownie ustawia kamerę
# na 148 i zbiera własne świeże, stabilne próbki.
#
# ============================================================

PICK_STOP_SETTLE_S = 0.80


# ============================================================
# WIELE KOSTEK
# ============================================================

TARGET_CUBE_COUNT = 4
POST_LOAD_WAIT_S = 1.00

# ============================================================
# WERYFIKACJA CHWYTU W POLOWIE DROGI DO BAGAZNIKA
# ============================================================
#
# cube_place zatrzymuje sie w P2 i trunk_loading publikuje:
#     check:pickup
#
# Cleanup przez chwile patrzy AI, czy kostka nadal lezy przed robotem.
#
PICK_VERIFY_TIME_S = 1.50
PICK_VERIFY_MIN_Z_M = 0.20
PICK_VERIFY_MAX_Z_M = 0.55

# ============================================================
# SZUKANIE KOLEJNEJ KOSTKI PO ZALADOWANIU
# ============================================================
#
# 1. max lewo + cofanie 0.5 s
# 2. STOP i pomiar
# 3. max prawo + jazda do przodu 0.5 s
# 4. STOP i pomiar
# 5. powtarzaj az AI zobaczy kostke
#
SEARCH_REVERSE_TIME_S = 0.50
SEARCH_FORWARD_TIME_S = 0.50
SEARCH_MEASURE_TIME_S = 0.45


# ============================================================
# TOPIKI
# ============================================================

COMMAND_TOPIC = "/cleanup_mission/command"
STATUS_TOPIC = "/cleanup_mission/status"
COUNT_TOPIC = "/cleanup_mission/cubes_loaded"

ARDUINO_COMMAND_TOPIC = "/manipulator/command"

# TO SĄ DOKŁADNIE TOPIKI UŻYWANE / PUBLIKOWANE PRZEZ
# działający tor ArUco -> cube_pick.
ARUCO_DETECTED_TOPIC = "/cube_aruco/detected"
ARUCO_POSITION_TOPIC = "/cube_aruco/position_camera"
ARUCO_ID_TOPIC = "/cube_aruco/id"
ARUCO_YAW_TOPIC = "/cube_aruco/yaw_deg"
ARUCO_QUALITY_TOPIC = "/cube_aruco/quality"

# ============================================================
# JAZDA: AI + DEPTH
# ============================================================

AI_DETECTED_TOPIC = "/ai_cube/detected"
AI_POSITION_TOPIC = "/ai_cube/position_camera_fast"

# Stereo depth centralny. To jest tylko dodatkowy warunek bezpieczeństwa
# dla jazdy do przodu - nie używamy go do lokalizacji samej kostki.
DEPTH_CENTER_TOPIC = "/stereo/center_distance"

# Jeśli środek depth pokaże przeszkodę bliżej niż ten próg,
# nie zaczynamy kolejnego burstu do przodu.
DEPTH_STOP_M = 0.22

# Świeżość danych AI podczas jazdy.
AI_POSITION_TIMEOUT_S = 0.90

# Gotowa sekwencja cube_pick -> cube_place.
TRUNK_LOADING_COMMAND_TOPIC = "/trunk_loading/command"
TRUNK_LOADING_STATUS_TOPIC = "/trunk_loading/status"

# Statusy tylko do logowania diagnostycznego.
# Nie sterujemy nimi bezpośrednio, bo trunk_loading już to robi.
CUBE_PICK_STATUS_TOPIC = "/cube_pick/status"
CUBE_PLACE_STATUS_TOPIC = "/cube_place/status"


# ============================================================
# KOMENDY ARDUINO
# ============================================================

CMD_AUTO = "A"
CMD_MANUAL = "M"

CMD_FORWARD = "1"
CMD_REVERSE = "2"
CMD_STOP = "0"
CMD_HARD_STOP = "@E"


# ============================================================
# STANY
# ============================================================

IDLE = 0
WAIT_FOR_CUBE = 1
STEERING_SETTLE = 2
DRIVE_BURST = 3
MEASURE_WAIT = 4
PICK_SETTLE = 5
WAIT_TRUNK_LOADING = 6
VERIFY_PICKUP = 7
POST_LOAD = 8
SEARCH_STEER = 9
SEARCH_REVERSE = 10
SEARCH_FORWARD = 11
SEARCH_MEASURE = 12
DONE = 13
ERROR = 14


STATE_NAMES = {
    IDLE: "IDLE",
    WAIT_FOR_CUBE: "WAIT_FOR_CUBE",
    STEERING_SETTLE: "STEERING_SETTLE",
    DRIVE_BURST: "DRIVE_BURST",
    MEASURE_WAIT: "MEASURE_WAIT",
    PICK_SETTLE: "PICK_SETTLE",
    WAIT_TRUNK_LOADING: "WAIT_TRUNK_LOADING",
    VERIFY_PICKUP: "VERIFY_PICKUP",
    POST_LOAD: "POST_LOAD",
    SEARCH_STEER: "SEARCH_STEER",
    SEARCH_REVERSE: "SEARCH_REVERSE",
    SEARCH_FORWARD: "SEARCH_FORWARD",
    SEARCH_MEASURE: "SEARCH_MEASURE",
    DONE: "DONE",
    ERROR: "ERROR",
}


class CleanupMissionNode(Node):

    def __init__(self):

        super().__init__(
            "cleanup_mission_node"
        )

        # ====================================================
        # STAN MISJI
        # ====================================================

        self.enabled = False

        self.state = IDLE
        self.state_started = self.get_clock().now()

        self.cubes_loaded = 0

        # ID kostki, która była celem w chwili startu trunk_loading.
        self.pick_target_id = None

        # ====================================================
        # OSTATNIE DANE Z GOTOWEGO ARUCO DETECTOR
        # ====================================================

        self.aruco_detected = False

        self.cube_x = None
        self.cube_y = None
        self.cube_z = None

        self.cube_id = None
        self.cube_yaw_deg = None
        self.cube_quality = None

        self.last_aruco_position_time = None

        # ====================================================
        # AI + DEPTH DO JAZDY
        # ====================================================

        self.ai_detected = False

        self.ai_x = None
        self.ai_y = None
        self.ai_z = None

        self.last_ai_position_time = None

        self.depth_center_m = None

        # ====================================================
        # JAZDA
        # ====================================================

        self.current_burst_time_s = DRIVE_BURST_FAR_S

        self.selected_steering_deg = STEERING_CENTER_DEG
        self.selected_steering_label = "CENTER"
        self.selected_bearing_deg = 0.0

        self.last_burst_pwm = None

        # Szukanie kolejnej kostki po zaladowaniu.
        self.search_left_next = True

        # Czy podczas okna weryfikacji po picku zobaczylismy kostke na ziemi.
        self.pick_verify_cube_seen = False

        # Zapobiega spamowaniu 0/@E co 20 ms.
        self.last_motion_command = None

        # Zapobiega spamowaniu tego samego kąta serwa.
        self.last_steering_command_deg = None
        self.last_camera_command_deg = None

        # Gdy podczas podejścia kamera raz zejdzie na NEAR (152°),
        # pozostaje na dole aż do potwierdzenia, że kostka została podniesiona.
        self.camera_near_latched = False

        # ====================================================
        # STATUS GOTOWYCH NODÓW
        # ====================================================

        self.trunk_status = "idle"
        self.cube_pick_status = "idle"
        self.cube_place_status = "idle"

        # ====================================================
        # PUBLISHERS
        # ====================================================

        self.arduino_pub = self.create_publisher(
            String,
            ARDUINO_COMMAND_TOPIC,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            10,
        )

        self.count_pub = self.create_publisher(
            Int32,
            COUNT_TOPIC,
            10,
        )

        self.trunk_loading_pub = self.create_publisher(
            String,
            TRUNK_LOADING_COMMAND_TOPIC,
            10,
        )

        # ====================================================
        # SUBSCRIPTIONS
        # ====================================================

        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            10,
        )

        self.create_subscription(
            Bool,
            ARUCO_DETECTED_TOPIC,
            self.aruco_detected_callback,
            20,
        )

        self.create_subscription(
            PointStamped,
            ARUCO_POSITION_TOPIC,
            self.aruco_position_callback,
            20,
        )

        self.create_subscription(
            Int32,
            ARUCO_ID_TOPIC,
            self.aruco_id_callback,
            20,
        )

        self.create_subscription(
            Float32,
            ARUCO_YAW_TOPIC,
            self.aruco_yaw_callback,
            20,
        )

        self.create_subscription(
            Float32,
            ARUCO_QUALITY_TOPIC,
            self.aruco_quality_callback,
            20,
        )

        # Jazda: AI + depth.
        self.create_subscription(
            Bool,
            AI_DETECTED_TOPIC,
            self.ai_detected_callback,
            20,
        )

        self.create_subscription(
            PointStamped,
            AI_POSITION_TOPIC,
            self.ai_position_callback,
            20,
        )

        self.create_subscription(
            Float32,
            DEPTH_CENTER_TOPIC,
            self.depth_center_callback,
            20,
        )

        self.create_subscription(
            String,
            TRUNK_LOADING_STATUS_TOPIC,
            self.trunk_loading_status_callback,
            20,
        )

        self.create_subscription(
            String,
            CUBE_PICK_STATUS_TOPIC,
            self.cube_pick_status_callback,
            20,
        )

        self.create_subscription(
            String,
            CUBE_PLACE_STATUS_TOPIC,
            self.cube_place_status_callback,
            20,
        )

        # ====================================================
        # TIMER
        # ====================================================

        self.timer = self.create_timer(
            0.02,
            self.update,
        )

        # ====================================================
        # STARTOWE STATUSY
        # ====================================================

        self.publish_count()
        self.publish_status("STOP")

        self.get_logger().info(
            "=========================================="
        )
        self.get_logger().info(
            "CLEANUP MISSION - AI+DEPTH DRIVE / ARUCO PICK"
        )
        self.get_logger().info(
            f"JAZDA AI+STEREO XYZ: {AI_POSITION_TOPIC}"
        )
        self.get_logger().info(
            f"JAZDA DEPTH: {DEPTH_CENTER_TOPIC}"
        )
        self.get_logger().info(
            "ARUCO: uzywa go TYLKO gotowy cube_pick po zatrzymaniu platformy"
        )
        self.get_logger().info(
            f"ARUCO XYZ: {ARUCO_POSITION_TOPIC}"
        )
        self.get_logger().info(
            "JAZDA: bearing = atan2(-ArUco_X, ArUco_Z)"
        )
        self.get_logger().info(
            f"ARUCO ID : {ARUCO_ID_TOPIC}"
        )
        self.get_logger().info(
            f"ARUCO YAW: {ARUCO_YAW_TOPIC} (tylko diagnostyka; cube_pick uzywa do J5)"
        )
        self.get_logger().info(
            f"PICK RANGE Z = {PICK_START_MIN_Z_M:.3f} .. {PICK_START_MAX_Z_M:.3f} m"
        )
        self.get_logger().info(
            "PICK/PLACE: TYLKO /trunk_loading/command=start"
        )
        self.get_logger().info(
            "Cleanup NIE publikuje zadnych targetow manipulatora."
        )
        self.get_logger().info(
            "=========================================="
        )

    # ========================================================
    # CZAS / STATUS
    # ========================================================

    def now(self):
        return self.get_clock().now()

    def state_time(self):
        return (
            self.now()
            -
            self.state_started
        ).nanoseconds / 1e9

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)

    def publish_count(self):
        msg = Int32()
        msg.data = int(self.cubes_loaded)
        self.count_pub.publish(msg)

    def set_state(self, new_state, status=None):
        old_name = STATE_NAMES.get(self.state, str(self.state))
        new_name = STATE_NAMES.get(new_state, str(new_state))

        self.state = new_state
        self.state_started = self.now()

        if status is None:
            status = new_name

        self.publish_status(status)

        self.get_logger().info(
            f"STATE {old_name} -> {new_name} | {status}"
        )

    # ========================================================
    # WYSYŁANIE KOMEND PLATFORMY
    # ========================================================

    def send_arduino(self, command):
        msg = String()
        msg.data = str(command)
        self.arduino_pub.publish(msg)

    def set_camera_angle(self, angle_deg):
        angle_deg = float(angle_deg)

        if (
            self.last_camera_command_deg is not None
            and
            abs(
                self.last_camera_command_deg
                -
                angle_deg
            )
            < 0.1
        ):
            return

        self.last_camera_command_deg = angle_deg

        self.send_arduino(
            f"@S{CAMERA_SERVO}:{angle_deg:.0f}"
        )

        self.get_logger().info(
            f"CAMERA S{CAMERA_SERVO} -> {angle_deg:.0f} deg"
        )

    def set_steering_angle(self, angle_deg):
        angle_deg = float(angle_deg)

        if (
            self.last_steering_command_deg is not None
            and
            abs(
                self.last_steering_command_deg
                -
                angle_deg
            )
            < 0.1
        ):
            return

        self.last_steering_command_deg = angle_deg

        self.send_arduino(
            f"@S{STEERING_SERVO}:{angle_deg:.0f}"
        )

        self.get_logger().info(
            f"STEERING S{STEERING_SERVO} -> {angle_deg:.0f} deg"
        )

    def stop_robot(self, hard=False):
        """
        Wysyła STOP tylko przy zmianie stanu ruchu.
        Dzięki temu nie ma lawiny @E / 0 co 20 ms.
        """

        desired = (
            CMD_HARD_STOP
            if hard
            else CMD_STOP
        )

        if self.last_motion_command == desired:
            return

        self.send_arduino(desired)
        self.last_motion_command = desired

        self.last_burst_pwm = None

    def set_drive_pwm(self, pwm):
        pwm = int(
            max(
                0,
                min(
                    255,
                    pwm,
                ),
            )
        )

        if self.last_burst_pwm == pwm:
            return

        self.last_burst_pwm = pwm

        self.send_arduino(
            f"@D:{pwm}"
        )

    def start_forward(self):
        if self.last_motion_command == CMD_FORWARD:
            return

        self.send_arduino(
            CMD_FORWARD
        )

        self.last_motion_command = CMD_FORWARD

    def start_reverse(self):
        if self.last_motion_command == CMD_REVERSE:
            return

        self.send_arduino(
            CMD_REVERSE
        )

        self.last_motion_command = CMD_REVERSE

    # ========================================================
    # AI + DEPTH - TYLKO JAZDA PLATFORMY
    # ========================================================

    def ai_detected_callback(self, msg):
        self.ai_detected = bool(
            msg.data
        )

    def ai_position_callback(self, msg):
        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)

        if (
            not math.isfinite(x)
            or
            not math.isfinite(y)
            or
            not math.isfinite(z)
            or
            z <= 0.0
            or
            z > 5.0
        ):
            return

        self.ai_x = x
        self.ai_y = y
        self.ai_z = z
        self.last_ai_position_time = self.now()

        # Kamera w czasie jazdy wg odległości z AI.
        if (
            self.enabled
            and
            self.state
            not in (
                PICK_SETTLE,
                WAIT_TRUNK_LOADING,
                VERIFY_PICKUP,
                POST_LOAD,
                SEARCH_STEER,
                SEARCH_REVERSE,
                SEARCH_FORWARD,
                SEARCH_MEASURE,
                DONE,
                ERROR,
            )
        ):
            self.update_camera_for_distance(z)

    def depth_center_callback(self, msg):
        value = float(msg.data)

        if not math.isfinite(value):
            return

        if value <= 0.0 or value > 10.0:
            return

        self.depth_center_m = value

    def ai_position_is_fresh(self):
        if self.last_ai_position_time is None:
            return False

        age = (
            self.now()
            -
            self.last_ai_position_time
        ).nanoseconds / 1e9

        return (
            self.ai_detected
            and
            age <= AI_POSITION_TIMEOUT_S
            and
            self.ai_x is not None
            and
            self.ai_y is not None
            and
            self.ai_z is not None
        )

    def depth_allows_forward(self):
        """
        Depth nie wskazuje celu.
        To tylko prosty warunek bezpieczeństwa na środku obrazu.
        """

        if self.depth_center_m is None:
            return True

        return self.depth_center_m > DEPTH_STOP_M


    # ========================================================
    # DANE ARUCO - DOKŁADNIE TE SAME, KTÓRE CZYTA CUBE_PICK
    # ========================================================

    def aruco_detected_callback(self, msg):
        self.aruco_detected = bool(
            msg.data
        )

    def aruco_position_callback(self, msg):
        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)

        if (
            not math.isfinite(x)
            or
            not math.isfinite(y)
            or
            not math.isfinite(z)
            or
            z <= 0.0
            or
            z > 5.0
        ):
            return

        self.cube_x = x
        self.cube_y = y
        self.cube_z = z

        self.last_aruco_position_time = self.now()

        # Cleanup nie rusza kamerą podczas pracy cube_pick/cube_place.
        if (
            self.enabled
            and
            self.state
            not in (
                PICK_SETTLE,
                WAIT_TRUNK_LOADING,
                POST_LOAD,
                DONE,
                ERROR,
            )
        ):
            self.update_camera_for_distance(z)

    def aruco_id_callback(self, msg):
        self.cube_id = int(
            msg.data
        )

    def aruco_yaw_callback(self, msg):
        value = float(
            msg.data
        )

        if math.isfinite(value):
            self.cube_yaw_deg = value

    def aruco_quality_callback(self, msg):
        value = float(
            msg.data
        )

        if math.isfinite(value):
            self.cube_quality = value

    def target_position_is_fresh(self):
        if self.last_aruco_position_time is None:
            return False

        age = (
            self.now()
            -
            self.last_aruco_position_time
        ).nanoseconds / 1e9

        return (
            age
            <=
            ARUCO_POSITION_TIMEOUT_S
            and
            self.cube_x is not None
            and
            self.cube_y is not None
            and
            self.cube_z is not None
        )

    # ========================================================
    # KĄT KOSTKI WZGLĘDEM ŚRODKA KAMERY
    # ========================================================

    def cube_bearing_deg(self):
        """
        Kąt używany WYŁĄCZNIE do jazdy platformy.

        Źródło:
            /ai_cube/position_camera_fast

        AI już pracuje na obrazie obróconym do poprawnej orientacji,
        więc NIE odwracamy tu znaku X drugi raz.

            bearing = atan2(AI_X, AI_Z)
        """

        if (
            self.ai_x is None
            or
            self.ai_z is None
        ):
            return 0.0

        return math.degrees(
            math.atan2(
                float(self.ai_x),
                max(
                    0.001,
                    float(self.ai_z),
                ),
            )
        )


    # ========================================================
    # KAMERA WG ODLEGŁOŚCI CELU (AI W JAZDZIE / ARUCO PRZED PICK)
    # ========================================================

    def update_camera_for_distance(self, z):
        """
        Kamera podczas podejścia:
        - powyżej CAMERA_SWITCH_Z_M może być na FAR,
        - po pierwszym zejściu poniżej progu ustawia NEAR=152°,
        - od tej chwili NIE podnosi się ponownie,
        - odblokowanie następuje dopiero po pozytywnej weryfikacji picku.
        """

        if self.camera_near_latched:
            self.set_camera_angle(
                CAMERA_NEAR_ANGLE_DEG
            )
            return

        if z <= CAMERA_SWITCH_Z_M:
            self.camera_near_latched = True

            self.get_logger().info(
                "CAMERA LATCH: kostka blisko -> kamera zostaje na dole "
                f"{CAMERA_NEAR_ANGLE_DEG:.0f} deg az do poprawnego PICK."
            )

            self.set_camera_angle(
                CAMERA_NEAR_ANGLE_DEG
            )
            return

        self.set_camera_angle(
            CAMERA_FAR_ANGLE_DEG
        )

    # ========================================================
    # CZAS BURSTU WG AI Z
    # ========================================================

    def choose_burst_time_from_distance(self, z):
        if z < DRIVE_CLOSE_Z_M:
            return DRIVE_BURST_CLOSE_S

        if z < CAMERA_SWITCH_Z_M:
            return DRIVE_BURST_NEAR_S

        return DRIVE_BURST_FAR_S

    # ========================================================
    # SKRĘT WG AI X/Z
    # ========================================================

    def choose_steering_from_target(self):
        bearing = self.cube_bearing_deg()

        if (
            -STEERING_CENTER_LIMIT_DEG
            <= bearing
            <= STEERING_CENTER_LIMIT_DEG
        ):
            steering = STEERING_CENTER_DEG
            label = "CENTER"

        elif bearing > STEERING_CENTER_LIMIT_DEG:
            if bearing <= STEERING_MAX_LIMIT_DEG:
                steering = STEERING_RIGHT_HALF_DEG
                label = "RIGHT"
            else:
                steering = STEERING_RIGHT_MAX_DEG
                label = "RIGHT_MAX"

        else:
            if bearing >= -STEERING_MAX_LIMIT_DEG:
                steering = STEERING_LEFT_HALF_DEG
                label = "LEFT"
            else:
                steering = STEERING_LEFT_MAX_DEG
                label = "LEFT_MAX"

        return (
            float(steering),
            label,
            float(bearing),
        )

    # ========================================================
    # PLAN JEDNEGO BURSTU
    # ========================================================

    def prepare_next_burst(self, z):
        (
            steering,
            label,
            bearing,
        ) = self.choose_steering_from_target()

        self.selected_steering_deg = steering
        self.selected_steering_label = label
        self.selected_bearing_deg = bearing

        self.current_burst_time_s = (
            self.choose_burst_time_from_distance(
                z
            )
        )

        # JEDNA decyzja skrętu na jedno zatrzymanie.
        self.set_steering_angle(
            steering
        )

        self.get_logger().info(
            "TARGET PLAN | "
            f"ID={self.cube_id} | "
            f"AI_XYZ=({self.ai_x:+.4f},"
            f"{self.ai_y:+.4f},"
            f"{self.ai_z:+.4f}) | "
            f"bearing={bearing:+.2f} deg | "
            f"{label} -> S2={steering:.0f} | "
            f"burst={self.current_burst_time_s:.2f}s"
        )

        self.set_state(
            STEERING_SETTLE,
            (
                f"USTAWIAM_SKRET;"
                f"ID={self.cube_id};"
                f"Z={z:.3f};"
                f"A={bearing:+.2f};"
                f"STEER={label}"
            ),
        )

    # ========================================================
    # TRAPEZ
    # ========================================================

    def start_drive_burst(self):
        self.last_burst_pwm = None

        self.set_drive_pwm(
            TRAPEZOID_PWM_LOW
        )

        self.start_forward()

        self.set_state(
            DRIVE_BURST,
            (
                f"JAZDA;"
                f"ID={self.cube_id};"
                f"Z={self.ai_z:.3f};"
                f"A={self.selected_bearing_deg:+.2f};"
                f"STEER={self.selected_steering_label};"
                f"T={self.current_burst_time_s:.2f}"
            ),
        )

    def update_trapezoid_pwm(self):
        t = self.state_time()
        total = self.current_burst_time_s

        step = max(
            0.10,
            total * TRAPEZOID_RAMP_FRACTION,
        )

        if t < step:
            pwm = TRAPEZOID_PWM_LOW

        elif t < 2.0 * step:
            pwm = TRAPEZOID_PWM_MID

        elif t >= total - step:
            pwm = TRAPEZOID_PWM_LOW

        elif t >= total - 2.0 * step:
            pwm = TRAPEZOID_PWM_MID

        else:
            pwm = TRAPEZOID_PWM_MAX

        self.set_drive_pwm(
            pwm
        )

    # ========================================================
    # ZAKRES PICK
    # ========================================================

    @staticmethod
    def distance_action(z):
        if (
            PICK_START_MIN_Z_M
            <= z
            <= PICK_START_MAX_Z_M
        ):
            return "pick"

        if z < PICK_START_MIN_Z_M:
            return "too_close"

        return "drive"

    # ========================================================
    # START PICK - TYLKO GOTOWY TRUNK_LOADING
    # ========================================================

    def enter_pick_settle(self, z):
        """
        Zatrzymujemy TYLKO platformę.
        Nie ruszamy ramieniem.

        Po krótkim postoju przekazujemy sterowanie do trunk_loading.
        Cleanup nie sprawdza ArUco.
        """

        self.stop_robot(
            hard=True
        )

        # Kamera dokładnie do kąta, który cube_pick i tak ustawi na starcie.
        self.set_camera_angle(
            CAMERA_NEAR_ANGLE_DEG
        )

        self.get_logger().info(
            "PICK RANGE ENTER | "
            f"ID={self.cube_id} | "
            f"XYZ=({self.cube_x:+.4f},"
            f"{self.cube_y:+.4f},"
            f"{self.cube_z:+.4f}) | "
            f"yaw={self.cube_yaw_deg} | "
            f"quality={self.cube_quality}"
        )

        self.set_state(
            PICK_SETTLE,
            (
                f"PICK_SETTLE;"
                f"ID={self.cube_id};"
                f"Z={z:.3f}"
            ),
        )

    def start_trunk_loading(self):
        """
        JEDYNA komenda dotycząca manipulatora wysyłana przez cleanup.

        Ważne:
        cleanup dojeżdża po AI, więc w chwili startu trunk_loading
        ArUco XYZ może jeszcze być None. To jest normalne.
        cube_pick_node sam czeka na świeże ArUco.
        """

        self.stop_robot(
            hard=True
        )

        self.pick_target_id = self.cube_id

        msg = String()
        msg.data = "start"

        self.trunk_loading_pub.publish(
            msg
        )

        ai_z_text = (
            f"{self.ai_z:.3f}"
            if self.ai_z is not None
            else "None"
        )

        aruco_text = (
            f"({self.cube_x:+.4f},{self.cube_y:+.4f},{self.cube_z:+.4f})"
            if (
                self.cube_x is not None
                and self.cube_y is not None
                and self.cube_z is not None
            )
            else "(brak swiezego ArUco - czeka cube_pick)"
        )

        self.get_logger().info(
            "START GOTOWEGO TRUNK_LOADING | "
            f"ID={self.pick_target_id} | "
            f"AI_Z={ai_z_text} | "
            f"ArUco XYZ={aruco_text}"
        )

        self.set_state(
            WAIT_TRUNK_LOADING,
            (
                f"TRUNK_START;"
                f"ID={self.pick_target_id};"
                f"AI_Z={ai_z_text}"
            ),
        )

    def send_trunk_command(self, command):
        msg = String()
        msg.data = str(command)
        self.trunk_loading_pub.publish(
            msg
        )

        self.get_logger().info(
            f"CLEANUP -> TRUNK: {command}"
        )

    # ========================================================
    # STATUSY GOTOWYCH NODÓW
    # ========================================================

    def cube_pick_status_callback(self, msg):
        self.cube_pick_status = (
            msg.data
            .strip()
            .lower()
        )

        if self.state == WAIT_TRUNK_LOADING:
            self.get_logger().info(
                f"CUBE_PICK STATUS -> {self.cube_pick_status}"
            )

    def cube_place_status_callback(self, msg):
        self.cube_place_status = (
            msg.data
            .strip()
            .lower()
        )

        if self.state == WAIT_TRUNK_LOADING:
            self.get_logger().info(
                f"CUBE_PLACE STATUS -> {self.cube_place_status}"
            )

    def trunk_loading_status_callback(self, msg):
        status = (
            msg.data
            .strip()
            .lower()
        )

        self.trunk_status = status

        if self.state != WAIT_TRUNK_LOADING:
            return

        self.get_logger().info(
            f"TRUNK STATUS -> {status}"
        )

        if status in (
            "running:pick",
            "running:pick_retry",
            "pick_done",
            "running:place",
            "running:abort_to_ready",
            "retry:ready",
        ):
            self.publish_status(
                (
                    f"TRUNK={status};"
                    f"ID={self.pick_target_id};"
                    f"COUNT={self.cubes_loaded}"
                )
            )
            return

        if status == "check:pickup":
            # P2: ramię zatrzymane w połowie drogi do bagażnika.
            # Teraz patrzymy WYŁĄCZNIE AI, czy kostka nadal leży przed robotem.
            self.stop_robot(
                hard=True
            )

            self.pick_verify_cube_seen = False

            # Wymuszamy świeże dane po wejściu w check.
            self.last_ai_position_time = None
            self.ai_x = None
            self.ai_y = None
            self.ai_z = None

            self.set_state(
                VERIFY_PICKUP,
                (
                    f"SPRAWDZAM_CZY_KOSTKA_ZOSTALA;"
                    f"ID={self.pick_target_id}"
                ),
            )
            return

        if status == "done":
            self.cubes_loaded += 1
            self.publish_count()

            self.stop_robot(
                hard=True
            )

            self.get_logger().info(
                "KOSTKA ZAKONCZONA PRZEZ TRUNK_LOADING | "
                f"ID={self.pick_target_id} | "
                f"COUNT={self.cubes_loaded}"
            )

            if self.cubes_loaded >= TARGET_CUBE_COUNT:
                self.enabled = False

                self.set_camera_angle(
                    CAMERA_FAR_ANGLE_DEG
                )

                self.set_steering_angle(
                    STEERING_CENTER_DEG
                )

                self.send_arduino(
                    CMD_MANUAL
                )

                self.set_state(
                    DONE,
                    (
                        f"DONE;"
                        f"COUNT={self.cubes_loaded}"
                    ),
                )

                return

            # Nie używamy starego ArUco punktu po odłożeniu kostki.
            self.last_aruco_position_time = None
            self.cube_x = None
            self.cube_y = None
            self.cube_z = None
            self.cube_id = None
            self.cube_yaw_deg = None
            self.cube_quality = None

            self.last_ai_position_time = None
            self.ai_x = None
            self.ai_y = None
            self.ai_z = None

            self.set_camera_angle(
                CAMERA_FAR_ANGLE_DEG
            )

            self.set_steering_angle(
                STEERING_CENTER_DEG
            )

            self.set_state(
                POST_LOAD,
                (
                    f"ZALADOWANO;"
                    f"COUNT={self.cubes_loaded}"
                ),
            )

            return

        if status.startswith("error"):
            self.stop_robot(
                hard=True
            )

            # Misja ma próbować podnosić kostkę do skutku.
            # Nie kończymy całej misji po pojedynczym błędzie pick/place.
            self.get_logger().warn(
                f"TRUNK ERROR -> RETRY: {status}"
            )

            self.publish_status(
                f"RETRY_PO_BLEDZIE;{status}"
            )

            # Krótkie ponowne uruchomienie całej sekwencji trunk.
            # trunk_loading/cube_pick same ustawiają ramię i świeże ArUco.
            self.start_trunk_loading()
            return

    # ========================================================
    # KOMENDY MISJI
    # ========================================================

    def command_callback(self, msg):
        command = (
            msg.data
            .strip()
            .lower()
        )

        if command == "start":
            self.enabled = True
            self.cubes_loaded = 0
            self.publish_count()

            self.pick_target_id = None
            self.search_left_next = True
            self.pick_verify_cube_seen = False
            self.camera_near_latched = False

            self.stop_robot(
                hard=True
            )

            self.send_arduino(
                CMD_AUTO
            )

            # Domyślna pełna moc pozostaje 255.
            self.send_arduino(
                f"@D:{DRIVE_PWM}"
            )

            # Podczas jazdy używamy WYŁĄCZNIE AI + depth.
            if self.ai_position_is_fresh():
                z = float(
                    self.ai_z
                )

                self.update_camera_for_distance(
                    z
                )

                # Gdy AI jest już bardzo blisko zakresu pick,
                # zatrzymujemy platformę i przechodzimy na ARUCO.
                if z <= ARUCO_HANDOFF_Z_M:
                    self.stop_robot(
                        hard=True
                    )

                    self.set_state(
                        PICK_SETTLE,
                        f"PICK_AI_RANGE;AI_Z={z:.3f}",
                    )

                elif not self.depth_allows_forward():
                    self.set_state(
                        WAIT_FOR_CUBE,
                        f"DEPTH_STOP;D={self.depth_center_m:.3f}",
                    )

                else:
                    self.prepare_next_burst(
                        z
                    )

                return

            # Bez świeżego AI robot nie jedzie.
            self.set_camera_angle(
                CAMERA_FAR_ANGLE_DEG
            )

            self.set_steering_angle(
                STEERING_CENTER_DEG
            )

            self.set_state(
                WAIT_FOR_CUBE,
                "CZEKA_NA_AI",
            )

            return

        if command == "stop":
            self.enabled = False

            self.stop_robot(
                hard=True
            )

            self.send_arduino(
                CMD_MANUAL
            )

            self.set_state(
                IDLE,
                "STOP",
            )

            return

        if command == "reset_count":
            self.cubes_loaded = 0
            self.publish_count()

            self.publish_status(
                "COUNT_RESET"
            )

            return

        if command == "status":
            self.publish_status(
                (
                    f"STATE={STATE_NAMES.get(self.state)};"
                    f"COUNT={self.cubes_loaded};"
                    f"AI_Z={self.ai_z};"
                    f"DEPTH={self.depth_center_m};"
                    f"ARUCO_ID={self.cube_id};"
                    f"ARUCO_Z={self.cube_z};"
                    f"PICK={self.cube_pick_status};"
                    f"PLACE={self.cube_place_status};"
                    f"TRUNK={self.trunk_status}"
                )
            )

            return

        self.get_logger().warn(
            f"Nieznana komenda cleanup: {command}"
        )

    # ========================================================
    # GŁÓWNA LOGIKA
    # ========================================================

    def update(self):

        if not self.enabled:
            return

        # ====================================================
        # CZEKA NA ŚWIEŻE ARUCO
        # ====================================================

        if self.state == WAIT_FOR_CUBE:
            self.stop_robot(
                hard=False
            )

            if not self.ai_position_is_fresh():
                self.publish_status(
                    "CZEKA_NA_AI"
                )
                return

            z = float(
                self.ai_z
            )

            self.update_camera_for_distance(
                z
            )

            # AI doprowadziło nas blisko kostki.
            # Od tego miejsca platforma stoi i czekamy na świeże ArUco,
            # którego używa cube_pick.
            if z <= ARUCO_HANDOFF_Z_M:
                self.set_state(
                    PICK_SETTLE,
                    f"PICK_AI_RANGE;AI_Z={z:.3f}",
                )
                return

            if not self.depth_allows_forward():
                self.publish_status(
                    f"DEPTH_STOP;D={self.depth_center_m:.3f}"
                )
                return

            self.prepare_next_burst(
                z
            )
            return

        # ====================================================
        # SERWO KIEROWNICY DOSTAŁO JEDNĄ KOMENDĘ
        # ====================================================

        if self.state == STEERING_SETTLE:
            if self.state_time() < STEERING_SETTLE_TIME_S:
                return

            if not self.ai_position_is_fresh():
                self.stop_robot(
                    hard=True
                )
                self.set_state(
                    WAIT_FOR_CUBE,
                    "CZEKA_NA_AI",
                )
                return

            z = float(
                self.ai_z
            )

            if z <= ARUCO_HANDOFF_Z_M:
                self.stop_robot(
                    hard=True
                )
                self.set_state(
                    PICK_SETTLE,
                    f"PICK_AI_RANGE;AI_Z={z:.3f}",
                )
                return

            if not self.depth_allows_forward():
                self.stop_robot(
                    hard=True
                )
                self.set_state(
                    WAIT_FOR_CUBE,
                    f"DEPTH_STOP;D={self.depth_center_m:.3f}",
                )
                return

            self.start_drive_burst()
            return

        # ====================================================
        # JAZDA
        # ====================================================

        if self.state == DRIVE_BURST:
            self.update_trapezoid_pwm()

            # Depth jako prosty STOP bezpieczeństwa.
            if not self.depth_allows_forward():
                self.stop_robot(
                    hard=True
                )
                self.set_state(
                    WAIT_FOR_CUBE,
                    f"DEPTH_STOP;D={self.depth_center_m:.3f}",
                )
                return

            # Podczas jazdy śledzimy TYLKO AI.
            if self.ai_position_is_fresh():
                z = float(
                    self.ai_z
                )

                if z <= ARUCO_HANDOFF_Z_M:
                    self.stop_robot(
                        hard=True
                    )
                    self.set_state(
                        PICK_SETTLE,
                        f"PICK_AI_RANGE;AI_Z={z:.3f}",
                    )
                    return

            if self.state_time() >= self.current_burst_time_s:
                self.stop_robot(
                    hard=False
                )

                self.set_state(
                    MEASURE_WAIT,
                    "POSTOJ_POMIAR_AI_DEPTH",
                )

            return

        # ====================================================
        # POSTÓJ / NOWY POMIAR
        # ====================================================

        if self.state == MEASURE_WAIT:
            if self.state_time() < MEASURE_WAIT_TIME_S:
                return

            if not self.ai_position_is_fresh():
                self.set_state(
                    WAIT_FOR_CUBE,
                    "CZEKA_NA_AI",
                )
                return

            z = float(
                self.ai_z
            )

            self.update_camera_for_distance(
                z
            )

            if z <= ARUCO_HANDOFF_Z_M:
                self.set_state(
                    PICK_SETTLE,
                    f"PICK_AI_RANGE;AI_Z={z:.3f}",
                )
                return

            if not self.depth_allows_forward():
                self.set_state(
                    WAIT_FOR_CUBE,
                    f"DEPTH_STOP;D={self.depth_center_m:.3f}",
                )
                return

            self.prepare_next_burst(
                z
            )
            return

        # ====================================================
        # STOP PRZED PICK + PONOWNE SPRAWDZENIE ARUCO
        # ====================================================

        if self.state == PICK_SETTLE:
            # =================================================
            # FINALNY STOP PRZED GOTOWYM CUBE_PICK
            # =================================================
            #
            # Cleanup NIE sprawdza ArUco.
            #
            # Logi pokazały ważną rzecz:
            # /cube_aruco/detected może być True, ale stabilne
            # /cube_aruco/position_camera może mieć kilkadziesiąt sekund.
            #
            # Dlatego cleanup nie może blokować startu na "fresh ArUco".
            # Od ArUco jest gotowy cube_pick_node.
            # =================================================

            self.stop_robot(
                hard=True
            )

            # Ustawiamy kamerę w dół przed przekazaniem kontroli.
            # cube_pick i tak ponownie ustawi swój wymagany kąt.
            self.set_camera_angle(
                CAMERA_NEAR_ANGLE_DEG
            )

            if self.state_time() < PICK_STOP_SETTLE_S:
                return

            # Jeśli AI nadal jest świeże i odskoczyło ponad zakres,
            # wracamy do jazdy zamiast rozpoczynać pick za daleko.
            if self.ai_position_is_fresh():

                ai_z = float(
                    self.ai_z
                )

                if ai_z > PICK_START_MAX_Z_M:

                    self.get_logger().info(
                        "AI po stopie poza PICK RANGE -> wracam do jazdy | "
                        f"AI_Z={ai_z:.3f}"
                    )

                    self.set_state(
                        WAIT_FOR_CUBE,
                        f"AI_ZA_DALEKO_PO_STOP;Z={ai_z:.3f}",
                    )

                    return

                if ai_z < PICK_START_MIN_Z_M:

                    # Za blisko: nie jedziemy dalej.
                    self.publish_status(
                        f"AI_ZA_BLISKO_PRZED_PICK;Z={ai_z:.3f}"
                    )

                    return

            # Tu kończy się odpowiedzialność cleanup.
            #
            # trunk_loading:
            #   -> cube_pick full
            #      -> cube_pick sam czeka na własne świeże ArUco
            #   -> cube_place start
            self.start_trunk_loading()
            return

        # ====================================================
        # TRUNK_LOADING MA PEŁNĄ KONTROLĘ NAD RAMIENIEM
        # ====================================================

        if self.state == WAIT_TRUNK_LOADING:
            # Trunk/cube_pick/cube_place sterują ramieniem.
            # Cleanup czeka na status, w tym "check:pickup".
            return

        # ====================================================
        # WERYFIKACJA CZY PICK NAPRAWDE PODNIOSL KOSTKE
        # ====================================================

        if self.state == VERIFY_PICKUP:
            self.stop_robot(
                hard=True
            )

            # Jeśli w czasie okna pojawi się świeża kostka w typowym
            # zakresie przed robotem, traktujemy to jako nieudany chwyt.
            if self.ai_position_is_fresh():
                verify_z = float(
                    self.ai_z
                )

                if (
                    PICK_VERIFY_MIN_Z_M
                    <= verify_z
                    <= PICK_VERIFY_MAX_Z_M
                ):
                    self.pick_verify_cube_seen = True

                    self.get_logger().warn(
                        "PICK VERIFY: AI nadal widzi kostke przed robotem | "
                        f"Z={verify_z:.3f} -> RETRY"
                    )

            if self.state_time() < PICK_VERIFY_TIME_S:
                return

            if self.pick_verify_cube_seen:
                # Kostka nadal na ziemi -> kamera ma zostać na 152°.
                self.camera_near_latched = True

                self.set_camera_angle(
                    CAMERA_NEAR_ANGLE_DEG
                )

                self.send_trunk_command(
                    "pickup_failed"
                )

                self.set_state(
                    WAIT_TRUNK_LOADING,
                    "PICK_FAIL_RETRY",
                )

            else:
                self.get_logger().info(
                    "PICK VERIFY: kostki nie ma na ziemi -> PICK POTWIERDZONY."
                )

                # Dopiero teraz wolno podnieść kamerę.
                self.camera_near_latched = False

                self.set_camera_angle(
                    CAMERA_FAR_ANGLE_DEG
                )

                self.get_logger().info(
                    "CAMERA UNLATCH: kostka podniesiona i nie lezy na ziemi -> kamera FAR."
                )

                self.send_trunk_command(
                    "pickup_ok"
                )

                self.set_state(
                    WAIT_TRUNK_LOADING,
                    "PICK_OK_CONTINUE_PLACE",
                )

            return

        # ====================================================
        # PO ZAŁADOWANIU - CZEKAJ I SZUKAJ KOLEJNEJ
        # ====================================================

        if self.state == POST_LOAD:
            if self.state_time() < POST_LOAD_WAIT_S:
                return

            # Po każdej poprawnie załadowanej kostce robimy wymuszony skan:
            # 1) max lewo + cofanie 0.5 s
            # 2) pomiar
            # 3) max prawo + jazda do przodu 0.5 s
            # 4) pomiar
            # 5) powtarzamy do znalezienia kostki.
            #
            # Czyścimy stare AI, żeby nie zareagować na klatkę sprzed manewru.
            self.last_ai_position_time = None
            self.ai_x = None
            self.ai_y = None
            self.ai_z = None

            self.search_left_next = True

            self.set_state(
                SEARCH_STEER,
                (
                    f"SZUKANIE_START_LEWO;"
                    f"COUNT={self.cubes_loaded}"
                ),
            )
            return

        # ====================================================
        # SZUKANIE KOLEJNEJ KOSTKI: COFANIE LEWO/PRAWO
        # ====================================================

        if self.state == SEARCH_STEER:
            self.stop_robot(
                hard=True
            )

            # Jeżeli już mamy świeżą kostkę, wracamy do normalnego dojazdu.
            if self.ai_position_is_fresh():
                z = float(
                    self.ai_z
                )
                self.prepare_next_burst(
                    z
                )
                return

            if self.search_left_next:
                # FAZA A:
                # max lewo + cofanie 0.5 s
                self.set_steering_angle(
                    STEERING_LEFT_MAX_DEG
                )
            else:
                # FAZA B:
                # max prawo + jazda do przodu 0.5 s
                self.set_steering_angle(
                    STEERING_RIGHT_MAX_DEG
                )

            self.set_drive_pwm(
                DRIVE_PWM
            )

            if self.state_time() < STEERING_SETTLE_TIME_S:
                return

            if self.search_left_next:
                self.start_reverse()

                self.set_state(
                    SEARCH_REVERSE,
                    "SKAN_COFAJ_LEWO_0.5S",
                )
            else:
                self.start_forward()

                self.set_state(
                    SEARCH_FORWARD,
                    "SKAN_PRZOD_PRAWO_0.5S",
                )

            return

        if self.state == SEARCH_REVERSE:
            # Skanujemy także podczas ruchu.
            if self.ai_position_is_fresh():
                self.stop_robot(
                    hard=True
                )

                z = float(
                    self.ai_z
                )

                self.prepare_next_burst(
                    z
                )
                return

            if self.state_time() >= SEARCH_REVERSE_TIME_S:
                self.stop_robot(
                    hard=True
                )

                # Po cofnięciu wymuszamy nowy pomiar.
                self.last_ai_position_time = None
                self.ai_x = None
                self.ai_y = None
                self.ai_z = None

                self.set_state(
                    SEARCH_MEASURE,
                    "POMIAR_PO_COFAJ_LEWO",
                )

            return

        if self.state == SEARCH_FORWARD:
            # Skanujemy także podczas ruchu.
            if self.ai_position_is_fresh():
                self.stop_robot(
                    hard=True
                )

                z = float(
                    self.ai_z
                )

                self.prepare_next_burst(
                    z
                )
                return

            if self.state_time() >= SEARCH_FORWARD_TIME_S:
                self.stop_robot(
                    hard=True
                )

                # Po jeździe do przodu wymuszamy nowy pomiar.
                self.last_ai_position_time = None
                self.ai_x = None
                self.ai_y = None
                self.ai_z = None

                self.set_state(
                    SEARCH_MEASURE,
                    "POMIAR_PO_PRZOD_PRAWO",
                )

            return

        if self.state == SEARCH_MEASURE:
            if self.ai_position_is_fresh():
                z = float(
                    self.ai_z
                )

                self.get_logger().info(
                    "SZUKANIE: znaleziono kostke -> normalny dojazd AI."
                )

                self.prepare_next_burst(
                    z
                )
                return

            if self.state_time() < SEARCH_MEASURE_TIME_S:
                return

            # Brak kostki:
            # po LEWO/cofaniu przechodzimy na PRAWO/przód,
            # po PRAWO/przód wracamy na LEWO/cofanie.
            self.search_left_next = (
                not self.search_left_next
            )

            self.set_state(
                SEARCH_STEER,
                (
                    "SZUKANIE_NASTEPNY_RUCH_"
                    +
                    (
                        "COFAJ_LEWO"
                        if self.search_left_next
                        else
                        "PRZOD_PRAWO"
                    )
                ),
            )
            return

        # ====================================================
        # KONIEC / BŁĄD
        # ====================================================

        if self.state in (
            DONE,
            ERROR,
        ):
            return


def main(args=None):

    rclpy.init(
        args=args
    )

    node = CleanupMissionNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot(
            hard=True
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()