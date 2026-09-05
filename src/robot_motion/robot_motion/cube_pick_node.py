# ROBUST PICK V5: fresh stable XYZ + SIGNED yaw [-45,+45), freeze before motion
# ROBUST J5 CALIBRATION FROM MEASURED PAIRS; YAW-Y OFFSET DISABLED
# PREGRASP PICK: camera->grasp calibration, pregrasp, J5 yaw, vertical descend, close, lift
import math
import os
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float32

from robot_motion.arm_kinematics import (
    ArmKinematics,
    JOINT_NAMES,
    apply_radial_tangential_offset,
)


# Cube pick korzysta tylko z danych publikowanych przez ArUco detector.
ARUCO_DETECTED_TOPIC = "/cube_aruco/detected"
ARUCO_RESET_TOPIC = "/cube_aruco/reset"
CAMERA_CUBE_TOPIC = "/cube_aruco/position_camera"
CARTESIAN_TARGET_TOPIC = "/manipulator/cartesian_target"
ARDUINO_COMMAND_TOPIC = "/manipulator/command"
PICK_COMMAND_TOPIC = "/cube_pick/command"
PICK_STATUS_TOPIC = "/cube_pick/status"
CUBE_YAW_TOPIC = "/cube_aruco/yaw_deg"
CUBE_YAW_QUALITY_TOPIC = "/cube_aruco/quality"

CALIBRATION_FILE = os.path.expanduser("~/robot_ws/cube_grasp_calibration.yaml")

CAMERA_SERVO = 1
CAMERA_PICK_ANGLE = 152.0       # pozycja kamery do CUBE PICK
GRIPPER_SERVO = 5
GRIPPER_OPEN_ANGLE = 54.0
GRIPPER_CLOSE_ANGLE = 70.0

# ============================================================
# J5 - ORIENTACJA CHWYTAKA
#
# cube_yaw_deg pochodzi bezposrednio z detektora ArUco i jest normalizowany
# to [-45, +45) because the cube is square.
#
# J5 reference below is the physical gripper angle that corresponds
# to cube_yaw_deg = 0. Start from the currently known nominal J5 = 82 deg.
# joint_5 w /joint_states jest tutaj traktowany bezposrednio w radianach ROS.
# Dla yaw ~= 0 target J5 powinien byc blisko aktualnego J5, bez duzego obrotu.
# ============================================================

# ============================================================
# J5 / SERWO 4 - NOWA KALIBRACJA
#
# NOWE, fizyczne granice:
#   MIN    = 52 deg
#   CENTER = 82 deg
#   MAX    = 112 deg
#
# Z nowych pomiarow wybieramy jedna SPOJNA, monotoniczna galaz:
#
#   ArUco yaw =  -6.545587 deg -> Serwo 4 = 82 deg  (srodek)
#   ArUco yaw = +23.093916 deg -> Serwo 4 = 52 deg  (min)
#   ArUco yaw = -30.609472 deg -> Serwo 4 = 112 deg (max)
#
# Pozostale podane pary reprezentuja inna/equivalentna galaz chwytu
# kwadratowej kostki i nie daja jednej funkcji yaw->servo
# (np. podobny yaw dawal bardzo rozne serwo). Nie mieszamy galezi.
#
# Funkcja ponizej liczy signed odchylenie ArUco modulo 90 stopni
# wzgledem yaw srodkowego i interpoluje osobno lewa/prawa strone.
# ============================================================

J5_SERVO_MIN_DEG = 52.0
J5_SERVO_CENTER_DEG = 82.0
J5_SERVO_MAX_DEG = 112.0

J5_SETTLE_TIME = 0.20
J5_MOVE_DEADBAND_DEG = 1.0
J5_MECHANICAL_TRIM_DEG = 0.0


def wrap_square_delta_deg(angle_deg):
    """Yaw kwadratowej kostki, wynik zawsze [-45,+45)."""
    return ((float(angle_deg) + 45.0) % 90.0) - 45.0


# ============================================================
# NOWA KALIBRACJA Z 8 NOWYCH PRÓBEK DLA KAMERY 152 DEG - KAMERA 152°
#
# Każdy punkt:
#   CAMERA_ARUCO XYZ
#   poprawne ręczne joint_states chwytu
#   ArUco yaw
#
# Q[:,0:4] służy do wyznaczania pozycji TCP.
# J5 wyznaczamy z CAMERA XYZ + yaw, bo sam yaw nie wystarcza
# dla wszystkich równoważnych orientacji kwadratowej kostki.
# ============================================================

CAL_CAMERA_POINTS = np.array(
    [
        [+0.0226576985976470, +0.0130055598942216, +0.3315756916999817],
        [+0.0176769960058039, +0.0124081185300722, +0.3256423026323318],
        [+0.0295085043737816, +0.0134290158001019, +0.3328921198844910],
        [-0.0538708515154030, +0.0121475934418191, +0.3309212476015091],
        [-0.0594962801040638, +0.0108261474574936, +0.3277793079614639],
        [+0.0583077568706324, +0.0109607480894421, +0.3278708457946777],
        [+0.0587289871198739, -0.0062857735756052, +0.2998921573162079],
        [-0.0199124895331761, -0.0101685599740095, +0.2989754378795624],
    ],
    dtype=float,
)

CAL_GRASP_Q = np.array(
    [
        [-3.9035799101936637, +3.4115732722576659, +2.5793532952730622, +2.2514747350726849, +1.4137166941154069],
        [-3.9035799101936637, +3.4115732722576659, +2.5793532952730622, +2.2514747350726849, +1.7627825445142729],
        [-3.9035799101936637, +3.4115732722576659, +2.5793532952730622, +2.2514747350726849, +1.0646508437165410],
        [-3.7023216308230675, +3.4115732722576659, +2.5793532952730622, +2.2514747350726849, +1.3264502315156905],
        [-3.7023216308230675, +3.4115732722576659, +2.5793532952730622, +2.2514747350726849, +1.6755160819145565],
        [-4.0217672299741452, +3.4115732722576659, +2.5793532952730622, +2.2514747350726849, +1.9373154697137058],
        [-4.0217672299741452, +3.3809880553176690, +2.7379433090360084, +2.2514747350726849, +1.5707963267948966],
        [-3.7835046140588613, +3.3809880553176690, +2.7379433090360084, +2.2514747350726849, +1.5707963267948966],
    ],
    dtype=float,
)

CAL_YAW_DEG = np.array(
    [
        -7.2639255523681641,
        +42.0751953125000000,
        +29.8728199005126953,
        -5.1013112068176270,
        -43.9704284667968750,
        +14.1246042251586914,
        -0.9757505655288696,
        +7.1320204734802246,
    ],
    dtype=float,
)

# Normalizacja odległości w przestrzeni CAMERA.
CAMERA_DISTANCE_SCALE = np.array(
    [0.040, 0.020, 0.025],
    dtype=float,
)

YAW_DISTANCE_SCALE_DEG = 14.0

XYZ_NEIGHBORS = 3
J5_NEIGHBORS = 3


# ============================================================
# J5 / SERWO 4 - DEDYKOWANA KALIBRACJA YAW -> J5
#
# To są TRZY pomiary wykonane specjalnie dla orientacji chwytaka.
# One są źródłem prawdy dla J5.
#
#   yaw = -30.609472 deg -> J5 = 112 deg
#   yaw =  -6.545587 deg -> J5 =  82 deg
#   yaw = +23.093916 deg -> J5 =  52 deg
#
# Nie używamy J5 z tabeli CAL_GRASP_Q do sterowania orientacją.
# CAL_GRASP_Q służy do pozycji chwytu / interpolacji J1..J4.
# W próbkach pozycyjnych występują równoważne gałęzie chwytu
# kwadratowej kostki, więc mieszanie ich spłaszczało albo skakało J5.
# ============================================================

J5_CAL_LEFT_YAW_DEG = -30.609472
J5_CAL_CENTER_YAW_DEG = -6.545587
J5_CAL_RIGHT_YAW_DEG = +23.093916

J5_CAL_LEFT_SERVO_DEG = 112.0
J5_CAL_CENTER_SERVO_DEG = 82.0
J5_CAL_RIGHT_SERVO_DEG = 52.0


def empirical_j5_from_camera_yaw(camera, yaw_deg):
    """
    Piecewise-linearna, monotoniczna kalibracja ArUco yaw -> fizyczny J5.

    CAMERA jest celowo ignorowane dla ORIENTACJI J5.
    Pozycja kostki wpływa na J1..J4/TCP, ale obrót szczęk ma wynikać
    z yaw markera.

    Najpierw liczymy yaw względem zmierzonego środka (-6.545587 deg)
    z symetrią kwadratu modulo 90 deg. Potem interpolujemy osobno
    lewą i prawą stronę. Po wyjściu poza zmierzony zakres wynik
    dochodzi do mechanicznych 52/112 deg i jest clampowany.
    """
    del camera

    if yaw_deg is None or not np.isfinite(float(yaw_deg)):
        return 0.0, float(J5_CAL_CENTER_SERVO_DEG)

    yaw = wrap_square_delta_deg(float(yaw_deg))

    # Signed delta względem FIZYCZNIE zmierzonego środka J5=82.
    delta = wrap_square_delta_deg(
        yaw - J5_CAL_CENTER_YAW_DEG
    )

    left_delta = wrap_square_delta_deg(
        J5_CAL_LEFT_YAW_DEG
        - J5_CAL_CENTER_YAW_DEG
    )

    right_delta = wrap_square_delta_deg(
        J5_CAL_RIGHT_YAW_DEG
        - J5_CAL_CENTER_YAW_DEG
    )

    if delta <= 0.0:
        # center: delta=0 -> 82 deg
        # left:   delta~= -24.06 -> 112 deg
        denom = abs(left_delta)

        if denom < 1e-9:
            servo_deg = J5_CAL_CENTER_SERVO_DEG
        else:
            alpha = min(
                1.0,
                max(
                    0.0,
                    abs(delta) / denom,
                ),
            )

            servo_deg = (
                J5_CAL_CENTER_SERVO_DEG
                +
                alpha
                *
                (
                    J5_CAL_LEFT_SERVO_DEG
                    -
                    J5_CAL_CENTER_SERVO_DEG
                )
            )

    else:
        # center: delta=0 -> 82 deg
        # right:  delta~= +29.64 -> 52 deg
        denom = abs(right_delta)

        if denom < 1e-9:
            servo_deg = J5_CAL_CENTER_SERVO_DEG
        else:
            alpha = min(
                1.0,
                max(
                    0.0,
                    delta / denom,
                ),
            )

            servo_deg = (
                J5_CAL_CENTER_SERVO_DEG
                +
                alpha
                *
                (
                    J5_CAL_RIGHT_SERVO_DEG
                    -
                    J5_CAL_CENTER_SERVO_DEG
                )
            )

    servo_deg += J5_MECHANICAL_TRIM_DEG

    servo_deg = max(
        J5_SERVO_MIN_DEG,
        min(
            J5_SERVO_MAX_DEG,
            servo_deg,
        ),
    )

    return float(delta), float(servo_deg)

def aruco_yaw_to_servo_deg(
    yaw_deg,
    camera=None,
):
    """
    Zachowujemy nazwę funkcji dla reszty node'a.
    J5 jest liczony wyłącznie z dedykowanej kalibracji ArUco yaw -> J5.
    """
    if camera is None:
        return (
            wrap_square_delta_deg(yaw_deg),
            J5_SERVO_CENTER_DEG,
        )

    return empirical_j5_from_camera_yaw(
        camera,
        yaw_deg,
    )


# ============================================================
# KOMPENSACJA POZYCJI PRZY OBROCIE J5
#
# Z obserwacji:
#   |yaw| = 0 deg  -> brak poprawki
#   |yaw| = 45 deg -> +10 mm w osi Y, czyli dalej od robota.
#
# Uzywamy |yaw|, bo +45 i -45 geometrycznie daja ten sam efekt
# wysuniecia chwytaka. Zmiana jest liniowa pomiedzy 0 i 45 deg.
# Jezeli w Twoim base_link znak Y jest odwrotny, zmien tylko
# YAW_Y_OFFSET_AT_45_M z +0.010 na -0.010.
# ============================================================
YAW_Y_OFFSET_AT_45_M = 0.000

CAMERA_SETTLE_TIME = 0.35
GRIPPER_OPEN_SETTLE_TIME = 0.25
ARUCO_SETTLE_TIME = 0.20
GRIPPER_CLOSE_SETTLE_TIME = 0.55

# /cube_aruco/position_camera and /cube_aruco/yaw_deg are stable ArUco outputs
# from the detector.  The pick node nevertheless collects several fresh
# post-START samples and freezes one final pose before any arm motion.
CUBE_SAMPLE_COUNT = 4
YAW_SAMPLE_COUNT = 4
DETECTION_TIMEOUT = 8.0

# Brak kostki w /cube_aruco/detected po rozpoczeciu zbierania danych.
# Arduino:
#   "7" = buzzer ON
#   "8" = buzzer OFF
NO_CUBE_BEEP_AFTER_S = 3.00
BUZZER_PIP_S = 0.10
BUZZER_GAP_S = 0.12
MAX_CAMERA_SPREAD_M = 0.018
MAX_YAW_SPREAD_DEG = 8.0
MIN_YAW_QUALITY = 0.40

MAX_COMMAND_SEGMENT_M = 0.150
TARGET_TOLERANCE_M = 0.002
MOVE_SEGMENT_TIMEOUT = 8.0
INTER_SEGMENT_DELAY_S = 0.08

PREGRASP_HEIGHT_M = 0.070
LIFT_HEIGHT_M = 0.050

# Po NOWEJ kalibracji camera->grasp te poprawki maja byc ZERO.
# Stare +60 mm/+5 mm byly latami na nieaktualna definicje pomiaru i nie
# powinny byc mieszane z nowym fizycznym srodkiem kostki.
GRASP_RADIAL_OFFSET_M = 0.000
GRASP_RIGHT_OFFSET_M = 0.000

# Stała korekcja po nowej CAL8_152:
# Brak recznego offsetu X.
# Pozycja chwytu wynika tylko z kalibracji CAMERA_ARUCO -> grasp.
GRASP_X_OFFSET_M = 0.0

# WAZNE:
# NIE stosujemy juz "offsetu Y = dalej od robota".
# +Y base_link nie jest fizycznym kierunkiem "dalej".
# Wszystkie korekcje chwytu sa radialne/tangencjalne wzgledem osi J1.

# ============================================================
# CAMERA_ARUCO -> TCP Z 8 NOWYCH PRÓBEK, KAMERA 152°
#
# Nie używamy starej macierzy CURRENT9.
# Dla bieżącego CAMERA XYZ interpolujemy poprawne ręczne joint_states,
# a następnie liczymy TCP przez aktualne ArmKinematics.
# ============================================================

# J4 w najnowszych próbkach NIE jest stałe (119..130°).
# To jest ważne: przed dojazdem ustawiamy J4 na wartość interpolowaną
# z kalibracji i dopiero wtedy wykonujemy PREGRASP + zjazd.
WRIST_J4_CAL_DEG = 130.0
WRIST_J5_HOME_DEG = 82.0

# Odrzucamy kalibracje, ktora sama na probkach byla za slaba.
MAX_ALLOWED_CALIBRATION_RMS_M = 0.015
MAX_ALLOWED_CALIBRATION_MAX_M = 0.030

IDLE = 0
SET_CAMERA = 1
OPEN_GRIPPER = 2
SET_J5_HOME = 3
WAIT_ARUCO_SETTLE = 4
COLLECT_CUBE = 5
MOVE_PREGRASP = 6
ROTATE_J5 = 7
MOVE_TO_GRASP = 8
CLOSE_GRIPPER = 9
MOVE_LIFT = 10
DONE = 11
ERROR = 12


class CubePickNode(Node):
    def __init__(self):
        super().__init__("cube_pick_node")
        self.kin = ArmKinematics()

        self.create_subscription(
            Bool, ARUCO_DETECTED_TOPIC, self.aruco_detected_callback, 20
        )
        self.create_subscription(
            PointStamped, CAMERA_CUBE_TOPIC, self.cube_callback, 20
        )
        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 30
        )
        self.create_subscription(
            String, PICK_COMMAND_TOPIC, self.command_callback, 10
        )

        self.create_subscription(
            Float32, CUBE_YAW_TOPIC, self.yaw_callback, 20
        )
        self.create_subscription(
            Float32, CUBE_YAW_QUALITY_TOPIC, self.yaw_quality_callback, 20
        )

        self.cartesian_pub = self.create_publisher(
            Point, CARTESIAN_TARGET_TOPIC, 10
        )
        self.arduino_pub = self.create_publisher(
            String, ARDUINO_COMMAND_TOPIC, 10
        )
        self.aruco_reset_pub = self.create_publisher(
            String, ARUCO_RESET_TOPIC, 10
        )
        self.status_pub = self.create_publisher(
            String, PICK_STATUS_TOPIC, 10
        )

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self.j5_motion_done = False
        self.j5_motion_started = False

        self.current_q = None

        # Swiezy stan /cube_aruco/detected dla biezacej proby pick.
        self.latest_aruco_detected = None
        self.aruco_detected_fresh = False
        self.no_cube_beep_sent = False
        self._one_shot_timers = []

        self.latest_cube_yaw_deg = None
        self.latest_yaw_quality = None
        self.yaw_samples = deque(maxlen=YAW_SAMPLE_COUNT)
        self.captured_cube_yaw_deg = None
        self.target_j4_deg = None
        self.target_j5_deg = None
        self.calibrated_grasp_q = None
        self.last_wait_yaw_log = None
        self.samples = deque(maxlen=CUBE_SAMPLE_COUNT)

        self.state = IDLE
        self.state_started = self.get_clock().now()
        self.state_command_sent = False
        self.full_pick = False
        self.pick_mode = "start"

        self.matrix = None
        self.offset = None
        self.calibration_rms = None
        self.calibration_max = None

        self.grasp_xyz = None
        self.pregrasp_xyz = None
        self.lift_xyz = None
        self.stage_target_xyz = None
        self.segment_target_xyz = None
        self.segment_started = None
        self.target_stable_counter = 0
        self.next_segment_not_before = None

        self.timer = self.create_timer(0.05, self.update)

        self.load_calibration(log_error=False)

        self.get_logger().info("==========================================")
        self.get_logger().info("CUBE PICK CAL8_152 - CAMERA152 - ARUCO ONLY - NEW POSITION CALIBRATION")
        self.get_logger().info(f"KAMERA: SERVO_1 = {CAMERA_PICK_ANGLE:.0f} deg")
        self.get_logger().info("ARUCO XYZ: /cube_aruco/position_camera -> CAL8_152")
        self.get_logger().info("ARUCO YAW: /cube_aruco/yaw_deg")
        self.get_logger().info(
            "J5 CAL: yaw -30.609->112 | -6.546->82 | +23.094->52 deg"
        )
        self.get_logger().info("BRAK ARUCO XYZ przez 3 s: pip-pip + STOP pick")
        self.get_logger().info("start = PREGRASP + ustawienie J5 i STOP")
        self.get_logger().info("grasp = PREGRASP + J5 + zjazd do GRASP i STOP (bez zamkniecia)")
        self.get_logger().info("full  = PREGRASP + J5 + pionowy zjazd + chwyt + lift")
        self.get_logger().info("XYZ+YAW: biorę świeże dane ArUco i stabilizuję je lokalnie w cube_pick.")
        self.get_logger().info("J4+J5 ustawiane PRZED PREGRASP; potem XYZ jedzie z zamrozonym nadgarstkiem.")
        self.get_logger().info(
            f"GRASP RADIAL TRIM = {GRASP_RADIAL_OFFSET_M*1000:+.1f} mm"
        )
        self.get_logger().info(
            f"J5 MECHANICAL TRIM = {J5_MECHANICAL_TRIM_DEG:+.1f} deg"
        )
        self.get_logger().info(
            "J5 MAP: CAL8_152 CAMERA XYZ + ArUco yaw"
        )
        self.get_logger().info("==========================================")

    def state_time(self):
        return (
            self.get_clock().now() - self.state_started
        ).nanoseconds / 1e9

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)
        self.get_logger().info(f"PICK STATUS -> {msg.data}")

    def set_state(self, state):
        self.state = state
        self.state_started = self.get_clock().now()
        self.state_command_sent = False
        self.target_stable_counter = 0

        if state == DONE:
            self.publish_status(f"done:{self.pick_mode}")
        elif state == ERROR:
            self.publish_status(f"error:{self.pick_mode}")

    def joint_state_callback(self, msg):
        values = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                values[name] = float(pos)
        if len(values) == len(JOINT_NAMES):
            self.current_q = np.array(
                [values[name] for name in JOINT_NAMES], dtype=float
            )

    def current_tcp(self):
        if self.current_q is None:
            return None
        return self.kin.tcp(self.current_q)

    def aruco_detected_callback(self, msg):
        self.latest_aruco_detected = bool(msg.data)

        if self.state == COLLECT_CUBE:
            self.aruco_detected_fresh = True

    def cube_callback(self, msg):
        if self.state != COLLECT_CUBE:
            return
        p = np.array(
            [float(msg.point.x), float(msg.point.y), float(msg.point.z)],
            dtype=float,
        )
        if not np.all(np.isfinite(p)):
            return
        self.samples.append(p)

    def yaw_callback(self, msg):
        value = float(msg.data)
        if not np.isfinite(value):
            return
        # Detector V2 publishes signed square yaw in [-45,+45).
        # Wrap once defensively, but do NOT take abs() and do NOT clamp to 0..45.
        value = ((value + 45.0) % 90.0) - 45.0
        if value < -45.0 or value >= 45.0:
            return
        self.latest_cube_yaw_deg = value
        if self.state == COLLECT_CUBE:
            # ARUCO ONLY: jezeli yaw jest skonczona liczba, przyjmujemy probke.
            # /cube_aruco/quality zostaje tylko do logow diagnostycznych.
            self.yaw_samples.append(value)

    def yaw_quality_callback(self, msg):
        value = float(msg.data)
        if np.isfinite(value):
            self.latest_yaw_quality = value

    def load_calibration(self, log_error=True):
        """
        Aktywuje kalibrację 8 nowych próbek dla kamery 152 deg.
        Kamera podczas CUBE PICK = 152°.
        """
        self.matrix = None
        self.offset = None
        self.calibration_rms = 0.0
        self.calibration_max = 0.0

        self.get_logger().info(
            "ARUCO->GRASP CAL8_152 aktywne | CAMERA_PICK=152 deg."
        )
        # Tylko informacyjnie pokazujemy rozpiętość próbek CAL8_152.
        # NIE jest to żaden limit: każdy skończony XYZ z ArUco jest używany.
        cal_min = np.min(CAL_CAMERA_POINTS, axis=0)
        cal_max = np.max(CAL_CAMERA_POINTS, axis=0)

        self.get_logger().info(
            "CAL8_152 SAMPLE SPAN (NO LIMITS): "
            f"X=[{cal_min[0]:+.4f},{cal_max[0]:+.4f}] "
            f"Y=[{cal_min[1]:+.4f},{cal_max[1]:+.4f}] "
            f"Z=[{cal_min[2]:+.4f},{cal_max[2]:+.4f}]"
        )
        return True

    def interpolate_grasp_q(self, camera):
        """
        Interpoluje ręcznie zmierzoną konfigurację chwytu
        z najbliższych punktów CAMERA XYZ.

        J5 z tej funkcji nie jest używany jako końcowy target yaw;
        służy jedynie do policzenia modelowego TCP.
        """
        camera = np.asarray(
            camera,
            dtype=float,
        )

        delta = (
            CAL_CAMERA_POINTS
            - camera.reshape(1, 3)
        ) / CAMERA_DISTANCE_SCALE.reshape(1, 3)

        dist = np.linalg.norm(
            delta,
            axis=1,
        )

        order = np.argsort(dist)
        use = order[:XYZ_NEIGHBORS]

        if float(dist[use[0]]) < 0.035:
            return CAL_GRASP_Q[
                use[0]
            ].copy()

        d = np.maximum(
            dist[use],
            1e-4,
        )

        weights = 1.0 / (
            d * d
        )

        q = np.sum(
            CAL_GRASP_Q[
                use
            ]
            *
            weights.reshape(-1, 1),
            axis=0,
        ) / np.sum(weights)

        return np.asarray(
            q,
            dtype=float,
        )

    def camera_to_grasp(self, camera):
        camera = np.asarray(
            camera,
            dtype=float,
        )

        if (
            camera.shape != (3,)
            or
            not np.all(
                np.isfinite(camera)
            )
        ):
            raise ValueError(
                "Nieprawidlowe CAMERA_ARUCO."
            )

        # Brak ograniczeń zakresu CAL8_152.
        # Jeżeli ArUco dostarcza skończone XYZ, zawsze próbujemy
        # wyznaczyć konfigurację chwytu z najbliższych punktów kalibracyjnych.
        q_grasp = self.interpolate_grasp_q(
            camera
        )

        # Zapamiętujemy interpolowaną konfigurację z RĘCZNEJ kalibracji.
        # Szczególnie J4 nie może być później na sztywno 130 deg.
        self.calibrated_grasp_q = q_grasp.copy()
        self.target_j4_deg = math.degrees(float(q_grasp[3]))

        grasp = np.asarray(
            self.kin.tcp(
                q_grasp
            ),
            dtype=float,
        )

        if (
            grasp.shape != (3,)
            or
            not np.all(
                np.isfinite(grasp)
            )
        ):
            raise ValueError(
                "ArmKinematics zwrocilo nieprawidlowy TCP."
            )

        self.get_logger().info(
            "CAL8_152 CAMERA->Q->TCP | "
            f"CAM=({camera[0]:+.4f},{camera[1]:+.4f},{camera[2]:+.4f}) | "
            f"Qdeg=["
            f"{math.degrees(q_grasp[0]):+.1f},"
            f"{math.degrees(q_grasp[1]):+.1f},"
            f"{math.degrees(q_grasp[2]):+.1f},"
            f"{math.degrees(q_grasp[3]):+.1f},"
            f"{math.degrees(q_grasp[4]):+.1f}"
            f"] | "
            f"TCP=({grasp[0]:+.4f},{grasp[1]:+.4f},{grasp[2]:+.4f})"
        )

        return grasp

    def stable_camera_position(self):
        if len(self.samples) < CUBE_SAMPLE_COUNT:
            return None

        data = np.asarray(self.samples, dtype=float)
        median = np.median(data, axis=0)
        dist = np.linalg.norm(data - median, axis=1)
        cutoff = max(0.006, float(np.percentile(dist, 75)) * 1.5)
        good = data[dist <= cutoff]

        if good.shape[0] < 3:
            return None

        spread = np.max(good, axis=0) - np.min(good, axis=0)
        if float(np.max(spread)) > MAX_CAMERA_SPREAD_M:
            self.get_logger().warn(
                "Kostka niestabilna w kamerze: "
                f"spread=({spread[0]*1000:.1f},"
                f"{spread[1]*1000:.1f},{spread[2]*1000:.1f}) mm"
            )
            return None

        return np.median(good, axis=0)

    @staticmethod
    def yaw_delta_deg(a, b):
        return ((float(a) - float(b) + 45.0) % 90.0) - 45.0

    def stable_cube_yaw(self):
        if len(self.yaw_samples) < YAW_SAMPLE_COUNT:
            return None

        yaw = np.asarray(self.yaw_samples, dtype=np.float64)

        # Square orientation has period 90 deg. Multiply angle by 4,
        # average on a circle, then divide by 4. This also works correctly
        # across the -45/+45 wrap boundary.
        aa = np.deg2rad(yaw * 4.0)
        center = math.degrees(
            math.atan2(float(np.sum(np.sin(aa))), float(np.sum(np.cos(aa))))
        ) / 4.0
        center = ((center + 45.0) % 90.0) - 45.0

        dev = np.asarray([abs(self.yaw_delta_deg(v, center)) for v in yaw])
        if float(np.max(dev)) > MAX_YAW_SPREAD_DEG:
            self.get_logger().warn(
                f"YAW niestabilny: values={[round(float(v),1) for v in yaw]} "
                f"maxDev={float(np.max(dev)):.1f} deg"
            )
            return None

        # Recalculate from accepted samples (all passed the spread gate).
        return float(center)

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd in ("start", "grasp", "full"):
            if self.state not in (IDLE, DONE, ERROR):
                self.get_logger().warn("Sekwencja juz trwa.")
                return
            if self.current_q is None:
                self.get_logger().debug("Brak /joint_states.")
                return
            if not self.load_calibration(log_error=True):
                return

            self.pick_mode = cmd
            self.full_pick = (cmd == "full")
            self.samples.clear()
            self.yaw_samples.clear()

            self.latest_aruco_detected = None
            self.aruco_detected_fresh = False
            self.no_cube_beep_sent = False
            # Bierzemy TYLKO swieze pomiary po START. Stary yaw/XYZ nie moze
            # zostac przypadkiem zamrozony po przesunieciu kostki.
            self.latest_cube_yaw_deg = None
            self.latest_yaw_quality = None
            self.grasp_xyz = None
            self.pregrasp_xyz = None
            self.lift_xyz = None
            self.captured_cube_yaw_deg = None
            self.target_j4_deg = None
            self.target_j5_deg = None
            self.calibrated_grasp_q = None
            self.last_wait_yaw_log = None
            self.j5_motion_started = False
            self.j5_motion_done = False
            self.stage_target_xyz = None
            self.segment_target_xyz = None
            self.next_segment_not_before = None
            self.set_state(SET_CAMERA)
            mode_label = {
                "start": "PREGRASP TEST",
                "grasp": "GRASP POSITION TEST",
                "full": "FULL PICK",
            }[self.pick_mode]
            self.get_logger().info(f"START {mode_label}")
            self.publish_status(f"running:{self.pick_mode}")
            return

        if cmd == "reset":
            self.set_state(IDLE)
            self.samples.clear()
            self.yaw_samples.clear()
            self.latest_aruco_detected = None
            self.aruco_detected_fresh = False
            self.no_cube_beep_sent = False
            self.latest_cube_yaw_deg = None
            self.latest_yaw_quality = None
            self.get_logger().warn("RESET -> IDLE")
            return

        if cmd == "status":
            tcp = self.current_tcp()
            self.get_logger().info(f"STATE={self.state}")
            if tcp is not None:
                self.get_logger().info(
                    f"MODEL TCP=({tcp[0]:+.4f},{tcp[1]:+.4f},{tcp[2]:+.4f})"
                )
            return

        self.get_logger().warn(f"Nieznana komenda: {cmd}")

    def start_wrist_target(
        self,
        target_j4_deg,
        target_j5_deg,
        label="WRIST",
    ):
        if self.current_q is None:
            self.fail("Brak joint_states do ustawienia nadgarstka.")
            return

        if not self.trajectory_client.server_is_ready():
            self.fail("Brak FollowJointTrajectory do ustawienia nadgarstka.")
            return

        target_j4_deg = max(5.0, min(180.0, float(target_j4_deg)))
        target_j5_deg = max(
            J5_SERVO_MIN_DEG,
            min(J5_SERVO_MAX_DEG, float(target_j5_deg)),
        )

        q = self.current_q.copy()

        current_j4_deg = math.degrees(float(q[3]))
        current_j5_deg = abs(math.degrees(float(q[4])))

        self.get_logger().info(
            f"{label}: "
            f"J4 {current_j4_deg:.2f}->{target_j4_deg:.2f} deg | "
            f"J5 {current_j5_deg:.2f}->{target_j5_deg:.2f} deg"
        )

        if (
            abs(target_j4_deg - current_j4_deg) < 0.8
            and
            abs(target_j5_deg - current_j5_deg) < J5_MOVE_DEADBAND_DEG
        ):
            self.j5_motion_started = True
            self.j5_motion_done = True
            self.get_logger().info(f"{label}: juz ustawione.")
            return

        q[3] = math.radians(target_j4_deg)
        q[4] = math.radians(target_j5_deg)

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 500000000
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.j5_motion_started = True
        self.j5_motion_done = False

        future = self.trajectory_client.send_goal_async(goal)
        future.add_done_callback(self.j5_goal_response)

    def start_j5_target(self, target_servo_deg, label="J5"):
        target_j4 = (
            WRIST_J4_CAL_DEG
            if self.target_j4_deg is None
            else float(self.target_j4_deg)
        )

        self.start_wrist_target(
            target_j4,
            target_servo_deg,
            label=label,
        )

    def start_j5_rotation(self):
        if self.target_j5_deg is None:
            self.fail("Brak target_j5_deg.")
            return

        self.start_j5_target(
            self.target_j5_deg,
            label="ALIGN J5",
        )

    def j5_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.fail(f"J5 goal exception: {exc}")
            return

        if not handle.accepted:
            self.fail("J5 trajectory odrzucona.")
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self.j5_result)

    def j5_result(self, future):
        try:
            wrapped = future.result()

            result_code = int(
                getattr(
                    wrapped.result,
                    "error_code",
                    0,
                )
            )

            self.get_logger().info(
                f"WRIST trajectory zakonczona, "
                f"status={wrapped.status}, error_code={result_code}"
            )

            # action_msgs/GoalStatus: STATUS_SUCCEEDED == 4.
            if wrapped.status != 4 or result_code != 0:
                self.fail(
                    "WRIST nie osiagnal celu - przerywam PICK."
                )
                return

            self.j5_motion_done = True

        except Exception as exc:
            self.fail(f"J5 result exception: {exc}")

    def reset_aruco_history(self):
        msg = String()
        msg.data = "reset"
        self.aruco_reset_pub.publish(msg)
        self.get_logger().info("ARUCO -> reset historii przed pomiarem PICK")

    def send_arduino(self, text):
        msg = String()
        msg.data = text
        self.arduino_pub.publish(msg)
        self.get_logger().info(f"ARDUINO -> {text}")

    def schedule_once(self, delay_s, callback):
        """
        Jednorazowy timer ROS bez blokowania spin().
        """
        holder = {"timer": None}

        def wrapped():
            timer = holder["timer"]

            if timer is not None:
                timer.cancel()

                try:
                    self._one_shot_timers.remove(timer)
                except ValueError:
                    pass

            callback()

        timer = self.create_timer(
            float(delay_s),
            wrapped,
        )

        holder["timer"] = timer
        self._one_shot_timers.append(timer)

        return timer

    def buzzer_double_beep(self):
        """
        Dwa krotkie dzwieki:
          7 -> ON
          8 -> OFF
          7 -> ON
          8 -> OFF
        """
        self.get_logger().warn(
            "BRAK WYKRYCIA KOSTKI ARUCO -> BUZZER: pip pip"
        )

        # pip 1
        self.send_arduino("7")

        self.schedule_once(
            BUZZER_PIP_S,
            lambda: self.send_arduino("8"),
        )

        # pip 2
        self.schedule_once(
            BUZZER_PIP_S + BUZZER_GAP_S,
            lambda: self.send_arduino("7"),
        )

        self.schedule_once(
            2.0 * BUZZER_PIP_S + BUZZER_GAP_S,
            lambda: self.send_arduino("8"),
        )

    def start_stage_move(self, target):
        self.stage_target_xyz = np.asarray(target, dtype=float)
        self.segment_target_xyz = None
        self.next_segment_not_before = None
        self.target_stable_counter = 0
        self.send_next_segment()

    def send_next_segment(self):
        if self.next_segment_not_before is not None:
            if self.get_clock().now() < self.next_segment_not_before:
                return
            self.next_segment_not_before = None

        current = self.current_tcp()
        if current is None:
            self.fail("Brak TCP.")
            return

        delta = self.stage_target_xyz - current
        distance = float(np.linalg.norm(delta))

        if distance <= TARGET_TOLERANCE_M:
            self.segment_target_xyz = None
            self.on_stage_reached()
            return

        step = min(MAX_COMMAND_SEGMENT_M, distance)
        target = current + delta * (step / distance)

        self.segment_target_xyz = target
        self.segment_started = self.get_clock().now()
        self.target_stable_counter = 0

        msg = Point()
        msg.x, msg.y, msg.z = [float(v) for v in target]
        self.cartesian_pub.publish(msg)

        self.get_logger().info(
            f"SEGMENT -> ({target[0]:+.4f},{target[1]:+.4f},{target[2]:+.4f}) "
            f"{step*1000:.1f} mm"
        )

    def update_stage_move(self):
        if self.segment_target_xyz is None:
            if self.next_segment_not_before is not None:
                self.send_next_segment()
            return

        current = self.current_tcp()
        if current is None:
            return

        error = float(np.linalg.norm(self.segment_target_xyz - current))

        if error <= TARGET_TOLERANCE_M:
            self.target_stable_counter += 1
        else:
            self.target_stable_counter = 0

        if self.target_stable_counter >= 2:
            self.segment_target_xyz = None
            self.target_stable_counter = 0
            self.next_segment_not_before = (
                self.get_clock().now()
                + rclpy.duration.Duration(seconds=INTER_SEGMENT_DELAY_S)
            )
            return

        elapsed = (
            self.get_clock().now() - self.segment_started
        ).nanoseconds / 1e9

        if elapsed > MOVE_SEGMENT_TIMEOUT:
            self.fail(
                f"TIMEOUT ruchu: error={error*1000:.1f} mm, "
                f"TCP=({current[0]:+.4f},{current[1]:+.4f},{current[2]:+.4f})"
            )

    def on_stage_reached(self):
        if self.state == MOVE_PREGRASP:
            self.get_logger().info("PREGRASP OSIAGNIETY.")

            if self.pick_mode == "start":
                self.get_logger().info(
                    "TEST START: PREGRASP osiagniety z poprawnym J4+J5, STOP."
                )
                self.set_state(DONE)
            else:
                self.get_logger().info(
                    "Nadgarstek juz ustawiony -> pionowy zjazd do GRASP."
                )
                self.set_state(MOVE_TO_GRASP)
                self.start_stage_move(self.grasp_xyz)

            return

        if self.state == MOVE_TO_GRASP:
            self.get_logger().info("POZYCJA CHWYTU OSIAGNIETA.")

            if self.pick_mode == "grasp":
                self.get_logger().info(
                    "TEST GRASP: pozycja chwytu osiagnieta, szczeki pozostaja OTWARTE, STOP."
                )
                self.set_state(DONE)
            else:
                self.set_state(CLOSE_GRIPPER)

            return

        if self.state == MOVE_LIFT:
            self.get_logger().info("KOSTKA PODNIESIONA.")
            self.set_state(DONE)

    def fail(self, text):
        self.get_logger().error(text)
        self.set_state(ERROR)

    def update(self):
        if self.state in (IDLE, DONE, ERROR):
            return

        if self.state == SET_CAMERA:
            if not self.state_command_sent:
                self.send_arduino(f"@S1:{CAMERA_PICK_ANGLE:.0f}")
                self.state_command_sent = True
            if self.state_time() >= CAMERA_SETTLE_TIME:
                self.set_state(OPEN_GRIPPER)
            return

        if self.state == OPEN_GRIPPER:
            if not self.state_command_sent:
                self.send_arduino(f"@S{GRIPPER_SERVO}:{GRIPPER_OPEN_ANGLE:.0f}")
                self.state_command_sent = True
            if self.state_time() >= GRIPPER_OPEN_SETTLE_TIME:
                self.set_state(SET_J5_HOME)
            return

        if self.state == SET_J5_HOME:
            # ZAWSZE zaczynamy sekwencje z Serwo 4 = 82 deg.
            # Dzieki temu caly dojazd XYZ do PREGRASP odbywa sie
            # z tym samym, znanym ustawieniem nadgarstka.
            if not self.j5_motion_started:
                self.start_wrist_target(
                    WRIST_J4_CAL_DEG,
                    WRIST_J5_HOME_DEG,
                    label="WRIST HOME",
                )
                return

            if self.j5_motion_done:
                self.j5_motion_started = False
                self.j5_motion_done = False
                self.get_logger().info(
                    "WRIST HOME gotowe: J4=130 deg, Serwo 4=82 deg."
                )
                self.set_state(WAIT_ARUCO_SETTLE)
            return

        if self.state == WAIT_ARUCO_SETTLE:
            # Nie resetujemy historii aruco_cube_detector.
            # Detektor publikuje bieżący poprawny XYZ/YAW, a cube_pick
            # zbiera WYŁĄCZNIE własne świeże próbki po wejściu do COLLECT_CUBE.
            if self.state_time() >= ARUCO_SETTLE_TIME:
                self.samples.clear()
                self.yaw_samples.clear()
                self.latest_cube_yaw_deg = None
                self.latest_yaw_quality = None

                # Od tej chwili interesuje nas tylko SWIEZA detekcja ArUco.
                self.latest_aruco_detected = None
                self.aruco_detected_fresh = False

                self.set_state(COLLECT_CUBE)
                self.get_logger().info(
                    "Zbieram swiezy, STABILNY pose kostki (XYZ + signed YAW)..."
                )
            return

        if self.state == COLLECT_CUBE:
            # Źródłem prawdy dla PICK są świeże XYZ/YAW z ArUco.
            # Detektor publikuje teraz każdy poprawny pose, a stabilność
            # sprawdzamy tutaj na kilku świeżych próbkach.
            #
            # Jeśli przez 3 s nie przyszło ANI JEDNO świeże XYZ,
            # robimy pip-pip i kończymy. Robot NIE jedzie do kostki.
            if (
                self.state_time() >= NO_CUBE_BEEP_AFTER_S
                and len(self.samples) == 0
            ):
                if not self.no_cube_beep_sent:
                    self.no_cube_beep_sent = True
                    self.buzzer_double_beep()

                self.get_logger().error(
                    "BRAK SWIEZEGO ARUCO XYZ -> STOP PICK."
                )
                self.publish_status("no_cube")
                self.set_state(ERROR)
                return

            if self.state_time() > DETECTION_TIMEOUT:
                xyz_count = len(self.samples)
                yaw_count = len(self.yaw_samples)
                self.publish_status("no_cube")
                self.fail(
                    "TIMEOUT stabilizacji kostki: "
                    f"XYZ={xyz_count}/{CUBE_SAMPLE_COUNT}, "
                    f"YAW={yaw_count}/{YAW_SAMPLE_COUNT}, "
                    f"yawQ={self.latest_yaw_quality}."
                )
                return

            camera = self.stable_camera_position()
            if camera is None:
                return

            try:
                grasp = self.camera_to_grasp(camera)
            except ValueError as exc:
                self.fail(f"LOCAL CALIBRATION: {exc}")
                return

            # ============================================================
            # Punkt chwytu po przeliczeniu CAMERA_ARUCO -> TCP base_link.
            #
            # NIE dodajemy tutaj surowego offsetu w osi Y.
            # Korekcje typu "dalej od robota / w prawo" sa wykonywane
            # nizej radialnie/tangencjalnie wzgledem osi J1.
            # ============================================================
            grasp = grasp.copy()

            self.get_logger().info(
                "GRASP BEFORE ROBOT OFFSET = "
                f"({grasp[0]:+.4f},{grasp[1]:+.4f},{grasp[2]:+.4f})"
            )

            # ============================================================
            # EMPIRYCZNA KOREKCJA CELU CHWYTU
            # ============================================================
            # Jedyna poprawna semantyka korekcji chwytu:
            #
            #   GRASP_RADIAL_OFFSET_M:
            #       + = dalej od osi J1
            #
            #   GRASP_RIGHT_OFFSET_M:
            #       + = fizycznie w prawo stycznie
            #
            # Nie uzywamy surowego +X/+Y base_link jako
            # "przod/prawo".
            try:
                grasp = apply_radial_tangential_offset(
                    grasp,
                    forward_m=GRASP_RADIAL_OFFSET_M,
                    right_m=GRASP_RIGHT_OFFSET_M,
                    up_m=0.0,
                )
            except ValueError as exc:
                self.fail(
                    f"Nie moge policzyc korekcji chwytu: {exc}"
                )
                return

            self.get_logger().info(
                f"RADIAL OFFSET = "
                f"{GRASP_RADIAL_OFFSET_M*1000:+.1f} mm"
            )

            self.get_logger().info(
                f"RIGHT OFFSET = "
                f"{GRASP_RIGHT_OFFSET_M*1000:+.1f} mm"
            )

            # Stała poprawka osi X z obserwacji po CAL8_152.
            grasp[0] += GRASP_X_OFFSET_M

            self.get_logger().info(
                f"X OFFSET = "
                f"{GRASP_X_OFFSET_M*1000:+.1f} mm"
            )

            self.get_logger().info(
                f"GRASP AFTER ROBOT OFFSET = "
                f"({grasp[0]:+.4f},{grasp[1]:+.4f},{grasp[2]:+.4f})"
            )

            # Freeze signed yaw from several FRESH stable detector outputs.
            yaw_frozen = self.stable_cube_yaw()
            if yaw_frozen is None:
                now = self.get_clock().now()
                if (
                    self.last_wait_yaw_log is None
                    or (now - self.last_wait_yaw_log).nanoseconds / 1e9 > 1.0
                ):
                    self.get_logger().info(
                        f"Czekam na YAW: {len(self.yaw_samples)}/{YAW_SAMPLE_COUNT} "
                        f"quality={self.latest_yaw_quality}"
                    )
                    self.last_wait_yaw_log = now
                return

            self.captured_cube_yaw_deg = float(yaw_frozen)

            # Stara kompensacja globalnego Y od yaw jest celowo wylaczona.
            # Nie mieszamy jej z poprawka radialna/right powyzej.
            yaw_y_offset_m = 0.0
            grasp = grasp.copy()

            self.grasp_xyz = grasp
            self.pregrasp_xyz = (
                grasp
                + np.array([0.0, 0.0, PREGRASP_HEIGHT_M])
            )
            self.lift_xyz = (
                grasp
                + np.array([0.0, 0.0, LIFT_HEIGHT_M])
            )

            # Jedna spojna galaz kalibracji ArUco -> fizyczne Serwo 4.
            yaw_delta_deg, target_j5 = aruco_yaw_to_servo_deg(
                self.captured_cube_yaw_deg,
                camera=camera,
            )

            target_j5 = float(target_j5)

            target_j5 = max(
                J5_SERVO_MIN_DEG,
                min(
                    J5_SERVO_MAX_DEG,
                    target_j5,
                ),
            )

            self.target_j5_deg = float(target_j5)

            if self.target_j4_deg is None:
                self.fail("Brak target_j4_deg z kalibracji.")
                return

            self.get_logger().info("==========================================")
            self.get_logger().info(
                f"CAMERA = ({camera[0]:+.4f},{camera[1]:+.4f},{camera[2]:+.4f})"
            )
            self.get_logger().info(
                f"GRASP  = ({grasp[0]:+.4f},{grasp[1]:+.4f},{grasp[2]:+.4f})"
            )
            self.get_logger().info(
                f"PREGRASP = ({self.pregrasp_xyz[0]:+.4f},"
                f"{self.pregrasp_xyz[1]:+.4f},{self.pregrasp_xyz[2]:+.4f})"
            )
            self.get_logger().info(
                f"YAW SIGNED ROBUST = "
                f"{self.captured_cube_yaw_deg:+.1f} deg "
                f"(samples={[round(float(v),1) for v in self.yaw_samples]})"
            )
            self.get_logger().info(
                f"YAW QUALITY = {self.latest_yaw_quality}"
            )
            self.get_logger().info(
                f"YAW Y OFFSET = {yaw_y_offset_m*1000:+.1f} mm "
                f"(wylaczone: offset Y = 0 mm)"
            )
            self.get_logger().info(
                f"ARUCO YAW RAW = {self.captured_cube_yaw_deg:+.2f} deg"
            )
            self.get_logger().info(
                f"ARUCO YAW DELTA FROM CAL CENTER = {yaw_delta_deg:+.2f} deg"
            )
            self.get_logger().info(
                f"J4 TARGET Z KALIBRACJI = {self.target_j4_deg:.2f} deg"
            )
            self.get_logger().info(
                f"SERWO 4 TARGET = {self.target_j5_deg:.2f} deg "
                f"(HARD LIMIT {J5_SERVO_MIN_DEG:.0f}.."
                f"{J5_SERVO_MAX_DEG:.0f}, CENTER {J5_SERVO_CENTER_DEG:.0f})"
            )
            self.get_logger().info(
                "CAL = CAL8_152 CAMERA152"
            )
            self.get_logger().info("==========================================")

            # KRYTYCZNE:
            # Najpierw ustawiamy J4+J5 z kalibracji, gdy jesteśmy jeszcze z dala
            # od kostki. Dopiero potem jedziemy na PREGRASP. CartesianController
            # zamraża J4/J5, więc cały PREGRASP i finalny zjazd używają dokładnie
            # tej orientacji nadgarstka, dla której wykonano kalibrację.
            self.set_state(ROTATE_J5)
            return

        if self.state == ROTATE_J5:
            if not self.j5_motion_started:
                self.start_j5_rotation()
                return

            if self.j5_motion_done:
                self.j5_motion_started = False
                self.j5_motion_done = False

                self.get_logger().info(
                    "J4+J5 gotowe -> jadę do PREGRASP z zamrożonym nadgarstkiem."
                )
                self.set_state(MOVE_PREGRASP)
                self.start_stage_move(self.pregrasp_xyz)

            return

        if self.state in (MOVE_PREGRASP, MOVE_TO_GRASP, MOVE_LIFT):
            self.update_stage_move()
            return

        if self.state == CLOSE_GRIPPER:
            if not self.state_command_sent:
                self.send_arduino(
                    f"@S{GRIPPER_SERVO}:{GRIPPER_CLOSE_ANGLE:.0f}"
                )
                self.state_command_sent = True
            if self.state_time() >= GRIPPER_CLOSE_SETTLE_TIME:
                self.set_state(MOVE_LIFT)
                self.start_stage_move(self.lift_xyz)
            return


def main(args=None):
    rclpy.init(args=args)
    node = CubePickNode()
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