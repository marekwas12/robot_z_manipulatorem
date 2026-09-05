import os
import time

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import message_filters


# ============================================================
# PLANSZA
#
# 7 x 9 pol
# = 6 x 8 wewnetrznych naroznikow
#
# kwadrat 25 mm
# ============================================================

BOARD_COLS = 6
BOARD_ROWS = 8
SQUARE_SIZE = 0.025


# ============================================================
# KAMERY
# ============================================================

LEFT_TOPIC = "/camera_left/left_camera/image_raw"
RIGHT_TOPIC = "/camera_right/right_camera/image_raw"


# ============================================================
# RZECZYWISTY BASELINE
# ============================================================

EXPECTED_BASELINE_M = 0.060

BASELINE_WARNING_TOLERANCE_M = 0.005


# ============================================================
# ZBIERANIE PROBEK
# ============================================================

TARGET_SAMPLES = 12

STABLE_FRAMES_REQUIRED = 4

STABILITY_THRESHOLD = 0.010

DIVERSITY_THRESHOLD = 0.075

SAMPLE_COOLDOWN_SEC = 1.5


# Plansza musi zajmowac przynajmniej 7% obrazu.
MIN_BOARD_AREA_RATIO = 0.10

MAX_BOARD_AREA_RATIO = 0.65


# ============================================================
# SYNCHRONIZACJA
# ============================================================

SYNC_QUEUE = 30

# Maksymalnie 50 ms roznicy timestampow.
SYNC_SLOP_SEC = 0.10


# ============================================================
# WYJSCIE
# ============================================================

OUTPUT_DIR = os.path.expanduser(
    "~/robot_ws/stereo_calibration_v3"
)

SAMPLES_DIR = os.path.join(
    OUTPUT_DIR,
    "samples"
)


class StereoCalibrator(Node):

    def __init__(self):

        super().__init__(
            "stereo_calibrator_headless"
        )

        self.bridge = CvBridge()

        self.done = False
        self.calibrating = False

        self.image_size = None

        self.object_points = []
        self.left_points = []
        self.right_points = []

        self.accepted_descriptors = []

        self.last_candidate_descriptor = None
        self.stable_counter = 0

        self.last_accept_time = 0.0
        self.last_status_time = 0.0

        os.makedirs(
            OUTPUT_DIR,
            exist_ok=True
        )

        os.makedirs(
            SAMPLES_DIR,
            exist_ok=True
        )


        # ====================================================
        # MODEL 3D PLANSZY
        # ====================================================

        self.object_template = np.zeros(
            (
                BOARD_ROWS * BOARD_COLS,
                3
            ),
            np.float32
        )

        grid = np.mgrid[
            0:BOARD_COLS,
            0:BOARD_ROWS
        ].T.reshape(-1, 2)

        self.object_template[:, :2] = (
            grid * SQUARE_SIZE
        )


        # ====================================================
        # SUBSKRYPCJE
        # ====================================================

        self.left_sub = message_filters.Subscriber(
            self,
            Image,
            LEFT_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )

        self.right_sub = message_filters.Subscriber(
            self,
            Image,
            RIGHT_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )


        self.sync = (
            message_filters.ApproximateTimeSynchronizer(
                [
                    self.left_sub,
                    self.right_sub,
                ],
                queue_size=SYNC_QUEUE,
                slop=SYNC_SLOP_SEC,
            )
        )

        self.sync.registerCallback(
            self.stereo_callback
        )


        self.get_logger().info(
            "================================================"
        )

        self.get_logger().info(
            "STEREO CALIBRATOR V3"
        )

        self.get_logger().info(
            f"Plansza: "
            f"{BOARD_COLS}x{BOARD_ROWS} "
            f"wewnetrznych naroznikow"
        )

        self.get_logger().info(
            f"Kwadrat: "
            f"{SQUARE_SIZE * 1000:.1f} mm"
        )

        self.get_logger().info(
            f"Rzeczywisty baseline: "
            f"{EXPECTED_BASELINE_M * 1000:.1f} mm"
        )

        self.get_logger().info(
            f"Cel: {TARGET_SAMPLES} par"
        )

        self.get_logger().info(
            f"Minimalna powierzchnia planszy: "
            f"{MIN_BOARD_AREA_RATIO * 100:.1f}% obrazu"
        )

        self.get_logger().info(
            f"Max roznica timestampow ATS: "
            f"{SYNC_SLOP_SEC * 1000:.0f} ms"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "USTAW PLANSZE -> TRZYMAJ NIERUCHOMO -> "
            "CZEKAJ NA AKCEPTACJE"
        )

        self.get_logger().info(
            "Po akceptacji zmien polozenie lub kat."
        )

        self.get_logger().info(
            "Celuj glownie w 10-25% powierzchni obrazu."
        )

        self.get_logger().info(
            "================================================"
        )


    # ========================================================
    # CZAS
    # ========================================================

    def stamp_to_seconds(
        self,
        stamp,
    ):

        return (
            float(stamp.sec)
            +
            float(stamp.nanosec) / 1e9
        )


    # ========================================================
    # UJEDNOLICENIE KOLEJNOSCI NAROZNIKOW
    # ========================================================


    def ros_image_to_bgr(
        self,
        msg,
    ):
        if msg.encoding != "bgr8":
            raise RuntimeError(
                f"Nieobslugiwane encoding: {msg.encoding}"
            )

        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)

        expected_row_bytes = width * 3

        if step < expected_row_bytes:
            raise RuntimeError(
                f"Niepoprawny step={step}, "
                f"oczekiwano co najmniej "
                f"{expected_row_bytes}"
            )

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        expected_total = (
            step
            *
            height
        )

        if raw.size < expected_total:
            raise RuntimeError(
                f"Za malo danych: {raw.size}, "
                f"oczekiwano {expected_total}"
            )

        rows = raw[
            :expected_total
        ].reshape(
            height,
            step,
        )

        pixels = rows[
            :,
            :expected_row_bytes
        ]

        image = pixels.reshape(
            height,
            width,
            3,
        )

        return np.ascontiguousarray(
            image
        )



    def canonicalize_corners(
        self,
        corners,
    ):

        grid = corners.reshape(
            BOARD_ROWS,
            BOARD_COLS,
            2
        ).copy()


        first_row_y = np.mean(
            grid[0, :, 1]
        )

        last_row_y = np.mean(
            grid[-1, :, 1]
        )


        if first_row_y > last_row_y:

            grid = np.flip(
                grid,
                axis=0
            )


        first_col_x = np.mean(
            grid[:, 0, 0]
        )

        last_col_x = np.mean(
            grid[:, -1, 0]
        )


        if first_col_x > last_col_x:

            grid = np.flip(
                grid,
                axis=1
            )


        return grid.reshape(
            -1,
            1,
            2
        ).astype(
            np.float32
        )


    # ========================================================
    # DETEKCJA SZACHOWNICY
    # ========================================================

    def find_board(
        self,
        image,
    ):

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )


        flags = (
            cv2.CALIB_CB_NORMALIZE_IMAGE
            |
            cv2.CALIB_CB_EXHAUSTIVE
            |
            cv2.CALIB_CB_ACCURACY
        )


        found, corners = (
            cv2.findChessboardCornersSB(
                gray,
                (
                    BOARD_COLS,
                    BOARD_ROWS,
                ),
                flags=flags,
            )
        )


        if not found:

            return None


        if corners is None:

            return None


        if len(corners) != (
            BOARD_COLS * BOARD_ROWS
        ):

            return None


        return self.canonicalize_corners(
            corners
        )


    # ========================================================
    # OPIS POLOZENIA PLANSZY
    # ========================================================

    def make_descriptor(
        self,
        corners,
        width,
        height,
    ):

        points = corners.reshape(
            -1,
            2
        )


        indices = [
            0,
            BOARD_COLS - 1,
            (
                (BOARD_ROWS - 1)
                *
                BOARD_COLS
            ),
            (
                BOARD_ROWS
                *
                BOARD_COLS
                -
                1
            ),
        ]


        descriptor = []


        for index in indices:

            descriptor.append(
                float(
                    points[index][0]
                    /
                    width
                )
            )

            descriptor.append(
                float(
                    points[index][1]
                    /
                    height
                )
            )


        return np.array(
            descriptor,
            dtype=np.float32
        )


    # ========================================================
    # POLE PLANSZY
    # ========================================================

    def board_area_ratio(
        self,
        corners,
        width,
        height,
    ):

        points = corners.reshape(
            -1,
            2
        )


        indices = [
            0,
            BOARD_COLS - 1,
            (
                BOARD_ROWS
                *
                BOARD_COLS
                -
                1
            ),
            (
                (BOARD_ROWS - 1)
                *
                BOARD_COLS
            ),
        ]


        polygon = points[
            indices
        ].astype(
            np.float32
        )


        area = abs(
            cv2.contourArea(
                polygon
            )
        )


        image_area = float(
            width * height
        )


        return area / image_area


    # ========================================================
    # CZY POLOZENIE JEST NOWE
    # ========================================================

    def is_diverse(
        self,
        descriptor,
    ):

        if not self.accepted_descriptors:

            return True


        distances = [
            float(
                np.linalg.norm(
                    descriptor
                    -
                    old
                )
            )
            for old
            in self.accepted_descriptors
        ]


        return (
            min(distances)
            >
            DIVERSITY_THRESHOLD
        )


    # ========================================================
    # STABILNOSC PLANSZY
    # ========================================================

    def update_stability(
        self,
        descriptor,
    ):

        if self.last_candidate_descriptor is None:

            self.last_candidate_descriptor = (
                descriptor.copy()
            )

            self.stable_counter = 1

            return False


        difference = float(
            np.linalg.norm(
                descriptor
                -
                self.last_candidate_descriptor
            )
        )


        if difference < STABILITY_THRESHOLD:

            self.stable_counter += 1

        else:

            self.stable_counter = 1


        self.last_candidate_descriptor = (
            descriptor.copy()
        )


        return (
            self.stable_counter
            >=
            STABLE_FRAMES_REQUIRED
        )


    def reset_stability(
        self,
    ):

        self.last_candidate_descriptor = None
        self.stable_counter = 0


    # ========================================================
    # ZAPIS DEBUGOWY
    # ========================================================

    def save_sample_images(
        self,
        left,
        right,
        left_corners,
        right_corners,
        number,
    ):

        left_debug = left.copy()
        right_debug = right.copy()


        cv2.drawChessboardCorners(
            left_debug,
            (
                BOARD_COLS,
                BOARD_ROWS,
            ),
            left_corners,
            True
        )


        cv2.drawChessboardCorners(
            right_debug,
            (
                BOARD_COLS,
                BOARD_ROWS,
            ),
            right_corners,
            True
        )


        cv2.imwrite(
            os.path.join(
                SAMPLES_DIR,
                f"left_{number:02d}.jpg"
            ),
            left_debug
        )


        cv2.imwrite(
            os.path.join(
                SAMPLES_DIR,
                f"right_{number:02d}.jpg"
            ),
            right_debug
        )


    # ========================================================
    # CALLBACK
    # ========================================================

    def stereo_callback(
        self,
        left_msg,
        right_msg,
    ):

        if self.done:

            return


        if self.calibrating:

            return


        now = time.monotonic()


        if (
            now
            -
            self.last_accept_time
            <
            SAMPLE_COOLDOWN_SEC
        ):

            return


        # ====================================================
        # KONWERSJA OBRAZOW
        # ====================================================

        try:

            left = self.ros_image_to_bgr(
                left_msg
            )

            right = self.ros_image_to_bgr(
                right_msg
            )

        except Exception as exc:

            self.get_logger().error(
                f"cv_bridge: {exc}"
            )

            return


        if (
            left.shape[:2]
            !=
            right.shape[:2]
        ):

            self.get_logger().error(
                "Rozne rozdzielczosci kamer."
            )

            return


        height, width = (
            left.shape[:2]
        )


        if self.image_size is None:

            self.image_size = (
                width,
                height
            )

            self.get_logger().info(
                f"Rozdzielczosc: "
                f"{width}x{height}"
            )


        # ====================================================
        # CZAS
        # ====================================================

        left_time = (
            self.stamp_to_seconds(
                left_msg.header.stamp
            )
        )

        right_time = (
            self.stamp_to_seconds(
                right_msg.header.stamp
            )
        )


        time_difference = abs(
            left_time
            -
            right_time
        )


        # Dodatkowe reczne zabezpieczenie.
        if time_difference > SYNC_SLOP_SEC:

            self.reset_stability()

            if (
                now
                -
                self.last_status_time
                >
                1.5
            ):

                self.get_logger().info(
                    f"Odrzucam pare: "
                    f"dt={time_difference * 1000:.1f} ms"
                )

                self.last_status_time = now

            return


        # ====================================================
        # NAROZNIKI
        # ====================================================

        left_corners = self.find_board(
            left
        )

        right_corners = self.find_board(
            right
        )


        if (
            left_corners is None
            or
            right_corners is None
        ):

            self.reset_stability()


            if (
                now
                -
                self.last_status_time
                >
                2.0
            ):

                self.get_logger().info(
                    f"Szukam planszy... "
                    f"{len(self.object_points)}/"
                    f"{TARGET_SAMPLES}"
                )

                self.last_status_time = now


            return


        # ====================================================
        # WIELKOSC PLANSZY
        # ====================================================

        left_area = self.board_area_ratio(
            left_corners,
            width,
            height,
        )

        right_area = self.board_area_ratio(
            right_corners,
            width,
            height,
        )


        if (
            left_area < MIN_BOARD_AREA_RATIO
            or
            right_area < MIN_BOARD_AREA_RATIO
        ):

            self.reset_stability()


            if (
                now
                -
                self.last_status_time
                >
                1.5
            ):

                self.get_logger().info(
                    f"Plansza za mala: "
                    f"L={left_area * 100:.1f}% "
                    f"R={right_area * 100:.1f}%. "
                    f"Podejdz blizej."
                )

                self.last_status_time = now


            return


        if (
            left_area > MAX_BOARD_AREA_RATIO
            or
            right_area > MAX_BOARD_AREA_RATIO
        ):

            self.reset_stability()


            if (
                now
                -
                self.last_status_time
                >
                1.5
            ):

                self.get_logger().info(
                    "Plansza za duza w obrazie. "
                    "Odsun ja."
                )

                self.last_status_time = now


            return


        # ====================================================
        # DESCRIPTOR
        # ====================================================

        left_descriptor = self.make_descriptor(
            left_corners,
            width,
            height,
        )

        right_descriptor = self.make_descriptor(
            right_corners,
            width,
            height,
        )


        descriptor = (
            left_descriptor
            +
            right_descriptor
        ) / 2.0


        # ====================================================
        # STABILNOSC
        # ====================================================

        stable = self.update_stability(
            descriptor
        )


        if not stable:

            if (
                now
                -
                self.last_status_time
                >
                1.0
            ):

                self.get_logger().info(
                    f"Plansza wykryta. "
                    f"TRZYMAJ NIERUCHOMO... "
                    f"{self.stable_counter}/"
                    f"{STABLE_FRAMES_REQUIRED}"
                )

                self.last_status_time = now


            return


        # ====================================================
        # ROZNORODNOSC
        # ====================================================

        if not self.is_diverse(
            descriptor
        ):

            self.reset_stability()


            if (
                now
                -
                self.last_status_time
                >
                1.0
            ):

                self.get_logger().info(
                    "Pozycja za podobna. "
                    "PRZESUN / PRZECHYL / ODDAL plansze."
                )

                self.last_status_time = now


            return


        # ====================================================
        # AKCEPTACJA
        # ====================================================

        self.object_points.append(
            self.object_template.copy()
        )

        self.left_points.append(
            left_corners.copy()
        )

        self.right_points.append(
            right_corners.copy()
        )

        self.accepted_descriptors.append(
            descriptor.copy()
        )


        sample_number = len(
            self.object_points
        )


        self.save_sample_images(
            left,
            right,
            left_corners,
            right_corners,
            sample_number,
        )


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "================================================"
        )

        self.get_logger().info(
            f"PARA ZAAKCEPTOWANA: "
            f"{sample_number}/{TARGET_SAMPLES}"
        )

        self.get_logger().info(
            f"Roznica czasu kamer: "
            f"{time_difference * 1000:.2f} ms"
        )

        self.get_logger().info(
            f"Plansza zajmuje: "
            f"L={left_area * 100:.1f}% "
            f"R={right_area * 100:.1f}% obrazu"
        )

        self.get_logger().info(
            "TERAZ ZMIEN POZYCJE / KAT / ODLEGLOSC"
        )

        self.get_logger().info(
            "================================================"
        )


        self.last_accept_time = (
            time.monotonic()
        )

        self.reset_stability()


        if sample_number >= TARGET_SAMPLES:

            self.calibrating = True

            self.perform_calibration()


    # ========================================================
    # KALIBRACJA POJEDYNCZEJ KAMERY
    #
    # V3:
    # - fx = fy
    # - principal point w centrum
    # - p1 = p2 = 0
    # - k3 = 0
    #
    # Dzięki temu solver nie moze "uciec" w absurdalne
    # parametry jak poprzednio.
    # ========================================================

    def calibrate_single_camera(
        self,
        image_points,
    ):

        width, height = (
            self.image_size
        )


        camera_matrix = (
            cv2.initCameraMatrix2D(
                self.object_points,
                image_points,
                self.image_size,
                aspectRatio=1.0,
            )
        )


        # Principal point:
        camera_matrix[0, 2] = (
            width - 1
        ) / 2.0

        camera_matrix[1, 2] = (
            height - 1
        ) / 2.0


        # Kwadratowe piksele:
        focal = float(
            (
                camera_matrix[0, 0]
                +
                camera_matrix[1, 1]
            )
            /
            2.0
        )

        camera_matrix[0, 0] = focal
        camera_matrix[1, 1] = focal


        distortion = np.zeros(
            (
                5,
                1
            ),
            dtype=np.float64,
        )


        flags = (
            cv2.CALIB_USE_INTRINSIC_GUESS
            |
            cv2.CALIB_FIX_ASPECT_RATIO
            |
            cv2.CALIB_FIX_PRINCIPAL_POINT
            |
            cv2.CALIB_ZERO_TANGENT_DIST
            |
            cv2.CALIB_FIX_K3
        )


        criteria = (
            cv2.TERM_CRITERIA_EPS
            |
            cv2.TERM_CRITERIA_MAX_ITER,
            200,
            1e-9,
        )


        return cv2.calibrateCamera(
            self.object_points,
            image_points,
            self.image_size,
            camera_matrix,
            distortion,
            flags=flags,
            criteria=criteria,
        )


    # ========================================================
    # YAML CAMERA_INFO
    # ========================================================

    def write_camera_yaml(
        self,
        path,
        camera_name,
        camera_matrix,
        distortion,
        rectification,
        projection,
    ):

        data = {
            "image_width":
                int(
                    self.image_size[0]
                ),

            "image_height":
                int(
                    self.image_size[1]
                ),

            "camera_name":
                camera_name,

            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data":
                    camera_matrix
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },

            "distortion_model":
                "plumb_bob",

            "distortion_coefficients": {
                "rows": 1,
                "cols":
                    int(
                        distortion.size
                    ),

                "data":
                    distortion
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },

            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data":
                    rectification
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },

            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data":
                    projection
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },
        }


        with open(
            path,
            "w"
        ) as file:

            yaml.safe_dump(
                data,
                file,
                sort_keys=False
            )


    # ========================================================
    # GLOWNA KALIBRACJA
    # ========================================================

    def perform_calibration(
        self,
    ):

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "###############################################"
        )

        self.get_logger().info(
            "LICZE KALIBRACJE V3..."
        )

        self.get_logger().info(
            "###############################################"
        )


        # ====================================================
        # LEFT
        # ====================================================

        (
            left_rms,
            left_k,
            left_d,
            left_rvecs,
            left_tvecs,
        ) = self.calibrate_single_camera(
            self.left_points
        )


        # ====================================================
        # RIGHT
        # ====================================================

        (
            right_rms,
            right_k,
            right_d,
            right_rvecs,
            right_tvecs,
        ) = self.calibrate_single_camera(
            self.right_points
        )


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "INTRINSICS LEFT:"
        )

        self.get_logger().info(
            f"fx={left_k[0, 0]:.3f} "
            f"fy={left_k[1, 1]:.3f} "
            f"cx={left_k[0, 2]:.3f} "
            f"cy={left_k[1, 2]:.3f}"
        )

        self.get_logger().info(
            f"D LEFT: "
            f"{left_d.reshape(-1)}"
        )


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "INTRINSICS RIGHT:"
        )

        self.get_logger().info(
            f"fx={right_k[0, 0]:.3f} "
            f"fy={right_k[1, 1]:.3f} "
            f"cx={right_k[0, 2]:.3f} "
            f"cy={right_k[1, 2]:.3f}"
        )

        self.get_logger().info(
            f"D RIGHT: "
            f"{right_d.reshape(-1)}"
        )


        # ====================================================
        # STEREO
        # ====================================================

        (
            stereo_rms,
            left_k,
            left_d,
            right_k,
            right_d,
            rotation,
            translation,
            essential,
            fundamental,
        ) = cv2.stereoCalibrate(
            self.object_points,
            self.left_points,
            self.right_points,

            left_k,
            left_d,

            right_k,
            right_d,

            self.image_size,

            criteria=(
                cv2.TERM_CRITERIA_EPS
                |
                cv2.TERM_CRITERIA_MAX_ITER,
                200,
                1e-8,
            ),

            flags=cv2.CALIB_FIX_INTRINSIC,
        )


        # ====================================================
        # RECTIFICATION
        # ====================================================

        (
            left_r,
            right_r,
            left_p,
            right_p,
            q_matrix,
            left_roi,
            right_roi,
        ) = cv2.stereoRectify(
            left_k,
            left_d,

            right_k,
            right_d,

            self.image_size,

            rotation,
            translation,

            flags=cv2.CALIB_ZERO_DISPARITY,

            alpha=0,
        )


        # ====================================================
        # BASELINE
        # ====================================================

        baseline = float(
            np.linalg.norm(
                translation
            )
        )


        baseline_error = abs(
            baseline
            -
            EXPECTED_BASELINE_M
        )


        tx = float(
            translation[0, 0]
        )

        ty = float(
            translation[1, 0]
        )

        tz = float(
            translation[2, 0]
        )


        # ====================================================
        # ZAPIS
        # ====================================================

        left_yaml = os.path.join(
            OUTPUT_DIR,
            "left.yaml"
        )

        right_yaml = os.path.join(
            OUTPUT_DIR,
            "right.yaml"
        )

        stereo_yaml = os.path.join(
            OUTPUT_DIR,
            "stereo.yaml"
        )


        self.write_camera_yaml(
            left_yaml,
            "left_camera",
            left_k,
            left_d,
            left_r,
            left_p,
        )


        self.write_camera_yaml(
            right_yaml,
            "right_camera",
            right_k,
            right_d,
            right_r,
            right_p,
        )


        stereo_data = {

            "image_width":
                int(
                    self.image_size[0]
                ),

            "image_height":
                int(
                    self.image_size[1]
                ),

            "board_inner_corners": [
                BOARD_COLS,
                BOARD_ROWS,
            ],

            "square_size_m":
                SQUARE_SIZE,

            "samples":
                len(
                    self.object_points
                ),

            "left_rms":
                float(
                    left_rms
                ),

            "right_rms":
                float(
                    right_rms
                ),

            "stereo_rms":
                float(
                    stereo_rms
                ),

            "measured_baseline_m":
                EXPECTED_BASELINE_M,

            "calculated_baseline_m":
                baseline,

            "baseline_error_m":
                baseline_error,

            "translation_x_m":
                tx,

            "translation_y_m":
                ty,

            "translation_z_m":
                tz,

            "rotation": {
                "rows": 3,
                "cols": 3,
                "data":
                    rotation
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },

            "translation_m": {
                "rows": 3,
                "cols": 1,
                "data":
                    translation
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },

            "essential_matrix": {
                "rows": 3,
                "cols": 3,
                "data":
                    essential
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },

            "fundamental_matrix": {
                "rows": 3,
                "cols": 3,
                "data":
                    fundamental
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },

            "Q": {
                "rows": 4,
                "cols": 4,
                "data":
                    q_matrix
                    .reshape(-1)
                    .astype(float)
                    .tolist(),
            },
        }


        with open(
            stereo_yaml,
            "w"
        ) as file:

            yaml.safe_dump(
                stereo_data,
                file,
                sort_keys=False
            )


        # ====================================================
        # OCENA
        # ====================================================

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "###############################################"
        )

        self.get_logger().info(
            "KALIBRACJA V3 GOTOWA"
        )

        self.get_logger().info(
            "###############################################"
        )


        self.get_logger().info(
            f"LEFT RMS:   "
            f"{left_rms:.5f} px"
        )

        self.get_logger().info(
            f"RIGHT RMS:  "
            f"{right_rms:.5f} px"
        )

        self.get_logger().info(
            f"STEREO RMS: "
            f"{stereo_rms:.5f} px"
        )


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            f"LEFT fx/fy: "
            f"{left_k[0, 0]:.2f} / "
            f"{left_k[1, 1]:.2f}"
        )

        self.get_logger().info(
            f"LEFT cx/cy: "
            f"{left_k[0, 2]:.2f} / "
            f"{left_k[1, 2]:.2f}"
        )


        self.get_logger().info(
            f"RIGHT fx/fy: "
            f"{right_k[0, 0]:.2f} / "
            f"{right_k[1, 1]:.2f}"
        )

        self.get_logger().info(
            f"RIGHT cx/cy: "
            f"{right_k[0, 2]:.2f} / "
            f"{right_k[1, 2]:.2f}"
        )


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            f"TRANSLACJA: "
            f"X={tx * 1000:.2f} mm "
            f"Y={ty * 1000:.2f} mm "
            f"Z={tz * 1000:.2f} mm"
        )


        self.get_logger().info(
            f"BASELINE RZECZYWISTY: "
            f"{EXPECTED_BASELINE_M * 1000:.2f} mm"
        )

        self.get_logger().info(
            f"BASELINE Z KALIBRACJI: "
            f"{baseline * 1000:.2f} mm"
        )

        self.get_logger().info(
            f"BLAD BASELINE: "
            f"{baseline_error * 1000:.2f} mm"
        )


        if (
            baseline_error
            <=
            BASELINE_WARNING_TOLERANCE_M
        ):

            self.get_logger().info(
                "BASELINE: OK"
            )

        else:

            self.get_logger().warn(
                "BASELINE: PODEJRZANY"
            )


        if stereo_rms < 1.5:

            self.get_logger().info(
                "STEREO RMS: BARDZO DOBRY"
            )

        elif stereo_rms < 2.5:

            self.get_logger().info(
                "STEREO RMS: DOBRY / AKCEPTOWALNY"
            )

        elif stereo_rms < 4.0:

            self.get_logger().warn(
                "STEREO RMS: SREDNI"
            )

        else:

            self.get_logger().error(
                "STEREO RMS: ZA DUZY"
            )


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            f"LEFT:   {left_yaml}"
        )

        self.get_logger().info(
            f"RIGHT:  {right_yaml}"
        )

        self.get_logger().info(
            f"STEREO: {stereo_yaml}"
        )

        self.get_logger().info(
            f"PROBKI: {SAMPLES_DIR}"
        )

        self.get_logger().info(
            "###############################################"
        )


        self.done = True


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = StereoCalibrator()


    try:

        while (
            rclpy.ok()
            and
            not node.done
        ):

            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )


    except KeyboardInterrupt:

        node.get_logger().info(
            "Kalibracja przerwana."
        )


    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()