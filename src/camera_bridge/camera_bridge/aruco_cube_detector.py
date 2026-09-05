import json
import math
import os
from collections import deque

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, Int32, String
import message_filters


IMAGE_TOPIC = "/stereo/left_rectified"
POINTS_TOPIC = "/stereo/points"

DETECTED_TOPIC = "/cube_aruco/detected"
CAMERA_POINT_TOPIC = "/cube_aruco/position_camera"
YAW_TOPIC = "/cube_aruco/yaw_deg"
QUALITY_TOPIC = "/cube_aruco/quality"
SELECTED_ID_TOPIC = "/cube_aruco/id"
INFO_TOPIC = "/cube_aruco/info"
DEBUG_TOPIC = "/cube_aruco/debug/compressed"
RESET_TOPIC = "/cube_aruco/reset"

CAMERA_FRAME = "stereo_camera_optical"

CALIBRATION_LEFT_YAML = os.path.expanduser(
    "~/robot_ws/stereo_calibration_v3/left.yaml"
)

ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50

# ============================================================
# OBSLUGIWANE KOSTKI
#
# Robot moze chwycic kazda kostke z markerem:
#   ID 0
#   ID 1
#   ID 2
#   ID 3
#
# Jezeli kamera widzi kilka z nich jednoczesnie, detektor wybiera
# NAJBLIZSZA kostke na podstawie Z w ukladzie kamery.
# ============================================================
ARUCO_MARKER_IDS = {0, 1, 2, 3}

MARKER_SIZE_M = 0.050

# Ile kolejnych klatek marker moze zniknac zanim jego historia
# stabilizacji zostanie wyczyszczona.
MAX_MISSED_FRAMES = 5

# Jeśli nowy pomiar tego samego ID skoczy bardziej niż 25 mm względem
# historii, uznajemy że kostka/kamera zmieniła położenie i zaczynamy lock od zera.
HISTORY_RESET_JUMP_M = 0.025

# Temporal stability
HISTORY_SIZE = 12
MIN_STABLE_SAMPLES = 6
MAX_POS_SPREAD_M = 0.012
MAX_YAW_SPREAD_DEG = 5.0

# Stereo consistency
STEREO_INNER_MARGIN_PX = 3
MIN_STEREO_POINTS = 30
MAX_PNP_STEREO_Z_DIFF_M = 0.035
MAX_REPROJ_ERROR_PX = 4.0

SYNC_QUEUE = 20
SYNC_SLOP_SEC = 0.12


def wrap_square_yaw_deg(angle_deg):
    return ((float(angle_deg) + 45.0) % 90.0) - 45.0


def robust_median(values):
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return None
    med = float(np.median(arr))
    dev = np.abs(arr - med)
    mad = float(np.median(dev))
    if mad > 1e-9:
        sigma = 1.4826 * mad
        keep = dev <= max(2.8 * sigma, 1e-6)
        if np.count_nonzero(keep) >= max(3, arr.size // 2):
            arr = arr[keep]
    return float(np.median(arr))


def robust_square_yaw_median(values_deg):
    if not values_deg:
        return None
    a = np.deg2rad(np.asarray(values_deg, dtype=np.float64) * 4.0)
    s = float(np.median(np.sin(a)))
    c = float(np.median(np.cos(a)))
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return None
    return wrap_square_yaw_deg(math.degrees(math.atan2(s, c)) / 4.0)


def square_yaw_error_deg(a, b):
    return abs(wrap_square_yaw_deg(float(a) - float(b)))



def put_text_180(
    image,
    text,
    org,
    font_face,
    font_scale,
    color,
    thickness,
    line_type=cv2.LINE_AA,
):
    """
    Rysuje TYLKO podany napis obrócony o 180 stopni.
    Nie obraca obrazu kamery, markerów ani żadnych innych napisów.
    Pozycja napisu pozostaje w tym samym obszarze ekranu.
    """
    (text_w, text_h), baseline = cv2.getTextSize(
        text,
        font_face,
        font_scale,
        thickness,
    )

    pad = 4

    patch_h = text_h + baseline + 2 * pad
    patch_w = text_w + 2 * pad

    mask = np.zeros(
        (patch_h, patch_w),
        dtype=np.uint8,
    )

    cv2.putText(
        mask,
        text,
        (pad, pad + text_h),
        font_face,
        font_scale,
        255,
        thickness,
        line_type,
    )

    mask = cv2.rotate(
        mask,
        cv2.ROTATE_180,
    )

    x = int(org[0])
    y_top = int(org[1] - text_h)

    x1 = max(0, x - pad)
    y1 = max(0, y_top - pad)

    x2 = min(
        image.shape[1],
        x1 + patch_w,
    )
    y2 = min(
        image.shape[0],
        y1 + patch_h,
    )

    if x2 <= x1 or y2 <= y1:
        return

    roi = image[y1:y2, x1:x2]
    local_mask = mask[
        0:(y2 - y1),
        0:(x2 - x1),
    ]

    roi[local_mask > 0] = color


class ArucoCubeDetector(Node):
    def __init__(self):
        super().__init__("aruco_cube_detector")

        self.K, self.D, self.cx, self.cy = self.load_camera_calibration()

        self.dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 40
        params.cornerRefinementMinAccuracy = 0.01
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)

        s = MARKER_SIZE_M / 2.0
        # TL, TR, BR, BL in marker frame, z=0
        self.object_points = np.array(
            [
                [-s, +s, 0.0],
                [+s, +s, 0.0],
                [+s, -s, 0.0],
                [-s, -s, 0.0],
            ],
            dtype=np.float32,
        )

        # Osobna historia dla kazdego ID.
        # To jest bardzo wazne: pozycje/yaw kostki ID0 nie moga
        # mieszac sie z historia ID1, ID2 lub ID3.
        self.pose_histories = {
            marker_id: deque(maxlen=HISTORY_SIZE)
            for marker_id in ARUCO_MARKER_IDS
        }

        self.missed_frames = {
            marker_id: 0
            for marker_id in ARUCO_MARKER_IDS
        }

        self.selected_marker_id = None

        self.detected_pub = self.create_publisher(Bool, DETECTED_TOPIC, 10)
        self.camera_pub = self.create_publisher(PointStamped, CAMERA_POINT_TOPIC, 10)
        self.yaw_pub = self.create_publisher(Float32, YAW_TOPIC, 10)
        self.quality_pub = self.create_publisher(Float32, QUALITY_TOPIC, 10)
        self.selected_id_pub = self.create_publisher(
            Int32,
            SELECTED_ID_TOPIC,
            10,
        )
        self.info_pub = self.create_publisher(String, INFO_TOPIC, 10)
        self.debug_pub = self.create_publisher(CompressedImage, DEBUG_TOPIC, 5)

        self.image_sub = message_filters.Subscriber(
            self, Image, IMAGE_TOPIC, qos_profile=qos_profile_sensor_data
        )
        self.points_sub = message_filters.Subscriber(
            self, Image, POINTS_TOPIC, qos_profile=qos_profile_sensor_data
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.points_sub],
            queue_size=SYNC_QUEUE,
            slop=SYNC_SLOP_SEC,
        )
        self.sync.registerCallback(self.callback)

        self.create_subscription(
            String,
            RESET_TOPIC,
            self.reset_callback,
            10,
        )

        self.get_logger().info("==========================================")
        self.get_logger().info("ARUCO CUBE DETECTOR")
        self.get_logger().info("DICT_4X4_50 | IDs=0,1,2,3 | marker=50 mm")
        self.get_logger().info(f"RGB: {IMAGE_TOPIC}")
        self.get_logger().info(f"XYZ: {POINTS_TOPIC}")
        self.get_logger().info(
            "PnP + stereo Z + osobna historia per ID + wybor NAJBLIZSZEJ"
        )
        self.get_logger().info(f"ID topic: {SELECTED_ID_TOPIC}")
        self.get_logger().info("detected=True gdy jest poprawny kandydat ArUco; stable pose jest tylko filtrem dodatkowym")
        self.get_logger().info(f"Reset historii: {RESET_TOPIC}")
        self.get_logger().info("==========================================")

    def reset_callback(self, msg):
        command = str(msg.data).strip().lower()

        if command not in ("", "reset", "clear", "1"):
            return

        for marker_id in ARUCO_MARKER_IDS:
            self.pose_histories[marker_id].clear()
            self.missed_frames[marker_id] = 0

        self.selected_marker_id = None

        dm = Bool()
        dm.data = False
        self.detected_pub.publish(dm)

        self.get_logger().info(
            "ARUCO RESET: wyczyszczono historie, czekam na świeży stabilny lock."
        )

    @staticmethod
    def _yaml_matrix(section):
        data = np.asarray(section["data"], dtype=np.float64)
        return data.reshape(int(section["rows"]), int(section["cols"]))

    def load_camera_calibration(self):
        if not os.path.exists(CALIBRATION_LEFT_YAML):
            raise RuntimeError(f"Brak kalibracji: {CALIBRATION_LEFT_YAML}")
        with open(CALIBRATION_LEFT_YAML, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        # left_rectified is rectified, so use projection matrix and zero distortion
        P = self._yaml_matrix(data["projection_matrix"])
        K = np.array(
            [
                [float(P[0, 0]), 0.0, float(P[0, 2])],
                [0.0, float(P[1, 1]), float(P[1, 2])],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        D = np.zeros((5, 1), dtype=np.float64)
        return K, D, float(P[0, 2]), float(P[1, 2])

    @staticmethod
    def ros_bgr(msg):
        if msg.encoding != "bgr8":
            raise RuntimeError(f"Expected bgr8, got {msg.encoding}")
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = int(msg.height * msg.step)
        if raw.size < expected:
            raise RuntimeError("Za malo danych BGR")
        rows = raw[:expected].reshape(msg.height, msg.step)
        img = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(img)

    @staticmethod
    def ros_xyz(msg):
        if msg.encoding != "32FC3":
            raise RuntimeError(f"Expected 32FC3, got {msg.encoding}")
        raw = np.frombuffer(msg.data, dtype=np.float32)
        expected = int(msg.height * msg.width * 3)
        if raw.size < expected:
            raise RuntimeError("Za malo danych XYZ")
        xyz = raw[:expected].reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(xyz)

    @staticmethod
    def marker_mask(shape, corners):
        mask = np.zeros(shape[:2], dtype=np.uint8)
        poly = np.rint(corners).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillConvexPoly(mask, poly, 255)
        if STEREO_INNER_MARGIN_PX > 0:
            k = 2 * STEREO_INNER_MARGIN_PX + 1
            kernel = np.ones((k, k), dtype=np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
        return mask

    def stereo_center(self, xyz, corners):
        mask = self.marker_mask(xyz.shape, corners)
        pts = xyz[mask > 0]
        if pts.size == 0:
            return None, 0, None
        valid = (
            np.all(np.isfinite(pts), axis=1)
            & (pts[:, 2] > 0.05)
            & (pts[:, 2] < 2.5)
        )
        pts = pts[valid]
        if pts.shape[0] < MIN_STEREO_POINTS:
            return None, int(pts.shape[0]), None

        z = pts[:, 2]
        z_med = float(np.median(z))
        z_mad = float(np.median(np.abs(z - z_med)))
        z_keep = np.abs(z - z_med) <= max(3.0 * 1.4826 * z_mad, 0.008)
        pts = pts[z_keep]
        if pts.shape[0] < MIN_STEREO_POINTS:
            return None, int(pts.shape[0]), z_mad
        return np.median(pts, axis=0).astype(np.float64), int(pts.shape[0]), z_mad

    def pose_from_pnp(self, corners):
        img_pts = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points,
            img_pts,
            self.K,
            self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None

        projected, _ = cv2.projectPoints(
            self.object_points, rvec, tvec, self.K, self.D
        )
        projected = projected.reshape(4, 2)
        reproj = float(np.mean(np.linalg.norm(projected - img_pts, axis=1)))

        R, _ = cv2.Rodrigues(rvec)
        # Marker x-axis projected into camera XY plane. For a top-facing marker this
        # is the in-plane orientation. Keep square symmetry in [-45,+45).
        x_axis_cam = R[:, 0]
        yaw = math.degrees(math.atan2(float(x_axis_cam[1]), float(x_axis_cam[0])))
        yaw = wrap_square_yaw_deg(yaw)

        return tvec.reshape(3).astype(np.float64), R, yaw, reproj

    def stable_pose(self, marker_id):
        history = self.pose_histories[marker_id]

        if len(history) < MIN_STABLE_SAMPLES:
            return None

        arr = np.asarray([p[0] for p in history], dtype=np.float64)
        yaws = [p[1] for p in history]
        qualities = np.asarray([p[2] for p in history], dtype=np.float64)

        med = np.median(arr, axis=0)
        dist = np.linalg.norm(arr - med, axis=1)
        cutoff = max(0.006, float(np.percentile(dist, 75)) * 1.8)
        keep = dist <= cutoff
        if np.count_nonzero(keep) < MIN_STABLE_SAMPLES:
            return None

        arr = arr[keep]
        yaws = [y for y, k in zip(yaws, keep) if k]
        qualities = qualities[keep]

        pos = np.median(arr, axis=0)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        yaw = robust_square_yaw_median(yaws)
        if yaw is None:
            return None
        yaw_dev = max(square_yaw_error_deg(v, yaw) for v in yaws)

        if float(np.max(spread)) > MAX_POS_SPREAD_M:
            return None
        if yaw_dev > MAX_YAW_SPREAD_DEG:
            return None

        return pos, float(yaw), spread, float(yaw_dev), float(np.median(qualities))

    def publish_debug(self, image, source_msg):
        ok, enc = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if not ok:
            return
        msg = CompressedImage()
        msg.header = source_msg.header
        msg.header.frame_id = CAMERA_FRAME
        msg.format = "jpeg"
        msg.data = enc.tobytes()
        self.debug_pub.publish(msg)

    def evaluate_marker(
        self,
        xyz,
        marker_id,
        marker_corners,
    ):
        """
        Liczy jedna kandydature kostki.

        Zwraca slownik zawierajacy:
          - marker_id
          - corners
          - fused XYZ
          - yaw
          - quality
          - dane diagnostyczne

        albo None, gdy klatka jest zbyt slaba.
        """

        selected = np.asarray(
            marker_corners,
            dtype=np.float32,
        ).reshape(4, 2)

        center_px = np.mean(
            selected,
            axis=0,
        )

        # ----------------------------------------------------
        # PnP
        # ----------------------------------------------------

        pnp = self.pose_from_pnp(
            selected
        )

        if pnp is None:
            return None

        pnp_xyz, _R, yaw, reproj = pnp

        # ----------------------------------------------------
        # Stereo XYZ wewnatrz markera
        # ----------------------------------------------------

        (
            stereo_xyz,
            stereo_count,
            z_mad,
        ) = self.stereo_center(
            xyz,
            selected,
        )

        stereo_ok = (
            stereo_xyz is not None
        )

        z_diff = (
            None
            if not stereo_ok
            else abs(
                float(
                    pnp_xyz[2]
                    -
                    stereo_xyz[2]
                )
            )
        )

        # ----------------------------------------------------
        # Jakosc
        # ----------------------------------------------------

        quality = 1.0

        quality *= max(
            0.0,
            1.0
            -
            reproj
            /
            max(
                MAX_REPROJ_ERROR_PX,
                1e-6,
            ),
        )

        if stereo_ok:

            quality *= max(
                0.0,
                1.0
                -
                z_diff
                /
                max(
                    MAX_PNP_STEREO_Z_DIFF_M,
                    1e-6,
                ),
            )

        else:

            quality *= 0.45

        # ----------------------------------------------------
        # Fuzja pozycji
        # ----------------------------------------------------

        fused = pnp_xyz.copy()

        if (
            stereo_ok
            and
            z_diff
            <=
            MAX_PNP_STEREO_Z_DIFF_M
        ):

            fused[2] = float(
                stereo_xyz[2]
            )

            # X/Y przeliczamy z centrum markera i stereo Z.
            u = float(center_px[0])
            v = float(center_px[1])

            fused[0] = (
                (u - self.K[0, 2])
                *
                fused[2]
                /
                self.K[0, 0]
            )

            fused[1] = (
                (v - self.K[1, 2])
                *
                fused[2]
                /
                self.K[1, 1]
            )

        valid_frame = (
            reproj
            <=
            MAX_REPROJ_ERROR_PX
            and
            quality
            >=
            0.10
            and
            np.all(
                np.isfinite(fused)
            )
            and
            0.05
            <
            fused[2]
            <
            2.5
        )

        if not valid_frame:
            return None

        return {
            "id": int(marker_id),
            "corners": selected,
            "center_px": center_px,
            "pnp_xyz": pnp_xyz,
            "stereo_xyz": stereo_xyz,
            "stereo_count": int(stereo_count),
            "z_mad": z_mad,
            "z_diff": z_diff,
            "yaw": float(yaw),
            "reproj": float(reproj),
            "quality": float(quality),
            "fused": fused,
        }


    def callback(
        self,
        image_msg,
        points_msg,
    ):

        try:

            image = self.ros_bgr(
                image_msg
            )

            xyz = self.ros_xyz(
                points_msg
            )

        except Exception as exc:

            self.get_logger().error(
                f"Input error: {exc}"
            )

            return


        debug = image.copy()

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY,
        )

        corners, ids, _ = (
            self.detector
            .detectMarkers(
                gray
            )
        )


        # ====================================================
        # 1. ZBIERAMY WSZYSTKIE KOSTKI ID 0..3
        # ====================================================

        candidates = []

        seen_ids = set()


        if ids is not None:

            ids_flat = ids.reshape(-1)

            for i, marker_id_raw in enumerate(
                ids_flat
            ):

                marker_id = int(
                    marker_id_raw
                )

                # Ignorujemy inne ArUco.
                if (
                    marker_id
                    not in
                    ARUCO_MARKER_IDS
                ):
                    continue


                seen_ids.add(
                    marker_id
                )


                candidate = (
                    self.evaluate_marker(
                        xyz,
                        marker_id,
                        corners[i],
                    )
                )


                if candidate is None:

                    # Widzimy marker, ale ta klatka nie nadaje sie
                    # do stabilizacji pozycji.
                    continue


                candidates.append(
                    candidate
                )


                # Jeżeli kostka albo kamera zmieniła położenie skokowo,
                # stara historia nie może mieszać się z nowym pomiarem.
                history = self.pose_histories[marker_id]

                if len(history) >= 3:
                    old_pos = np.median(
                        np.asarray(
                            [item[0] for item in history],
                            dtype=np.float64,
                        ),
                        axis=0,
                    )

                    jump = float(
                        np.linalg.norm(
                            candidate["fused"] - old_pos
                        )
                    )

                    if jump > HISTORY_RESET_JUMP_M:
                        history.clear()

                # Aktualizujemy HISTORIE TEGO KONKRETNEGO ID.
                self.pose_histories[
                    marker_id
                ].append(
                    (
                        candidate[
                            "fused"
                        ].copy(),

                        float(
                            candidate[
                                "yaw"
                            ]
                        ),

                        float(
                            candidate[
                                "quality"
                            ]
                        ),
                    )
                )


        # ====================================================
        # 2. CZYSZCZENIE STARYCH HISTORII
        # ====================================================

        for marker_id in (
            ARUCO_MARKER_IDS
        ):

            if marker_id in seen_ids:

                self.missed_frames[
                    marker_id
                ] = 0

            else:

                self.missed_frames[
                    marker_id
                ] += 1


                if (
                    self.missed_frames[
                        marker_id
                    ]
                    >
                    MAX_MISSED_FRAMES
                ):

                    self.pose_histories[
                        marker_id
                    ].clear()


        # ====================================================
        # 3. RYSUJEMY WSZYSTKIE WIDOCZNE ID
        # ====================================================

        if ids is not None:

            ids_flat = ids.reshape(-1)

            for i, marker_id_raw in enumerate(
                ids_flat
            ):

                marker_id = int(
                    marker_id_raw
                )

                if (
                    marker_id
                    not in
                    ARUCO_MARKER_IDS
                ):
                    continue


                marker_corners = (
                    np.asarray(
                        corners[i],
                        dtype=np.float32,
                    )
                    .reshape(
                        4,
                        2,
                    )
                )


                cv2.polylines(
                    debug,
                    [
                        np.rint(
                            marker_corners
                        )
                        .astype(
                            np.int32
                        )
                    ],
                    True,
                    (
                        0,
                        165,
                        255,
                    ),
                    1,
                )


                center = np.mean(
                    marker_corners,
                    axis=0,
                )


                cv2.putText(
                    debug,
                    f"ID {marker_id}",
                    (
                        int(
                            center[0]
                        )
                        -
                        18,

                        int(
                            center[1]
                        )
                        -
                        10,
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (
                        0,
                        165,
                        255,
                    ),
                    2,
                    cv2.LINE_AA,
                )


        # ====================================================
        # 4. BRAK POPRAWNEJ KOSTKI
        # ====================================================

        visible_valid = (
            len(candidates)
            >
            0
        )

        if not visible_valid:

            # /cube_aruco/detected oznacza od teraz:
            # "mam STABILNY pose XYZ+YAW gotowy dla cube_pick",
            # a nie tylko "w tej jednej klatce widzę narożniki markera".
            dm = Bool()
            dm.data = False
            self.detected_pub.publish(dm)

            self.selected_marker_id = None


            cv2.putText(
                debug,
                "ARUCO ID0-3: ---",
                (
                    10,
                    28,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (
                    0,
                    0,
                    255,
                ),
                2,
            )


            self.publish_debug(
                debug,
                image_msg,
            )

            return


        # ====================================================
        # 5. WYBOR NAJBLIZSZEJ KOSTKI
        # ====================================================
        #
        # Najmniejsze fused Z = kostka najblizej kamery.
        #
        # Przy prawie identycznym Z wygrywa wyzsza quality.
        # ====================================================

        candidates.sort(
            key=lambda c: (
                float(
                    c["fused"][2]
                ),
                -float(
                    c["quality"]
                ),
            )
        )


        selected = candidates[0]

        selected_id = int(
            selected["id"]
        )

        self.selected_marker_id = (
            selected_id
        )


        # Publikujemy ID wybranej kostki.
        id_msg = Int32()
        id_msg.data = (
            selected_id
        )

        self.selected_id_pub.publish(
            id_msg
        )


        # ====================================================
        # 6. DEBUG WYBRANEJ KOSTKI
        # ====================================================

        selected_corners = (
            selected[
                "corners"
            ]
        )


        cv2.polylines(
            debug,
            [
                np.rint(
                    selected_corners
                )
                .astype(
                    np.int32
                )
            ],
            True,
            (
                0,
                255,
                0,
            ),
            3,
        )


        center_px = (
            selected[
                "center_px"
            ]
        )


        cv2.circle(
            debug,
            tuple(
                np.rint(
                    center_px
                )
                .astype(
                    int
                )
            ),
            5,
            (
                0,
                255,
                255,
            ),
            -1,
        )


        pnp_xyz = (
            selected[
                "pnp_xyz"
            ]
        )

        stereo_xyz = (
            selected[
                "stereo_xyz"
            ]
        )

        stereo_count = int(
            selected[
                "stereo_count"
            ]
        )

        z_mad = (
            selected[
                "z_mad"
            ]
        )

        z_diff = (
            selected[
                "z_diff"
            ]
        )

        yaw = float(
            selected[
                "yaw"
            ]
        )

        reproj = float(
            selected[
                "reproj"
            ]
        )

        quality = float(
            selected[
                "quality"
            ]
        )

        fused = (
            selected[
                "fused"
            ]
        )


        # ====================================================
        # 7. STABILNA POZYCJA TYLKO DLA WYBRANEGO ID
        # ====================================================

        stable = self.stable_pose(
            selected_id
        )


        history = (
            self.pose_histories[
                selected_id
            ]
        )


        # JEDYNY autorytatywny sygnał dla cube_pick i aplikacji:
        # True tylko wtedy, gdy dokładnie w tej chwili mamy stabilny XYZ+YAW.
        dm = Bool()
        dm.data = bool(stable is not None)
        self.detected_pub.publish(dm)


        # ====================================================
        # AUTORYTATYWNY POSE DLA CUBE_PICK
        # ====================================================
        #
        # Detektor publikuje dane przy KAŻDEJ poprawnej klatce ArUco.
        # Jeżeli historia jest już stabilna, używamy mediany stabilnej.
        # Jeżeli jeszcze nie jest stabilna, używamy bieżącego fused XYZ/yaw.
        #
        # Dzięki temu cube_pick nie odmawia tylko dlatego, że historia
        # detektora nie zdążyła uzyskać locka. Cube_pick sam zbiera kilka
        # kolejnych próbek i sprawdza ich stabilność.
        # ====================================================

        if stable is not None:
            (
                pose_xyz,
                pose_yaw,
                spread,
                yaw_dev,
                pose_quality,
            ) = stable

            pose_source = "stable"

        else:
            pose_xyz = np.asarray(
                selected["fused"],
                dtype=np.float64,
            )
            pose_yaw = float(
                selected["yaw"]
            )
            pose_quality = float(
                selected["quality"]
            )
            spread = None
            yaw_dev = None
            pose_source = "live"

        # Mamy poprawny kandydat ArUco -> detected=True.
        dm = Bool()
        dm.data = True
        self.detected_pub.publish(dm)

        pm = PointStamped()
        pm.header = image_msg.header
        pm.header.frame_id = CAMERA_FRAME
        pm.point.x = float(pose_xyz[0])
        pm.point.y = float(pose_xyz[1])
        pm.point.z = float(pose_xyz[2])
        self.camera_pub.publish(pm)

        ym = Float32()
        ym.data = float(pose_yaw)
        self.yaw_pub.publish(ym)

        qm = Float32()
        qm.data = float(pose_quality)
        self.quality_pub.publish(qm)

        self.get_logger().info(
            f"ARUCO POSE {pose_source.upper()} | "
            f"ID={selected_id} | "
            f"X={pose_xyz[0]:+.4f} "
            f"Y={pose_xyz[1]:+.4f} "
            f"Z={pose_xyz[2]:+.4f} | "
            f"yaw={pose_yaw:+.2f} deg | "
            f"Q={pose_quality:.2f}"
        )

        info = {
            "detected": True,
            "stable": stable is not None,
            "pose_source": pose_source,
            "allowed_ids": sorted(ARUCO_MARKER_IDS),
            "visible_valid_ids": [
                int(candidate["id"])
                for candidate in candidates
            ],
            "selected_id": selected_id,
            "selection": "nearest_fused_z",
            "marker_size_m": MARKER_SIZE_M,
            "reproj_error_px": round(reproj, 3),
            "pnp_xyz": [
                round(float(v), 5)
                for v in pnp_xyz
            ],
            "stereo_xyz": (
                None
                if stereo_xyz is None
                else [
                    round(float(v), 5)
                    for v in stereo_xyz
                ]
            ),
            "stereo_points": stereo_count,
            "stereo_z_mad_m": (
                None
                if z_mad is None
                else round(float(z_mad), 5)
            ),
            "pnp_stereo_z_diff_m": (
                None
                if z_diff is None
                else round(float(z_diff), 5)
            ),
            "raw_yaw_deg": round(yaw, 3),
            "quality": round(quality, 3),
            "history": len(history),
            "stable_xyz": [
                round(float(v), 5)
                for v in pose_xyz
            ],
            "stable_yaw_deg": round(float(pose_yaw), 3),
            "spread_m": (
                None
                if spread is None
                else [
                    round(float(v), 5)
                    for v in spread
                ]
            ),
            "yaw_max_dev_deg": (
                None
                if yaw_dev is None
                else round(float(yaw_dev), 3)
            ),
        }

        put_text_180(
            debug,
            (
                f"ARUCO {pose_source.upper()} ID{selected_id} "
                f"{pose_xyz[0]:+.3f} "
                f"{pose_xyz[1]:+.3f} "
                f"{pose_xyz[2]:+.3f}"
            ),
            (10, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (
                0,
                255,
                0,
            ),
            2,
            cv2.LINE_AA,
        )

        im = String()
        im.data = json.dumps(info)
        self.info_pub.publish(im)

        self.publish_debug(
            debug,
            image_msg,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCubeDetector()
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