import os
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros


CAMERA_TOPIC = "/ai_cube/position_camera"
COMMAND_TOPIC = "/cube_tcp_calibration/command"
BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"

OUTPUT_FILE = os.path.expanduser("~/robot_ws/cube_grasp_calibration.yaml")
SAMPLES_FILE = os.path.expanduser("~/robot_ws/cube_grasp_samples.yaml")

CAMERA_BUFFER_SIZE = 15
MIN_CAMERA_SAMPLES = 6
MAX_CAMERA_SPREAD_M = 0.020
MIN_SAMPLES_TO_SOLVE = 10


class CubeTcpCalibrator(Node):
    """
    Kalibracja bezposrednia:
        surowy punkt 3D z kamery -> prawidlowy TOOL0 do chwytu

    Celowo NIE korzysta z /ai_cube/position_base ani z TF kamery.
    Dzieki temu bledna ekstrynsyka camera->base nie moze popsuc chwytu.
    """

    def __init__(self):
        super().__init__("cube_tcp_calibrator")

        self.camera_buffer = deque(maxlen=CAMERA_BUFFER_SIZE)
        self.camera_samples = []
        self.tool_samples = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointStamped, CAMERA_TOPIC, self.camera_callback, 20
        )
        self.create_subscription(
            String, COMMAND_TOPIC, self.command_callback, 10
        )

        self.load_samples()

        self.get_logger().info("==========================================")
        self.get_logger().info("CUBE GRASP CALIBRATOR V2")
        self.get_logger().info("KAMERA MUSI BYC NA SERVO_1 = 152 deg")
        self.get_logger().info("Kalibracja: CAMERA RAW 3D -> TOOL0")
        self.get_logger().info("Nie uzywa camera->base TF.")
        self.get_logger().info(f"Zapis probek: {SAMPLES_FILE}")
        self.get_logger().info(f"Wynik: {OUTPUT_FILE}")
        self.get_logger().info("Komendy: capture, list, solve, clear, flush")
        self.get_logger().info("==========================================")

    def camera_callback(self, msg):
        p = np.array(
            [float(msg.point.x), float(msg.point.y), float(msg.point.z)],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(p)):
            return
        if p[2] < 0.10 or p[2] > 2.0:
            return
        self.camera_buffer.append(p)

    def get_tool_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, TOOL_FRAME, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().error(
                f"Brak TF {BASE_FRAME}->{TOOL_FRAME}: {exc}"
            )
            return None

        t = tf.transform.translation
        return np.array([t.x, t.y, t.z], dtype=np.float64)

    def stable_camera_point(self):
        if len(self.camera_buffer) < MIN_CAMERA_SAMPLES:
            self.get_logger().error(
                f"Za malo swiezych pomiarow kamery: "
                f"{len(self.camera_buffer)}/{MIN_CAMERA_SAMPLES}"
            )
            return None

        data = np.asarray(self.camera_buffer, dtype=np.float64)
        median = np.median(data, axis=0)
        dist = np.linalg.norm(data - median, axis=1)

        cutoff = max(0.006, float(np.percentile(dist, 75)) * 1.5)
        good = data[dist <= cutoff]

        if good.shape[0] < MIN_CAMERA_SAMPLES:
            self.get_logger().error("Pomiar kamery niestabilny.")
            return None

        spread = np.max(good, axis=0) - np.min(good, axis=0)
        if float(np.max(spread)) > MAX_CAMERA_SPREAD_M:
            self.get_logger().error(
                "Za duzy spread kamery: "
                f"X={spread[0]*1000:.1f} "
                f"Y={spread[1]*1000:.1f} "
                f"Z={spread[2]*1000:.1f} mm"
            )
            return None

        p = np.median(good, axis=0)
        self.get_logger().info(
            "CAMERA stable: "
            f"({p[0]:+.5f},{p[1]:+.5f},{p[2]:+.5f}), "
            f"spread=({spread[0]*1000:.1f},"
            f"{spread[1]*1000:.1f},{spread[2]*1000:.1f}) mm"
        )
        return p

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "capture":
            self.capture()
        elif cmd == "list":
            self.list_samples()
        elif cmd == "solve":
            self.solve()
        elif cmd == "clear":
            self.clear()
        elif cmd == "flush":
            self.camera_buffer.clear()
            self.get_logger().info("Wyczyszczono tylko bufor biezacych pomiarow kamery.")
        else:
            self.get_logger().warn(f"Nieznana komenda: {cmd}")

    def capture(self):
        camera = self.stable_camera_point()
        if camera is None:
            return

        tool = self.get_tool_position()
        if tool is None:
            return

        self.camera_samples.append(camera.copy())
        self.tool_samples.append(tool.copy())
        self.save_samples()

        # KRYTYCZNE:
        # po zapisaniu probki czyscimy bufor pomiarow kamery.
        # Inaczej po przestawieniu kostki stare i nowe pozycje mieszaja sie
        # w jednym oknie i kalibrator raportuje ogromny spread.
        self.camera_buffer.clear()

        n = len(self.camera_samples)
        self.get_logger().info("")
        self.get_logger().info("==========================================")
        self.get_logger().info(f"PROBKA {n} ZAPISANA")
        self.get_logger().info(
            f"CAMERA = ({camera[0]:+.5f},{camera[1]:+.5f},{camera[2]:+.5f})"
        )
        self.get_logger().info(
            f"TOOL   = ({tool[0]:+.5f},{tool[1]:+.5f},{tool[2]:+.5f})"
        )
        self.get_logger().info("Przestaw kostke i ustaw chwytak od nowa.")
        self.get_logger().info("==========================================")

    def list_samples(self):
        self.get_logger().info(f"Liczba probek: {len(self.camera_samples)}")
        for i, (camera, tool) in enumerate(
            zip(self.camera_samples, self.tool_samples), start=1
        ):
            self.get_logger().info(
                f"{i:02d}: C=[{camera[0]:+.4f},{camera[1]:+.4f},{camera[2]:+.4f}] "
                f"T=[{tool[0]:+.4f},{tool[1]:+.4f},{tool[2]:+.4f}]"
            )

    def clear(self):
        self.camera_samples.clear()
        self.tool_samples.clear()
        self.camera_buffer.clear()
        self.save_samples()
        self.get_logger().warn("Wyczyszczono probki kalibracyjne.")

    def save_samples(self):
        data = {
            "camera_servo_deg": 152.0,
            "camera_frame": "stereo_camera_optical",
            "tool_frame": TOOL_FRAME,
            "samples": [
                {
                    "camera": [float(v) for v in camera],
                    "tool": [float(v) for v in tool],
                }
                for camera, tool in zip(self.camera_samples, self.tool_samples)
            ],
        }
        with open(SAMPLES_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def load_samples(self):
        if not os.path.exists(SAMPLES_FILE):
            return
        try:
            with open(SAMPLES_FILE, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            for item in data.get("samples", []):
                camera = np.asarray(item["camera"], dtype=np.float64)
                tool = np.asarray(item["tool"], dtype=np.float64)
                if camera.shape == (3,) and tool.shape == (3,):
                    self.camera_samples.append(camera)
                    self.tool_samples.append(tool)
            if self.camera_samples:
                self.get_logger().info(
                    f"Wczytano {len(self.camera_samples)} starych probek."
                )
        except Exception as exc:
            self.get_logger().error(f"Nie moge wczytac probek: {exc}")

    def solve(self):
        n = len(self.camera_samples)
        if n < MIN_SAMPLES_TO_SOLVE:
            self.get_logger().error(
                f"Za malo probek: {n}. Zrob minimum {MIN_SAMPLES_TO_SOLVE}, "
                "najlepiej 12-16 rozlozonych po calej strefie chwytu."
            )
            return

        camera = np.asarray(self.camera_samples, dtype=np.float64)
        tool = np.asarray(self.tool_samples, dtype=np.float64)

        design = np.hstack([camera, np.ones((n, 1), dtype=np.float64)])
        coeff, _, rank, _ = np.linalg.lstsq(design, tool, rcond=None)

        if rank < 4:
            self.get_logger().error(
                "Probki sa geometrycznie zbyt podobne. "
                "Rozloz kostke szerzej i na roznych odleglosciach."
            )
            return

        matrix = coeff[:3, :].T
        offset = coeff[3, :]
        predicted = design @ coeff
        errors = predicted - tool
        norms = np.linalg.norm(errors, axis=1)

        rms = float(np.sqrt(np.mean(norms ** 2)))
        max_error = float(np.max(norms))
        axis_rms = np.sqrt(np.mean(errors ** 2, axis=0))

        result = {
            "model": "affine_camera_to_grasp_tcp",
            "formula": "tool_base = matrix * cube_camera + offset",
            "camera_servo_deg": 152.0,
            "sample_count": int(n),
            "matrix": [[float(v) for v in row] for row in matrix],
            "offset": [float(v) for v in offset],
            "rms_error_m": rms,
            "max_error_m": max_error,
            "axis_rms_error_m": {
                "x": float(axis_rms[0]),
                "y": float(axis_rms[1]),
                "z": float(axis_rms[2]),
            },
        }

        with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(result, f, sort_keys=False)

        self.get_logger().info("")
        self.get_logger().info("==========================================")
        self.get_logger().info("KALIBRACJA POLICZONA")
        self.get_logger().info(f"RMS = {rms*1000:.1f} mm")
        self.get_logger().info(f"MAX = {max_error*1000:.1f} mm")
        self.get_logger().info(
            f"AXIS RMS = X:{axis_rms[0]*1000:.1f} "
            f"Y:{axis_rms[1]*1000:.1f} Z:{axis_rms[2]*1000:.1f} mm"
        )
        self.get_logger().info(f"Zapisano: {OUTPUT_FILE}")
        if rms > 0.010 or max_error > 0.020:
            self.get_logger().warn(
                "Blad nadal jest duzy. Dodaj probki w calej strefie i solve ponownie."
            )
        else:
            self.get_logger().info("Wynik wyglada wystarczajaco dobrze do testu START.")
        self.get_logger().info("==========================================")


def main(args=None):
    rclpy.init(args=args)
    node = CubeTcpCalibrator()
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