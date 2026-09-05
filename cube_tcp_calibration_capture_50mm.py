#!/usr/bin/env python3
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import JointState

from robot_motion.arm_kinematics import ArmKinematics, JOINT_NAMES


JOINT_TOPIC = "/joint_states"
CUBE_TOPIC = "/cube_aruco/position_camera"
CARTESIAN_TARGET_TOPIC = "/manipulator/cartesian_target"

LIFT_M = 0.050
TCP_TOLERANCE_M = 0.004
MOVE_TIMEOUT_S = 10.0

CUBE_SAMPLES = 10
CUBE_SAMPLE_TIMEOUT_S = 8.0
CUBE_MAX_SPREAD_M = 0.010


class CalibrationCapture(Node):
    def __init__(self):
        super().__init__("cube_tcp_calibration_capture")

        self.kin = ArmKinematics()

        self.current_q = None
        self.latest_cube = None
        self.cube_samples = deque(maxlen=100)

        self.create_subscription(
            JointState,
            JOINT_TOPIC,
            self.joint_cb,
            30,
        )

        self.create_subscription(
            PointStamped,
            CUBE_TOPIC,
            self.cube_cb,
            30,
        )

        self.target_pub = self.create_publisher(
            Point,
            CARTESIAN_TARGET_TOPIC,
            10,
        )

    def joint_cb(self, msg):
        values = dict(zip(msg.name, msg.position))

        if not all(name in values for name in JOINT_NAMES):
            return

        self.current_q = np.array(
            [float(values[name]) for name in JOINT_NAMES],
            dtype=float,
        )

    def cube_cb(self, msg):
        p = np.array(
            [
                float(msg.point.x),
                float(msg.point.y),
                float(msg.point.z),
            ],
            dtype=float,
        )

        if not np.all(np.isfinite(p)):
            return

        self.latest_cube = p
        self.cube_samples.append(p)

    def spin_until_joint_state(self, timeout=5.0):
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.current_q is not None:
                return True

        return False

    def current_tcp(self):
        if self.current_q is None:
            return None

        return np.asarray(
            self.kin.tcp(self.current_q),
            dtype=float,
        )

    def wait_for_cartesian_controller(self, timeout=5.0):
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.target_pub.get_subscription_count() > 0:
                return True

        return False

    def publish_target(self, xyz):
        msg = Point()
        msg.x = float(xyz[0])
        msg.y = float(xyz[1])
        msg.z = float(xyz[2])

        self.target_pub.publish(msg)

        self.get_logger().info(
            "CARTESIAN TARGET = "
            f"[{msg.x:+.6f}, {msg.y:+.6f}, {msg.z:+.6f}]"
        )

    def wait_until_tcp(self, target_xyz, timeout=MOVE_TIMEOUT_S):
        target_xyz = np.asarray(target_xyz, dtype=float)
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            tcp = self.current_tcp()
            if tcp is None:
                continue

            error = float(np.linalg.norm(tcp - target_xyz))

            if error <= TCP_TOLERANCE_M:
                self.get_logger().info(
                    f"TCP osiagnal cel. Blad = {error*1000:.1f} mm"
                )
                return True

        tcp = self.current_tcp()
        if tcp is not None:
            error = float(np.linalg.norm(tcp - target_xyz))
            self.get_logger().error(
                f"Timeout ruchu. Blad TCP = {error*1000:.1f} mm"
            )
        else:
            self.get_logger().error("Timeout ruchu - brak joint_states.")

        return False

    def collect_cube_median(self):
        # Wyrzucamy wszystko, co detector widzial przed podniesieniem chwytaka.
        self.cube_samples.clear()

        self.get_logger().info(
            f"Zbieram {CUBE_SAMPLES} nowych probek ArUco..."
        )

        deadline = time.monotonic() + CUBE_SAMPLE_TIMEOUT_S

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if len(self.cube_samples) >= CUBE_SAMPLES:
                break

        if len(self.cube_samples) < CUBE_SAMPLES:
            self.get_logger().error(
                f"Za malo probek kostki: "
                f"{len(self.cube_samples)}/{CUBE_SAMPLES}"
            )
            return None

        arr = np.asarray(
            list(self.cube_samples)[-CUBE_SAMPLES:],
            dtype=float,
        )

        median = np.median(arr, axis=0)

        spread = np.max(
            np.linalg.norm(arr - median, axis=1)
        )

        self.get_logger().info(
            f"Spread ArUco = {spread*1000:.1f} mm"
        )

        if spread > CUBE_MAX_SPREAD_M:
            self.get_logger().warn(
                f"ArUco ma duzy rozrzut: {spread*1000:.1f} mm"
            )

        return median


def fmt(v):
    return (
        f"[{float(v[0]):+.6f}, "
        f"{float(v[1]):+.6f}, "
        f"{float(v[2]):+.6f}]"
    )


def main():
    rclpy.init()
    node = CalibrationCapture()

    try:
        print()
        print("===================================================")
        print("KALIBRACJA KAMERA -> TCP")
        print("1. Najpierw zapisuje pozycje chwytaka.")
        print("2. Podnosi TCP o 50 mm.")
        print("3. Dopiero wtedy zbiera ArUco kostki.")
        print("4. Wraca DOKLADNIE do zapisanej pozycji chwytu.")
        print("===================================================")
        print()

        if not node.spin_until_joint_state():
            raise RuntimeError("Brak /joint_states.")

        if not node.wait_for_cartesian_controller():
            raise RuntimeError(
                "Brak subskrybenta /manipulator/cartesian_target. "
                "Czy cartesian_controller dziala?"
            )

        # --------------------------------------------------------
        # ETAP 1: chwytak jest recznie ustawiony idealnie nad kostka.
        # Zapisujemy JOINTY i TCP, NIE potrzebujemy jeszcze widziec ArUco.
        # --------------------------------------------------------
        grasp_q = node.current_q.copy()
        grasp_tcp = node.current_tcp().copy()

        print("ETAP 1 - ZAPISANO POZYCJE CHWYTAKA")
        print(f"GRASP_TCP_BASE = {fmt(grasp_tcp)}")
        print(
            "GRASP_JOINTS   = ["
            + ", ".join(f"{float(x):+.9f}" for x in grasp_q)
            + "]"
        )
        print()

        # --------------------------------------------------------
        # ETAP 2: 50 mm pionowo do gory w BASE.
        # --------------------------------------------------------
        lifted_tcp = grasp_tcp.copy()
        lifted_tcp[2] += LIFT_M

        print("ETAP 2 - PODNOSZE CHWYTAK +50 mm")
        print(f"LIFT_TARGET_BASE = {fmt(lifted_tcp)}")

        node.publish_target(lifted_tcp)

        if not node.wait_until_tcp(lifted_tcp):
            raise RuntimeError(
                "Nie udalo sie osiagnac pozycji +50 mm. "
                "Nie zbieram kalibracji."
            )

        # Dajemy kamerze chwile po zatrzymaniu.
        time.sleep(0.5)

        # --------------------------------------------------------
        # ETAP 3: dopiero teraz zbieramy kostke.
        # --------------------------------------------------------
        print()
        print("ETAP 3 - ZBIERAM KOSTKE ARUCO")

        cube_camera = node.collect_cube_median()

        if cube_camera is None:
            raise RuntimeError(
                "Nie udalo sie zebrac stabilnej pozycji ArUco."
            )

        print(f"CUBE_CAMERA = {fmt(cube_camera)}")

        # --------------------------------------------------------
        # ETAP 4: wracamy do DOKLADNIE tej samej zapisanej pozycji TCP.
        # --------------------------------------------------------
        print()
        print("ETAP 4 - SCHODZE -50 mm DO ZAPISANEGO TCP")

        node.publish_target(grasp_tcp)

        if not node.wait_until_tcp(grasp_tcp):
            raise RuntimeError(
                "Nie udalo sie wrocic do zapisanej pozycji chwytu."
            )

        final_tcp = node.current_tcp().copy()

        print()
        print("===================================================")
        print("GOTOWY PUNKT KALIBRACYJNY")
        print("===================================================")
        print(f"CAMERA_ARUCO = {fmt(cube_camera)}")
        print(f"TCP_BASE     = {fmt(grasp_tcp)}")
        print(f"TCP_FINAL    = {fmt(final_tcp)}")
        print(
            "JOINTS_GRASP = ["
            + ", ".join(f"{float(x):+.9f}" for x in grasp_q)
            + "]"
        )
        print("===================================================")
        print()
        print(
            "Wyslij mi blok CAMERA_ARUCO + TCP_BASE. "
            "Najlepiej zbierzemy tak 5-7 roznych polozen kostki."
        )

    except Exception as exc:
        node.get_logger().error(str(exc))

        # Jesli znamy pierwotny TCP, probujemy bezpiecznie wrocic,
        # ale tylko gdy zmienna istnieje i kontroler jest dostepny.
        if "grasp_tcp" in locals():
            try:
                node.get_logger().warn(
                    "Probuje wrocic do zapisanej pozycji chwytu..."
                )
                node.publish_target(grasp_tcp)
                node.wait_until_tcp(grasp_tcp, timeout=8.0)
            except Exception:
                pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()