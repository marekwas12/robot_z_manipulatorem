import math
from dataclasses import dataclass

import numpy as np


# ============================================================
# JEDEN MODEL KINEMATYCZNY MANIPULATORA
# ============================================================

J1_J2_M = 0.155
J2_J3_M = 0.295

J3_J4_SIDE_M = 0.010
J3_J4_M = 0.150

J4_J5_M = 0.115
J5_TCP_M = 0.140

J2_STRAIGHT_RAD = 1.502451582768484
J3_STRAIGHT_RAD = 2.2172394305143346
J4_STRAIGHT_RAD = 1.6580627893946132

JOINT_LIMITS = {
    "joint_1": (-5.87425, 0.0),
    "joint_2": (0.0, 3.81371),
    "joint_3": (0.0, 4.46884),
    "joint_4": (math.radians(5.0), math.pi),
    "joint_5": (0.0, math.pi),
}

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


# ============================================================
# UKLAD ROBOCZY ROBOTA
#
# To NIE zmienia FK/IK ani TF base_link.
# FK/IK pozostaje w matematycznie poprawnym base_link.
#
# Ten uklad sluzy tylko do polecen fizycznych:
#
#   TASK +X = PRAWO robota
#   TASK +Y = PRZOD robota
#   TASK +Z = GORA
#
# Z testu uzytkownika:
#
#   base +X -> przod + lewo
#   base +Y -> przod + prawo
#
# dlatego poczatkowa korekcja wynosi 45 deg.
#
# Jezeli pozniej zmierzymy dokladny kat, zmieniamy TYLKO
# TASK_FRAME_YAW_DEG w tym jednym pliku.
# ============================================================

TASK_FRAME_YAW_DEG = 45.0
TASK_FRAME_YAW_RAD = math.radians(TASK_FRAME_YAW_DEG)


def task_axes_in_base():
    """
    Zwraca fizyczne osie robota wyrazone w base_link:

        right   = +X task
        forward = +Y task
        up      = +Z task

    Dla 45 deg:
        right   = (-1/sqrt(2), +1/sqrt(2), 0)
        forward = (+1/sqrt(2), +1/sqrt(2), 0)
    """
    a = TASK_FRAME_YAW_RAD
    c = math.cos(a)
    s = math.sin(a)

    forward = np.array(
        [c, s, 0.0],
        dtype=float,
    )

    right = np.array(
        [-s, c, 0.0],
        dtype=float,
    )

    up = np.array(
        [0.0, 0.0, 1.0],
        dtype=float,
    )

    return right, forward, up


def task_delta_to_base(dx_task, dy_task, dz_task):
    """
    Wektor ruchu w fizycznych osiach robota -> base_link.

    dx_task:
        + = w PRAWO
        - = w LEWO

    dy_task:
        + = do PRZODU
        - = do TYLU

    dz_task:
        + = GORA
        - = DOL
    """
    right, forward, up = task_axes_in_base()

    return (
        float(dx_task) * right
        + float(dy_task) * forward
        + float(dz_task) * up
    )


def base_delta_to_task(delta_xyz):
    """
    Odwrotne przeliczenie base_link -> fizyczne osie robota.
    """
    delta = np.asarray(delta_xyz, dtype=float)
    right, forward, up = task_axes_in_base()

    return np.array(
        [
            float(np.dot(delta, right)),
            float(np.dot(delta, forward)),
            float(np.dot(delta, up)),
        ],
        dtype=float,
    )


def rx(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ], dtype=float)


def rz(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def clamp_joint_vector(q):
    q = np.asarray(q, dtype=float).copy()

    for i, name in enumerate(JOINT_NAMES):
        lo, hi = JOINT_LIMITS[name]
        q[i] = min(
            hi,
            max(lo, q[i]),
        )

    return q


@dataclass
class FKResult:
    tcp: np.ndarray
    j2_origin: np.ndarray
    j3_origin: np.ndarray
    j4_origin: np.ndarray
    j5_origin: np.ndarray
    rotation_before_j5: np.ndarray


class ArmKinematics:
    """
    Model geometryczny manipulatora w base_link.

    UWAGA:
    TASK_FRAME nie jest tutaj wprowadzany do FK.
    To celowe.

    Test TF wykazal praktycznie idealna zgodnosc:
        ArmKinematics.tcp(q) == TF base_link->tool0

    Dlatego FK/IK pozostawiamy w base_link i tylko przeliczamy
    polecenia operatora przez task_delta_to_base().
    """

    def forward(self, q):
        q = np.asarray(q, dtype=float)

        if q.shape != (5,):
            raise ValueError(
                "q musi miec 5 elementow"
            )

        q1, q2, q3, q4, q5 = q

        # Zweryfikowane TF:
        # base_link->tool0 odpowiada Rz(+q1).
        R = rz(q1)

        p_j2 = np.array(
            [0.0, 0.0, J1_J2_M],
            dtype=float,
        )

        # J2
        R = (
            R
            @
            rx(
                q2
                -
                J2_STRAIGHT_RAD
            )
        )

        p_j3 = (
            p_j2
            +
            R
            @
            np.array(
                [0.0, 0.0, J2_J3_M]
            )
        )

        # J3
        R = (
            R
            @
            rx(
                q3
                -
                J3_STRAIGHT_RAD
            )
        )

        p_j4 = (
            p_j3
            +
            R
            @
            np.array([
                J3_J4_SIDE_M,
                0.0,
                J3_J4_M,
            ])
        )

        # J4
        R = (
            R
            @
            rx(
                q4
                -
                J4_STRAIGHT_RAD
            )
        )

        p_j5 = (
            p_j4
            +
            R
            @
            np.array(
                [0.0, 0.0, J4_J5_M]
            )
        )

        # J5 obraca orientacje, ale TCP lezy na osi J5.
        tcp = (
            p_j5
            +
            R
            @
            np.array(
                [0.0, 0.0, J5_TCP_M]
            )
        )

        return FKResult(
            tcp=tcp,
            j2_origin=p_j2,
            j3_origin=p_j3,
            j4_origin=p_j4,
            j5_origin=p_j5,
            rotation_before_j5=R,
        )

    def tcp(self, q):
        return self.forward(q).tcp

    def position_jacobian(
        self,
        q,
        active=(0, 1, 2),
    ):
        q = np.asarray(
            q,
            dtype=float,
        )

        base = self.tcp(q)
        eps = 1e-5

        J = np.zeros(
            (
                3,
                len(active),
            ),
            dtype=float,
        )

        for col, idx in enumerate(
            active
        ):
            qp = q.copy()
            qp[idx] += eps

            J[:, col] = (
                self.tcp(qp)
                -
                base
            ) / eps

        return J

    def solve_local_position(
        self,
        q_start,
        target_xyz,
        lock_j4=True,
        lock_j5=True,
        max_iterations=40,
        tolerance_m=0.0004,
    ):
        """
        Lokalna DLS IK pozycji TCP w base_link.
        """

        q = np.asarray(
            q_start,
            dtype=float,
        ).copy()

        target = np.asarray(
            target_xyz,
            dtype=float,
        )

        active = [
            0,
            1,
            2,
        ]

        if not lock_j4:
            active.append(3)

        if not lock_j5:
            active.append(4)

        # J5 nie zmienia pozycji TCP.
        if 4 in active:
            active.remove(4)

        initial_j4 = q[3]
        initial_j5 = q[4]

        damping = 0.015
        max_joint_step = (
            math.radians(2.0)
        )

        for _ in range(
            max_iterations
        ):
            current = self.tcp(q)
            error = target - current

            if (
                float(
                    np.linalg.norm(error)
                )
                <=
                tolerance_m
            ):
                if lock_j4:
                    q[3] = initial_j4

                if lock_j5:
                    q[4] = initial_j5

                return (
                    True,
                    clamp_joint_vector(q),
                    float(
                        np.linalg.norm(error)
                    ),
                )

            J = (
                self.position_jacobian(
                    q,
                    active=tuple(active),
                )
            )

            A = (
                J
                @
                J.T
                +
                (
                    damping
                    **
                    2
                )
                *
                np.eye(3)
            )

            try:
                dq_active = (
                    J.T
                    @
                    np.linalg.solve(
                        A,
                        error,
                    )
                )
            except np.linalg.LinAlgError:
                return (
                    False,
                    q,
                    float(
                        np.linalg.norm(error)
                    ),
                )

            dq_active = np.clip(
                dq_active,
                -max_joint_step,
                +max_joint_step,
            )

            for i, joint_idx in enumerate(
                active
            ):
                q[joint_idx] += (
                    dq_active[i]
                )

            if lock_j4:
                q[3] = initial_j4

            if lock_j5:
                q[4] = initial_j5

            q = clamp_joint_vector(q)

        final_error = float(
            np.linalg.norm(
                target
                -
                self.tcp(q)
            )
        )

        return (
            False,
            q,
            final_error,
        )


def radial_and_left_units(xyz):
    """
    Kierunki wzgledem osi obrotu J1.

    forward:
        od osi J1 na zewnatrz

    left:
        stycznie w lewo

    To jest najlepsza definicja dla polecen:
        "dalej od robota"
        "blizej robota"
        "w lewo/prawo wokol podstawy"

    i jest niezalezna od TASK_FRAME_YAW_DEG.
    """

    xyz = np.asarray(
        xyz,
        dtype=float,
    )

    r = np.array(
        [
            xyz[0],
            xyz[1],
            0.0,
        ],
        dtype=float,
    )

    n = float(
        np.linalg.norm(r)
    )

    if n < 1e-6:
        raise ValueError(
            "TCP jest za blisko osi J1"
        )

    forward = r / n

    left = np.array(
        [
            -forward[1],
            forward[0],
            0.0,
        ],
        dtype=float,
    )

    return forward, left


def apply_radial_tangential_offset(
    xyz,
    forward_m=0.0,
    right_m=0.0,
    up_m=0.0,
):
    """
    Wspolna funkcja do automatyki chwytu.

    forward_m:
        + = dalej od osi J1
        - = blizej osi J1

    right_m:
        + = w prawo stycznie
        - = w lewo stycznie

    up_m:
        + = gora
        - = dol
    """

    result = np.asarray(
        xyz,
        dtype=float,
    ).copy()

    forward, left = (
        radial_and_left_units(
            result
        )
    )

    result += (
        float(forward_m)
        *
        forward
    )

    # right = -left
    result -= (
        float(right_m)
        *
        left
    )

    result[2] += float(up_m)

    return result