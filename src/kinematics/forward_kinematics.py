import numpy as np
import math


# ============================================================
# ARIA-4 ANALYTICAL FORWARD KINEMATICS
#
# Convention:
# Standard Denavit-Hartenberg
#
# A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
#
# Units:
# distance -> metres
# angles   -> radians
# ============================================================


def dh_matrix(a, alpha, d, theta):

    ct = math.cos(theta)
    st = math.sin(theta)

    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array([
        [
            ct,
            -st * ca,
            st * sa,
            a * ct
        ],
        [
            st,
            ct * ca,
            -ct * sa,
            a * st
        ],
        [
            0.0,
            sa,
            ca,
            d
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0
        ]
    ], dtype=float)


# ============================================================
# WORLD -> DH FRAME 0
# ============================================================

T_WORLD_0 = np.array([
    [1.0, 0.0, 0.0,  0.120],
    [0.0, 1.0, 0.0, -0.225],
    [0.0, 0.0, 1.0,  0.496],
    [0.0, 0.0, 0.0,  1.000]
], dtype=float)


# ============================================================
# EXACT PARAMETERS DERIVED FROM COPPELIASIM JOINT GEOMETRY
#
# Tuple:
# (a, alpha, d, theta_offset)
# ============================================================

DH_PARAMETERS = [

    # Joint 1
    (
        0.142000000,
        math.radians(90.0),
        0.000000000,
        0.0
    ),

    # Joint 2
    (
        math.sqrt(0.008**2 + 0.406**2),
        0.0,
        0.020000000,
        math.atan2(0.406, 0.008)
    ),

    # Joint 3
    (
        math.sqrt(0.474**2 + 0.005**2),
        0.0,
        -0.015000000,
        (
            math.atan2(0.005, 0.474)
            - math.atan2(0.406, 0.008)
        )
    ),

    # Joint 4
    (
        0.0,
        0.0,
        0.0,
        0.0
    )
]


# ============================================================
# FIXED DH FRAME 4 -> IK_TIP TRANSFORM
#
# Derived from zero-configuration CoppeliaSim geometry.
# ============================================================

T_4_TIP = np.array([
    [
        0.0,
        -0.999944369,
        -0.010547936,
        0.13805643
    ],
    [
        0.0,
        0.010547936,
        -0.999944369,
        -0.12450314
    ],
    [
        1.0,
        0.0,
        0.0,
        0.00200000
    ],
    [
        0.0,
        0.0,
        0.0,
        1.0
    ]
], dtype=float)


def forward_kinematics(q):
    """
    Calculate ARIA-4 IK_Tip pose in the
    CoppeliaSim world coordinate system.

    Parameters
    ----------
    q : sequence of 4 floats
        Joint angles [q1,q2,q3,q4] in radians.

    Returns
    -------
    T_world_tip : 4x4 numpy array
        Homogeneous transformation matrix.
    """

    if len(q) != 4:
        raise ValueError(
            "ARIA-4 requires exactly four joint angles."
        )

    T = T_WORLD_0.copy()

    for i in range(4):

        a, alpha, d, theta_offset = (
            DH_PARAMETERS[i]
        )

        theta = float(q[i]) + theta_offset

        A = dh_matrix(
            a=a,
            alpha=alpha,
            d=d,
            theta=theta
        )

        T = T @ A

    T_world_tip = T @ T_4_TIP

    return T_world_tip


def get_position(T):

    return T[:3, 3].copy()


def get_rotation(T):

    return T[:3, :3].copy()


def print_dh_table():

    print("\n================================================")
    print("              ARIA-4 DH PARAMETERS")
    print("================================================")

    for i, parameter in enumerate(
        DH_PARAMETERS,
        start=1
    ):

        a, alpha, d, offset = parameter

        print(
            f"J{i}: "
            f"a={a:.9f} m | "
            f"d={d:.9f} m | "
            f"alpha={math.degrees(alpha):.6f} deg | "
            f"theta_offset="
            f"{math.degrees(offset):.6f} deg"
        )


def main():

    print_dh_table()

    q = np.zeros(4)

    T = forward_kinematics(q)

    position = get_position(T)

    print("\n================================================")
    print("          ZERO-CONFIGURATION FK")
    print("================================================")

    print("\nT_world_tip:")
    print(T)

    print("\nCartesian position:")

    print(f"X = {position[0]:.9f} m")
    print(f"Y = {position[1]:.9f} m")
    print(f"Z = {position[2]:.9f} m")

    expected = np.array([
        0.883362,
        -0.232000,
        0.783960
    ])

    error = np.linalg.norm(
        position - expected
    )

    print(
        f"\nZero-pose error = "
        f"{error * 1000:.9f} mm"
    )


if __name__ == "__main__":
    main()