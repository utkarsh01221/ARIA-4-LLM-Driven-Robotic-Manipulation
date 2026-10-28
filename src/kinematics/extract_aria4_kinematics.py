from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math
import time


BASE_PATH = "/base_link"

JOINT_PATHS = [
    "/base_link/Joint1",
    "/base_link/Joint1/Link1/Joint2",
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3",
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3/Link3/Joint4",
]

TIP_PATH = (
    "/base_link/Joint1/Link1/Joint2/Link2/"
    "Joint3/Link3/Joint4/Link4/IK_Tip"
)


def matrix12_to_T(m):
    """
    Convert CoppeliaSim 12-element transformation matrix
    into a conventional 4x4 homogeneous matrix.

    CoppeliaSim matrix is interpreted as:

    [R11 R12 R13 Px
     R21 R22 R23 Py
     R31 R32 R33 Pz]
    """

    return np.array([
        [m[0], m[1], m[2],  m[3]],
        [m[4], m[5], m[6],  m[7]],
        [m[8], m[9], m[10], m[11]],
        [0.0,  0.0,  0.0,   1.0]
    ], dtype=float)


def rotation_axis_from_T(T):
    """
    Revolute joints in CoppeliaSim rotate about
    their local Z axis.
    """

    return T[:3, 2]


def print_vector(name, v):
    print(
        f"{name:<15}: "
        f"[{v[0]: .9f}, {v[1]: .9f}, {v[2]: .9f}]"
    )


def angle_between(v1, v2):

    v1 = np.array(v1, dtype=float)
    v2 = np.array(v2, dtype=float)

    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)

    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)

    return math.degrees(math.acos(dot))


def main():

    client = RemoteAPIClient()
    sim = client.require("sim")

    print("\n==============================================")
    print("       ARIA-4 KINEMATIC GEOMETRY EXTRACTION")
    print("==============================================")

    base = sim.getObject(BASE_PATH)

    joints = [
        sim.getObject(path)
        for path in JOINT_PATHS
    ]

    tip = sim.getObject(TIP_PATH)

    print("\nHandles:")
    print("Base :", base)

    for i, joint in enumerate(joints, start=1):
        print(f"J{i}   :", joint)

    print("Tip  :", tip)

    # -----------------------------------------
    # Put all revolute joints at zero
    # -----------------------------------------

    print("\nSetting q = [0, 0, 0, 0]")

    for joint in joints:
        sim.setJointPosition(joint, 0.0)

    for _ in range(20):
        client.step()
        time.sleep(0.02)

    # -----------------------------------------
    # BASE TRANSFORM
    # -----------------------------------------

    base_matrix = sim.getObjectMatrix(base, -1)
    T_world_base = matrix12_to_T(base_matrix)

    print("\n==============================================")
    print("BASE TRANSFORM")
    print("==============================================")

    print(T_world_base)

    # -----------------------------------------
    # JOINT TRANSFORMS
    # -----------------------------------------

    joint_positions = []
    joint_axes = []
    joint_transforms = []

    for i, joint in enumerate(joints):

        matrix = sim.getObjectMatrix(joint, -1)
        T = matrix12_to_T(matrix)

        position = T[:3, 3]
        axis = rotation_axis_from_T(T)

        joint_positions.append(position)
        joint_axes.append(axis)
        joint_transforms.append(T)

        q = sim.getJointPosition(joint)

        print("\n==============================================")
        print(f"JOINT {i + 1}")
        print("==============================================")

        print(f"Path : {JOINT_PATHS[i]}")
        print(f"q    : {q:.9f} rad")
        print(f"q    : {math.degrees(q):.6f} deg")

        print_vector("Origin", position)
        print_vector("X axis", T[:3, 0])
        print_vector("Y axis", T[:3, 1])
        print_vector("Z axis", T[:3, 2])

        print("\nTransformation matrix:")
        print(T)

    # -----------------------------------------
    # TIP TRANSFORM
    # -----------------------------------------

    tip_matrix = sim.getObjectMatrix(tip, -1)
    T_world_tip = matrix12_to_T(tip_matrix)

    print("\n==============================================")
    print("IK TIP")
    print("==============================================")

    print_vector("Tip position", T_world_tip[:3, 3])

    print_vector("Tip X axis", T_world_tip[:3, 0])
    print_vector("Tip Y axis", T_world_tip[:3, 1])
    print_vector("Tip Z axis", T_world_tip[:3, 2])

    print("\nTip transformation matrix:")
    print(T_world_tip)

    # -----------------------------------------
    # DISTANCES
    # -----------------------------------------

    print("\n==============================================")
    print("JOINT ORIGIN DISTANCES")
    print("==============================================")

    for i in range(len(joint_positions) - 1):

        distance = np.linalg.norm(
            joint_positions[i + 1]
            - joint_positions[i]
        )

        print(
            f"|J{i + 1} -> J{i + 2}| "
            f"= {distance:.9f} m"
        )

    tip_distance = np.linalg.norm(
        T_world_tip[:3, 3]
        - joint_positions[-1]
    )

    print(
        f"|J4 -> IK_Tip| = "
        f"{tip_distance:.9f} m"
    )

    # -----------------------------------------
    # AXIS ANGLES
    # -----------------------------------------

    print("\n==============================================")
    print("ANGLE BETWEEN CONSECUTIVE JOINT AXES")
    print("==============================================")

    for i in range(len(joint_axes) - 1):

        angle = angle_between(
            joint_axes[i],
            joint_axes[i + 1]
        )

        print(
            f"J{i + 1} axis -> J{i + 2} axis "
            f"= {angle:.6f} deg"
        )

    # -----------------------------------------
    # TRANSFORM RELATIVE TO BASE
    # -----------------------------------------

    print("\n==============================================")
    print("JOINT TRANSFORMS RELATIVE TO BASE")
    print("==============================================")

    T_base_world = np.linalg.inv(T_world_base)

    for i, T_world_joint in enumerate(joint_transforms):

        T_base_joint = (
            T_base_world @ T_world_joint
        )

        print(f"\nT_base_J{i + 1}:")
        print(T_base_joint)

    T_base_tip = (
        T_base_world @ T_world_tip
    )

    print("\nT_base_tip:")
    print(T_base_tip)

    # -----------------------------------------
    # RELATIVE TRANSFORMS
    # -----------------------------------------

    print("\n==============================================")
    print("RELATIVE JOINT TRANSFORMS")
    print("==============================================")

    transforms = [
        T_world_base,
        *joint_transforms,
        T_world_tip
    ]

    names = [
        "Base",
        "J1",
        "J2",
        "J3",
        "J4",
        "Tip"
    ]

    for i in range(len(transforms) - 1):

        T_relative = (
            np.linalg.inv(transforms[i])
            @ transforms[i + 1]
        )

        print(
            f"\nT_{names[i]}_{names[i + 1]}:"
        )

        print(T_relative)

    # -----------------------------------------
    # SINGLE-JOINT PERTURBATION TEST
    # -----------------------------------------

    print("\n==============================================")
    print("SINGLE JOINT +0.1 RAD TEST")
    print("==============================================")

    perturbation = 0.1

    for i in range(len(joints)):

        for joint in joints:
            sim.setJointPosition(joint, 0.0)

        sim.setJointPosition(
            joints[i],
            perturbation
        )

        for _ in range(10):
            client.step()

        position = sim.getObjectPosition(
            tip,
            -1
        )

        print(
            f"q{i + 1}=+0.1 rad -> "
            f"Tip = "
            f"[{position[0]:.9f}, "
            f"{position[1]:.9f}, "
            f"{position[2]:.9f}]"
        )

    # -----------------------------------------
    # Restore zero configuration
    # -----------------------------------------

    for joint in joints:
        sim.setJointPosition(joint, 0.0)

    for _ in range(10):
        client.step()

    print("\n==============================================")
    print("GEOMETRY EXTRACTION COMPLETE")
    print("==============================================")


if __name__ == "__main__":
    main()