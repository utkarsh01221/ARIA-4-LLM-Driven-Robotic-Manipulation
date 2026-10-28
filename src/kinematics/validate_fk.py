from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import numpy as np
import math
import csv
import time
from pathlib import Path

from forward_kinematics import forward_kinematics


# ============================================================
# ARIA-4 PATHS
# ============================================================

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


# ============================================================
# TEST CONFIGURATIONS
#
# Specified in degrees for readability.
# Converted to radians before use.
# ============================================================

TEST_CONFIGS_DEG = [

    [0, 0, 0, 0],

    [10, 0, 0, 0],
    [0, 10, 0, 0],
    [0, 0, 10, 0],
    [0, 0, 0, 10],

    [15, -20, 10, -15],
    [-20, 15, -25, 20],

    [30, -30, 20, 10],
    [-30, 25, 15, -20],

    [45, -35, 30, -25],
    [-45, 40, -30, 35],

    [60, -45, 35, 20],
    [-60, 45, -35, -20],

    [75, -50, 40, -30],
    [-75, 55, -40, 30],

    [90, -60, 45, -45],
    [-90, 60, -45, 45],

    [120, -70, 55, -60],
    [-120, 70, -55, 60],

    [150, -80, 65, -75],
]


def degrees_to_radians(configuration):

    return [
        math.radians(angle)
        for angle in configuration
    ]


def set_joint_configuration(
    sim,
    client,
    joints,
    q
):

    for joint, angle in zip(
        joints,
        q
    ):

        sim.setJointPosition(
            joint,
            float(angle)
        )

    # Allow CoppeliaSim to update hierarchy
    for _ in range(20):

        client.step()

        time.sleep(0.005)


def main():

    # ========================================================
    # CONNECT TO COPPELIASIM
    # ========================================================

    client = RemoteAPIClient()

    sim = client.require("sim")

    joints = [
        sim.getObject(path)
        for path in JOINT_PATHS
    ]

    tip = sim.getObject(
        TIP_PATH
    )

    print("\n============================================================")
    print("             ARIA-4 ANALYTICAL FK VALIDATION")
    print("============================================================")

    print(
        f"Number of configurations : "
        f"{len(TEST_CONFIGS_DEG)}"
    )

    print(
        "Reference                : "
        "CoppeliaSim IK_Tip"
    )

    print(
        "FK model                 : "
        "Standard DH analytical model"
    )

    print("============================================================")

    results = []

    # ========================================================
    # RUN TESTS
    # ========================================================

    for test_number, config_deg in enumerate(
        TEST_CONFIGS_DEG,
        start=1
    ):

        q = degrees_to_radians(
            config_deg
        )

        print("\n------------------------------------------------------------")

        print(
            f"TEST {test_number:02d}"
        )

        print(
            "Joint angles (deg): "
            f"{config_deg}"
        )

        # ----------------------------------------------------
        # Apply exact joint configuration to CoppeliaSim
        # ----------------------------------------------------

        set_joint_configuration(
            sim,
            client,
            joints,
            q
        )

        # ----------------------------------------------------
        # Read actual CoppeliaSim IK_Tip position
        # ----------------------------------------------------

        sim_xyz = np.array(

            sim.getObjectPosition(
                tip,
                -1
            ),

            dtype=float
        )

        # ----------------------------------------------------
        # Calculate analytical FK
        # ----------------------------------------------------

        T_fk = forward_kinematics(
            q
        )

        fk_xyz = T_fk[:3, 3]

        # ----------------------------------------------------
        # Cartesian error
        # ----------------------------------------------------

        error_m = (
            fk_xyz
            - sim_xyz
        )

        error_mm = (
            error_m
            * 1000.0
        )

        cartesian_error_mm = (
            np.linalg.norm(
                error_mm
            )
        )

        # ----------------------------------------------------
        # Display results
        # ----------------------------------------------------

        print(
            "\nAnalytical FK XYZ:"
        )

        print(
            f"X = {fk_xyz[0]: .9f} m"
        )

        print(
            f"Y = {fk_xyz[1]: .9f} m"
        )

        print(
            f"Z = {fk_xyz[2]: .9f} m"
        )

        print(
            "\nCoppeliaSim IK_Tip XYZ:"
        )

        print(
            f"X = {sim_xyz[0]: .9f} m"
        )

        print(
            f"Y = {sim_xyz[1]: .9f} m"
        )

        print(
            f"Z = {sim_xyz[2]: .9f} m"
        )

        print(
            "\nAxis error:"
        )

        print(
            f"Ex = {error_mm[0]: .9f} mm"
        )

        print(
            f"Ey = {error_mm[1]: .9f} mm"
        )

        print(
            f"Ez = {error_mm[2]: .9f} mm"
        )

        print(
            "\nCartesian position error:"
        )

        print(
            f"E = {cartesian_error_mm:.9f} mm"
        )

        # ----------------------------------------------------
        # Store result
        # ----------------------------------------------------

        results.append({

            "test":
                test_number,

            "q1_deg":
                config_deg[0],

            "q2_deg":
                config_deg[1],

            "q3_deg":
                config_deg[2],

            "q4_deg":
                config_deg[3],

            "q1_rad":
                q[0],

            "q2_rad":
                q[1],

            "q3_rad":
                q[2],

            "q4_rad":
                q[3],

            "fk_x_m":
                fk_xyz[0],

            "fk_y_m":
                fk_xyz[1],

            "fk_z_m":
                fk_xyz[2],

            "sim_x_m":
                sim_xyz[0],

            "sim_y_m":
                sim_xyz[1],

            "sim_z_m":
                sim_xyz[2],

            "error_x_mm":
                error_mm[0],

            "error_y_mm":
                error_mm[1],

            "error_z_mm":
                error_mm[2],

            "cartesian_error_mm":
                cartesian_error_mm
        })

    # ========================================================
    # STATISTICS
    # ========================================================

    cartesian_errors = np.array([

        result[
            "cartesian_error_mm"
        ]

        for result in results

    ])

    error_x = np.array([

        result[
            "error_x_mm"
        ]

        for result in results

    ])

    error_y = np.array([

        result[
            "error_y_mm"
        ]

        for result in results

    ])

    error_z = np.array([

        result[
            "error_z_mm"
        ]

        for result in results

    ])

    mean_error = np.mean(
        cartesian_errors
    )

    rms_error = np.sqrt(
        np.mean(
            cartesian_errors ** 2
        )
    )

    minimum_error = np.min(
        cartesian_errors
    )

    maximum_error = np.max(
        cartesian_errors
    )

    std_error = np.std(
        cartesian_errors
    )

    max_x_error = np.max(
        np.abs(error_x)
    )

    max_y_error = np.max(
        np.abs(error_y)
    )

    max_z_error = np.max(
        np.abs(error_z)
    )

    # ========================================================
    # PRINT SUMMARY
    # ========================================================

    print("\n")
    print("============================================================")
    print("                   FK VALIDATION SUMMARY")
    print("============================================================")

    print(
        f"Total configurations      : "
        f"{len(results)}"
    )

    print(
        f"Mean Cartesian error      : "
        f"{mean_error:.9f} mm"
    )

    print(
        f"RMS Cartesian error       : "
        f"{rms_error:.9f} mm"
    )

    print(
        f"Minimum Cartesian error   : "
        f"{minimum_error:.9f} mm"
    )

    print(
        f"Maximum Cartesian error   : "
        f"{maximum_error:.9f} mm"
    )

    print(
        f"Standard deviation        : "
        f"{std_error:.9f} mm"
    )

    print(
        f"Maximum |X error|         : "
        f"{max_x_error:.9f} mm"
    )

    print(
        f"Maximum |Y error|         : "
        f"{max_y_error:.9f} mm"
    )

    print(
        f"Maximum |Z error|         : "
        f"{max_z_error:.9f} mm"
    )

    print("============================================================")

    # ========================================================
    # SAVE CSV
    # ========================================================

    output_path = (
        Path(__file__).resolve().parent
        / "fk_validation_results.csv"
    )

    with open(
        output_path,
        "w",
        newline=""
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=results[0].keys()
        )

        writer.writeheader()

        writer.writerows(
            results
        )

    print(
        f"\nResults saved to:\n"
        f"{output_path}"
    )

    # ========================================================
    # RESTORE ZERO CONFIGURATION
    # ========================================================

    print(
        "\nRestoring robot to zero configuration..."
    )

    set_joint_configuration(
        sim,
        client,
        joints,
        [0.0, 0.0, 0.0, 0.0]
    )

    print(
        "Robot restored."
    )

    print("\n============================================================")
    print("               FK VALIDATION COMPLETE")
    print("============================================================")


if __name__ == "__main__":
    main()