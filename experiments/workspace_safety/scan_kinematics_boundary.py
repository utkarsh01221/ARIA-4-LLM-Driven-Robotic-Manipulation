from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from dls_ik import DLSInverseKinematics

import numpy as np
import csv
import math
import time
from pathlib import Path


# ============================================================
# ARIA-4 OBJECT PATHS
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
# SAME DLS PARAMETERS AS PREVIOUS EXPERIMENT
# ============================================================

DAMPING = 0.5
MAX_ITERATIONS = 100
TOLERANCE = 0.005
NUMERICAL_STEP = 1e-5


# ============================================================
# STANDARD INITIAL CONFIGURATION
# ============================================================

INITIAL_Q = np.zeros(4)


# ============================================================
# EXPLORATORY KINEMATIC TARGETS
#
# IMPORTANT:
# These are NOT application-safe coordinates.
# They are deliberately outside the normal operating envelope
# for kinematic characterization only.
# ============================================================

TARGETS = [

    # --------------------------------------------------------
    # X+ direction
    # --------------------------------------------------------

    ("X_PLUS_01", 1.10, -0.30, 0.40),
    ("X_PLUS_02", 1.20, -0.30, 0.40),
    ("X_PLUS_03", 1.30, -0.30, 0.40),
    ("X_PLUS_04", 1.40, -0.30, 0.40),
    ("X_PLUS_05", 1.50, -0.30, 0.40),

    # --------------------------------------------------------
    # X- direction
    # --------------------------------------------------------

    ("X_MINUS_01", 0.60, -0.30, 0.40),
    ("X_MINUS_02", 0.50, -0.30, 0.40),
    ("X_MINUS_03", 0.40, -0.30, 0.40),
    ("X_MINUS_04", 0.30, -0.30, 0.40),

    # --------------------------------------------------------
    # Y- direction
    # --------------------------------------------------------

    ("Y_MINUS_01", 0.85, -0.70, 0.40),
    ("Y_MINUS_02", 0.85, -0.80, 0.40),
    ("Y_MINUS_03", 0.85, -0.90, 0.40),
    ("Y_MINUS_04", 0.85, -1.00, 0.40),
    ("Y_MINUS_05", 0.85, -1.10, 0.40),

    # --------------------------------------------------------
    # Y+ direction
    # --------------------------------------------------------

    ("Y_PLUS_01", 0.85, 0.10, 0.40),
    ("Y_PLUS_02", 0.85, 0.20, 0.40),
    ("Y_PLUS_03", 0.85, 0.30, 0.40),
    ("Y_PLUS_04", 0.85, 0.40, 0.40),

    # --------------------------------------------------------
    # Z+ direction
    # --------------------------------------------------------

    ("Z_PLUS_01", 0.85, -0.30, 0.70),
    ("Z_PLUS_02", 0.85, -0.30, 0.80),
    ("Z_PLUS_03", 0.85, -0.30, 0.90),
    ("Z_PLUS_04", 0.85, -0.30, 1.00),
    ("Z_PLUS_05", 0.85, -0.30, 1.10),

    # --------------------------------------------------------
    # Z- direction
    # --------------------------------------------------------

    ("Z_MINUS_01", 0.85, -0.30, 0.20),
    ("Z_MINUS_02", 0.85, -0.30, 0.10),
    ("Z_MINUS_03", 0.85, -0.30, 0.00),
]


OUTPUT_FILE = (
    Path(__file__).resolve().parent
    / "kinematic_boundary_scan.csv"
)


# ============================================================
# RESET JOINTS
# ============================================================

def reset_robot(
    sim,
    client,
    joints
):

    for joint, q in zip(
        joints,
        INITIAL_Q
    ):

        sim.setJointPosition(
            joint,
            float(q)
        )

    for _ in range(3):

        client.step()

        time.sleep(0.005)


# ============================================================
# MAIN
# ============================================================

def main():

    print("\n============================================================")
    print("           ARIA-4 KINEMATIC BOUNDARY SCAN")
    print("============================================================")

    print(
        f"Number of exploratory targets : "
        f"{len(TARGETS)}"
    )

    print(
        f"DLS tolerance                 : "
        f"{TOLERANCE * 1000:.1f} mm"
    )

    print(
        f"Maximum iterations            : "
        f"{MAX_ITERATIONS}"
    )

    print("============================================================")

    print("\nConnecting to CoppeliaSim...")

    client = RemoteAPIClient()

    sim = client.require("sim")

    print("Connected.")


    # ========================================================
    # HANDLES
    # ========================================================

    joints = [

        sim.getObject(path)

        for path in JOINT_PATHS

    ]


    # ========================================================
    # DLS
    # ========================================================

    dls = DLSInverseKinematics(

        sim=sim,

        client=client,

        joint_paths=JOINT_PATHS,

        tip_path=TIP_PATH,

        damping=DAMPING,

        max_iterations=MAX_ITERATIONS,

        tolerance=TOLERANCE,

        numerical_step=NUMERICAL_STEP
    )


    results = []

    experiment_start = time.time()


    # ========================================================
    # TEST TARGETS
    # ========================================================

    for test_number, target_data in enumerate(
        TARGETS,
        start=1
    ):

        name, x, y, z = target_data

        print("\n============================================================")

        print(
            f"TEST {test_number:02d}/{len(TARGETS)}"
        )

        print(
            f"Name   : {name}"
        )

        print(
            f"Target : "
            f"({x:.3f}, {y:.3f}, {z:.3f})"
        )

        print("============================================================")


        # ====================================================
        # ALWAYS RESET TO ZERO
        # ====================================================

        reset_robot(
            sim,
            client,
            joints
        )


        # ====================================================
        # SOLVE
        # ====================================================

        try:

            result = dls.solve(
                [x, y, z]
            )

            q = result[
                "joint_positions"
            ]

            success = bool(
                result["success"]
            )

            classification = (

                "DLS_REACHABLE"

                if success

                else "DLS_UNRESOLVED"
            )


            results.append({

                "test":
                    test_number,

                "name":
                    name,

                "target_x":
                    x,

                "target_y":
                    y,

                "target_z":
                    z,

                "dls_success":
                    success,

                "classification":
                    classification,

                "iterations":
                    result["iterations"],

                "final_error_m":
                    result["error"],

                "final_error_mm":
                    result["error"] * 1000.0,

                "solver_time_s":
                    result["time"],

                "q1_rad":
                    q[0],

                "q2_rad":
                    q[1],

                "q3_rad":
                    q[2],

                "q4_rad":
                    q[3],
            })


            print(
                "\nBOUNDARY CLASSIFICATION:"
            )

            print(
                classification
            )


        except Exception as error:

            print(
                "\nSOLVER ERROR:"
            )

            print(
                error
            )


            results.append({

                "test":
                    test_number,

                "name":
                    name,

                "target_x":
                    x,

                "target_y":
                    y,

                "target_z":
                    z,

                "dls_success":
                    False,

                "classification":
                    "SOLVER_ERROR",

                "iterations":
                    0,

                "final_error_m":
                    float("nan"),

                "final_error_mm":
                    float("nan"),

                "solver_time_s":
                    0.0,

                "q1_rad":
                    float("nan"),

                "q2_rad":
                    float("nan"),

                "q3_rad":
                    float("nan"),

                "q4_rad":
                    float("nan"),
            })


    # ========================================================
    # SAVE
    # ========================================================

    with open(
        OUTPUT_FILE,
        "w",
        newline="",
        encoding="utf-8"
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=results[0].keys()
        )

        writer.writeheader()

        writer.writerows(
            results
        )


    # ========================================================
    # SUMMARY
    # ========================================================

    reachable = [

        r for r in results

        if r["classification"]
        == "DLS_REACHABLE"

    ]

    unresolved = [

        r for r in results

        if r["classification"]
        == "DLS_UNRESOLVED"

    ]

    errors = [

        r for r in results

        if r["classification"]
        == "SOLVER_ERROR"

    ]


    total_time = (
        time.time()
        -
        experiment_start
    )


    print("\n============================================================")
    print("            KINEMATIC BOUNDARY SUMMARY")
    print("============================================================")

    print(
        f"Total targets       : "
        f"{len(results)}"
    )

    print(
        f"DLS reachable       : "
        f"{len(reachable)}"
    )

    print(
        f"DLS unresolved      : "
        f"{len(unresolved)}"
    )

    print(
        f"Solver errors       : "
        f"{len(errors)}"
    )

    print(
        f"Total time          : "
        f"{total_time:.3f} s"
    )


    # ========================================================
    # PRINT UNRESOLVED
    # ========================================================

    print("\n------------------------------------------------------------")
    print("DLS-UNRESOLVED TARGETS")
    print("------------------------------------------------------------")


    if unresolved:

        for r in unresolved:

            print(

                f"{r['name']:<12} | "

                f"XYZ=("
                f"{r['target_x']:.3f}, "
                f"{r['target_y']:.3f}, "
                f"{r['target_z']:.3f}) | "

                f"Error="
                f"{r['final_error_mm']:.3f} mm"
            )

    else:

        print(
            "No unresolved targets found."
        )


    print("\n------------------------------------------------------------")
    print("DLS-REACHABLE TARGETS")
    print("------------------------------------------------------------")


    for r in reachable:

        print(

            f"{r['name']:<12} | "

            f"Error="
            f"{r['final_error_mm']:.3f} mm | "

            f"Iterations="
            f"{r['iterations']}"
        )


    print("\nResults saved to:")

    print(
        OUTPUT_FILE
    )

    print("============================================================")


    # ========================================================
    # RESTORE ROBOT
    # ========================================================

    print(
        "\nRestoring zero configuration..."
    )

    reset_robot(
        sim,
        client,
        joints
    )

    print(
        "Robot restored."
    )


if __name__ == "__main__":

    main()