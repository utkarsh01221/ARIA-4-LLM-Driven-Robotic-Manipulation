from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from dls_ik import DLSInverseKinematics
from workspace_validator import WorkspaceValidator

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
# EXACT DLS PARAMETERS FROM EXISTING BENCHMARK
# ============================================================

DAMPING = 0.5

MAX_ITERATIONS = 100

TOLERANCE = 0.005

NUMERICAL_STEP = 1e-5


# ============================================================
# STANDARDIZED INITIAL CONFIGURATION
# ============================================================

INITIAL_Q = np.array([
    0.0,
    0.0,
    0.0,
    0.0
], dtype=float)


# ============================================================
# WORKSPACE GRID
# ============================================================

X_VALUES = np.linspace(
    WorkspaceValidator.X_MIN,
    WorkspaceValidator.X_MAX,
    5
)

Y_VALUES = np.linspace(
    WorkspaceValidator.Y_MIN,
    WorkspaceValidator.Y_MAX,
    5
)

Z_VALUES = np.linspace(
    WorkspaceValidator.Z_MIN,
    WorkspaceValidator.Z_MAX,
    4
)


# ============================================================
# OUTPUT
# ============================================================

OUTPUT_FILE = (
    Path(__file__).resolve().parent
    / "workspace_reachability_map.csv"
)


# ============================================================
# JOINT RESET
# ============================================================

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

    # Allow hierarchy to update
    for _ in range(3):

        client.step()

        time.sleep(0.005)


# ============================================================
# MAIN
# ============================================================

def main():

    print("\n============================================================")
    print("          ARIA-4 WORKSPACE REACHABILITY SCAN")
    print("============================================================")

    print(
        f"X samples : {len(X_VALUES)}"
    )

    print(
        f"Y samples : {len(Y_VALUES)}"
    )

    print(
        f"Z samples : {len(Z_VALUES)}"
    )

    total_points = (
        len(X_VALUES)
        *
        len(Y_VALUES)
        *
        len(Z_VALUES)
    )

    print(
        f"Total Cartesian points : {total_points}"
    )

    print("\nDLS parameters:")

    print(
        f"  Damping lambda      : {DAMPING}"
    )

    print(
        f"  Maximum iterations  : {MAX_ITERATIONS}"
    )

    print(
        f"  Tolerance           : {TOLERANCE} m"
    )

    print(
        f"  Numerical step      : {NUMERICAL_STEP}"
    )

    print(
        "\nInitial configuration:"
    )

    print(
        "  q = [0, 0, 0, 0] rad"
    )

    print("============================================================")


    # ========================================================
    # CONNECT
    # ========================================================

    print(
        "\nConnecting to CoppeliaSim..."
    )

    client = RemoteAPIClient()

    sim = client.require("sim")

    print(
        "Connected."
    )


    # ========================================================
    # OBJECT HANDLES
    # ========================================================

    joints = [

        sim.getObject(path)

        for path in JOINT_PATHS

    ]


    # ========================================================
    # DLS SOLVER
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


    # ========================================================
    # RESULTS
    # ========================================================

    results = []

    scan_start = time.time()

    test_number = 0


    # ========================================================
    # SCAN WORKSPACE
    # ========================================================

    for z in Z_VALUES:

        for y in Y_VALUES:

            for x in X_VALUES:

                test_number += 1

                target = np.array(
                    [
                        float(x),
                        float(y),
                        float(z)
                    ],
                    dtype=float
                )

                print("\n")
                print("============================================================")

                print(
                    f"POINT {test_number:03d}/{total_points}"
                )

                print("============================================================")

                print(
                    f"Target XYZ = "
                    f"({x:.4f}, "
                    f"{y:.4f}, "
                    f"{z:.4f})"
                )


                # =================================================
                # WORKSPACE POLICY CHECK
                # =================================================

                workspace_result = (
                    WorkspaceValidator.validate(
                        float(x),
                        float(y),
                        float(z)
                    )
                )

                if not workspace_result[
                    "accepted"
                ]:

                    print(
                        "Unexpected workspace rejection."
                    )

                    results.append({

                        "test":
                            test_number,

                        "target_x":
                            float(x),

                        "target_y":
                            float(y),

                        "target_z":
                            float(z),

                        "workspace_pass":
                            False,

                        "dls_executed":
                            False,

                        "dls_success":
                            False,

                        "classification":
                            "REJECTED_WORKSPACE",

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

                    continue


                # =================================================
                # RESET TO SAME INITIAL CONFIGURATION
                # =================================================

                print(
                    "Workspace check: PASS"
                )

                print(
                    "Resetting joints to standardized "
                    "initial configuration..."
                )

                set_joint_configuration(

                    sim,
                    client,
                    joints,
                    INITIAL_Q
                )


                # =================================================
                # RUN EXISTING DLS SOLVER
                # =================================================

                try:

                    result = dls.solve(
                        target
                    )

                    q = result[
                        "joint_positions"
                    ]

                    success = bool(
                        result["success"]
                    )

                    classification = (

                        "REACHABLE"

                        if success

                        else "DLS_UNRESOLVED"
                    )

                    results.append({

                        "test":
                            test_number,

                        "target_x":
                            float(x),

                        "target_y":
                            float(y),

                        "target_z":
                            float(z),

                        "workspace_pass":
                            True,

                        "dls_executed":
                            True,

                        "dls_success":
                            success,

                        "classification":
                            classification,

                        "iterations":
                            int(
                                result["iterations"]
                            ),

                        "final_error_m":
                            float(
                                result["error"]
                            ),

                        "final_error_mm":
                            float(
                                result["error"]
                                * 1000.0
                            ),

                        "solver_time_s":
                            float(
                                result["time"]
                            ),

                        "q1_rad":
                            float(q[0]),

                        "q2_rad":
                            float(q[1]),

                        "q3_rad":
                            float(q[2]),

                        "q4_rad":
                            float(q[3]),
                    })


                    print(
                        "\nSCAN CLASSIFICATION:"
                    )

                    print(
                        classification
                    )


                except Exception as error:

                    print(
                        "\nSOLVER EXCEPTION:"
                    )

                    print(
                        error
                    )

                    results.append({

                        "test":
                            test_number,

                        "target_x":
                            float(x),

                        "target_y":
                            float(y),

                        "target_z":
                            float(z),

                        "workspace_pass":
                            True,

                        "dls_executed":
                            True,

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
    # TOTAL TIME
    # ========================================================

    total_time = (
        time.time()
        -
        scan_start
    )


    # ========================================================
    # SAVE CSV
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
        == "REACHABLE"

    ]

    unresolved = [

        r for r in results

        if r["classification"]
        == "DLS_UNRESOLVED"

    ]

    solver_errors = [

        r for r in results

        if r["classification"]
        == "SOLVER_ERROR"

    ]


    # ========================================================
    # CONVERGED ERRORS
    # ========================================================

    reachable_errors = [

        r["final_error_mm"]

        for r in reachable

        if math.isfinite(
            r["final_error_mm"]
        )

    ]


    print("\n")
    print("============================================================")
    print("             WORKSPACE REACHABILITY SUMMARY")
    print("============================================================")

    print(
        f"Total sampled points      : "
        f"{len(results)}"
    )

    print(
        f"DLS reachable             : "
        f"{len(reachable)}"
    )

    print(
        f"DLS unresolved            : "
        f"{len(unresolved)}"
    )

    print(
        f"Solver errors              : "
        f"{len(solver_errors)}"
    )


    if results:

        reachable_percentage = (

            len(reachable)
            /
            len(results)
            *
            100.0
        )

        unresolved_percentage = (

            len(unresolved)
            /
            len(results)
            *
            100.0
        )

        print(
            f"Reachable percentage       : "
            f"{reachable_percentage:.2f}%"
        )

        print(
            f"Unresolved percentage      : "
            f"{unresolved_percentage:.2f}%"
        )


    if reachable_errors:

        print(
            f"Mean converged error       : "
            f"{np.mean(reachable_errors):.3f} mm"
        )

        print(
            f"Maximum converged error    : "
            f"{np.max(reachable_errors):.3f} mm"
        )


    # ========================================================
    # UNRESOLVED POINTS
    # ========================================================

    print("\n------------------------------------------------------------")
    print("DLS-UNRESOLVED POINTS")
    print("------------------------------------------------------------")


    if unresolved:

        for result in unresolved:

            print(

                f"Test {result['test']:03d} | "

                f"XYZ = ("
                f"{result['target_x']:.4f}, "
                f"{result['target_y']:.4f}, "
                f"{result['target_z']:.4f}) | "

                f"Final error = "
                f"{result['final_error_mm']:.3f} mm"
            )

    else:

        print(
            "None found in coarse scan."
        )


    print("\n------------------------------------------------------------")

    print(
        f"Total scan time            : "
        f"{total_time:.3f} s"
    )

    print(
        f"Results saved to:\n"
        f"{OUTPUT_FILE}"
    )

    print("============================================================")


    # ========================================================
    # RESTORE ZERO CONFIGURATION
    # ========================================================

    print(
        "\nRestoring ARIA-4 to zero configuration..."
    )

    set_joint_configuration(

        sim,
        client,
        joints,
        INITIAL_Q
    )

    print(
        "Robot restored."
    )

    print("\n============================================================")
    print("             REACHABILITY SCAN COMPLETE")
    print("============================================================")


if __name__ == "__main__":

    main()