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
# SAME DLS PARAMETERS
# ============================================================

DAMPING = 0.5
MAX_ITERATIONS = 100
TOLERANCE = 0.005
NUMERICAL_STEP = 1e-5


# ============================================================
# TARGETS
#
# 1.30 m = positive reachable control
# 1.40 m = unresolved candidate
# 1.50 m = unresolved candidate
# ============================================================

TARGETS = [

    (
        "CONTROL_X_1_30",
        1.30,
        -0.30,
        0.40
    ),

    (
        "CANDIDATE_X_1_40",
        1.40,
        -0.30,
        0.40
    ),

    (
        "CANDIDATE_X_1_50",
        1.50,
        -0.30,
        0.40
    ),
]


# ============================================================
# MULTI-START SEEDS
# ============================================================

SEEDS_DEG = [

    (
        "ZERO",
        [0, 0, 0, 0]
    ),

    (
        "SEED_A",
        [30, -30, 30, 0]
    ),

    (
        "SEED_B",
        [-30, 30, -30, 0]
    ),

    (
        "SEED_C",
        [60, -45, 30, -30]
    ),

    (
        "SEED_D",
        [-60, 45, -30, 30]
    ),
]


OUTPUT_FILE = (
    Path(__file__).resolve().parent
    / "multistart_reachability_results.csv"
)


# ============================================================
# SET JOINT CONFIGURATION
# ============================================================

def set_joint_configuration(
    sim,
    client,
    joints,
    q_rad
):

    for joint, angle in zip(
        joints,
        q_rad
    ):

        sim.setJointPosition(
            joint,
            float(angle)
        )

    # Allow scene hierarchy to update
    for _ in range(3):

        client.step()

        time.sleep(0.005)


# ============================================================
# MAIN
# ============================================================

def main():

    print("\n============================================================")
    print("       ARIA-4 MULTI-START REACHABILITY VERIFICATION")
    print("============================================================")

    print(
        f"Targets              : {len(TARGETS)}"
    )

    print(
        f"Seeds per target     : {len(SEEDS_DEG)}"
    )

    print(
        f"Total DLS trials     : "
        f"{len(TARGETS) * len(SEEDS_DEG)}"
    )

    print(
        f"Tolerance            : "
        f"{TOLERANCE * 1000:.1f} mm"
    )

    print(
        f"Maximum iterations   : "
        f"{MAX_ITERATIONS}"
    )

    print("============================================================")


    # ========================================================
    # CONNECT
    # ========================================================

    print("\nConnecting to CoppeliaSim...")

    client = RemoteAPIClient()

    sim = client.require("sim")

    print("Connected.")


    joints = [

        sim.getObject(path)

        for path in JOINT_PATHS

    ]


    # ========================================================
    # EXISTING DLS SOLVER
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

    trial_number = 0


    # ========================================================
    # TARGET LOOP
    # ========================================================

    for (
        target_name,
        x,
        y,
        z
    ) in TARGETS:


        print("\n\n============================================================")
        print(f"TARGET: {target_name}")
        print(
            f"XYZ = ({x:.3f}, {y:.3f}, {z:.3f})"
        )
        print("============================================================")


        # ====================================================
        # SEED LOOP
        # ====================================================

        for (
            seed_name,
            seed_deg
        ) in SEEDS_DEG:

            trial_number += 1

            seed_rad = np.radians(
                np.array(
                    seed_deg,
                    dtype=float
                )
            )


            print("\n------------------------------------------------------------")

            print(
                f"TRIAL {trial_number:02d}"
            )

            print(
                f"Seed : {seed_name}"
            )

            print(
                f"q0   : {seed_deg} deg"
            )

            print("------------------------------------------------------------")


            # ================================================
            # SET INITIAL CONFIGURATION
            # ================================================

            set_joint_configuration(

                sim,
                client,
                joints,
                seed_rad
            )


            # ================================================
            # READ ACTUAL INITIAL TIP POSITION
            # ================================================

            initial_tip = np.array(

                sim.getObjectPosition(
                    dls.tip,
                    -1
                ),

                dtype=float
            )


            # ================================================
            # SOLVE
            # ================================================

            try:

                result = dls.solve(
                    [x, y, z]
                )

                q_final = np.array(
                    result["joint_positions"],
                    dtype=float
                )

                success = bool(
                    result["success"]
                )


                results.append({

                    "trial":
                        trial_number,

                    "target_name":
                        target_name,

                    "target_x":
                        x,

                    "target_y":
                        y,

                    "target_z":
                        z,

                    "seed_name":
                        seed_name,

                    "seed_q1_deg":
                        seed_deg[0],

                    "seed_q2_deg":
                        seed_deg[1],

                    "seed_q3_deg":
                        seed_deg[2],

                    "seed_q4_deg":
                        seed_deg[3],

                    "initial_tip_x":
                        initial_tip[0],

                    "initial_tip_y":
                        initial_tip[1],

                    "initial_tip_z":
                        initial_tip[2],

                    "success":
                        success,

                    "iterations":
                        result["iterations"],

                    "final_error_m":
                        result["error"],

                    "final_error_mm":
                        result["error"] * 1000.0,

                    "solver_time_s":
                        result["time"],

                    "final_q1_rad":
                        q_final[0],

                    "final_q2_rad":
                        q_final[1],

                    "final_q3_rad":
                        q_final[2],

                    "final_q4_rad":
                        q_final[3],

                    "final_q1_deg":
                        math.degrees(
                            q_final[0]
                        ),

                    "final_q2_deg":
                        math.degrees(
                            q_final[1]
                        ),

                    "final_q3_deg":
                        math.degrees(
                            q_final[2]
                        ),

                    "final_q4_deg":
                        math.degrees(
                            q_final[3]
                        ),
                })


                if success:

                    print(
                        "\nMULTI-START RESULT: "
                        "CONVERGED"
                    )

                else:

                    print(
                        "\nMULTI-START RESULT: "
                        "UNRESOLVED"
                    )


            except Exception as error:

                print(
                    "\nSOLVER EXCEPTION:"
                )

                print(
                    error
                )


                results.append({

                    "trial":
                        trial_number,

                    "target_name":
                        target_name,

                    "target_x":
                        x,

                    "target_y":
                        y,

                    "target_z":
                        z,

                    "seed_name":
                        seed_name,

                    "seed_q1_deg":
                        seed_deg[0],

                    "seed_q2_deg":
                        seed_deg[1],

                    "seed_q3_deg":
                        seed_deg[2],

                    "seed_q4_deg":
                        seed_deg[3],

                    "initial_tip_x":
                        initial_tip[0],

                    "initial_tip_y":
                        initial_tip[1],

                    "initial_tip_z":
                        initial_tip[2],

                    "success":
                        False,

                    "iterations":
                        0,

                    "final_error_m":
                        float("nan"),

                    "final_error_mm":
                        float("nan"),

                    "solver_time_s":
                        0.0,

                    "final_q1_rad":
                        float("nan"),

                    "final_q2_rad":
                        float("nan"),

                    "final_q3_rad":
                        float("nan"),

                    "final_q4_rad":
                        float("nan"),

                    "final_q1_deg":
                        float("nan"),

                    "final_q2_deg":
                        float("nan"),

                    "final_q3_deg":
                        float("nan"),

                    "final_q4_deg":
                        float("nan"),
                })


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

    print("\n\n============================================================")
    print("          MULTI-START VERIFICATION SUMMARY")
    print("============================================================")


    for (
        target_name,
        x,
        y,
        z
    ) in TARGETS:

        target_results = [

            r for r in results

            if r["target_name"]
            == target_name

        ]

        successful = [

            r for r in target_results

            if r["success"]

        ]


        print(
            f"\n{target_name}"
        )

        print(
            f"XYZ = "
            f"({x:.3f}, "
            f"{y:.3f}, "
            f"{z:.3f})"
        )

        print(
            f"Successful seeds : "
            f"{len(successful)}/"
            f"{len(target_results)}"
        )


        if successful:

            best = min(

                successful,

                key=lambda r:
                    r["final_error_mm"]
            )

            print(
                "Classification   : "
                "MULTISTART_REACHABLE"
            )

            print(
                f"Best error       : "
                f"{best['final_error_mm']:.3f} mm"
            )

            print(
                f"Best seed        : "
                f"{best['seed_name']}"
            )

        else:

            finite_failures = [

                r for r in target_results

                if math.isfinite(
                    r["final_error_mm"]
                )

            ]

            print(
                "Classification   : "
                "PERSISTENTLY_UNRESOLVED"
            )

            if finite_failures:

                best_failure = min(

                    finite_failures,

                    key=lambda r:
                        r["final_error_mm"]
                )

                print(
                    f"Smallest residual: "
                    f"{best_failure['final_error_mm']:.3f} mm"
                )

                print(
                    f"Best failed seed : "
                    f"{best_failure['seed_name']}"
                )


    total_time = (
        time.time()
        -
        experiment_start
    )


    print(
        f"\nTotal experiment time : "
        f"{total_time:.3f} s"
    )

    print(
        f"\nResults saved to:\n"
        f"{OUTPUT_FILE}"
    )

    print("============================================================")


    # ========================================================
    # RESTORE ZERO
    # ========================================================

    print(
        "\nRestoring zero configuration..."
    )

    set_joint_configuration(

        sim,
        client,
        joints,
        np.zeros(4)
    )

    print(
        "Robot restored."
    )


if __name__ == "__main__":

    main()