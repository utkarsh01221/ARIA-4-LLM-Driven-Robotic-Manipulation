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
# DLS PARAMETERS
# ============================================================

DAMPING = 0.5
MAX_ITERATIONS = 100
TOLERANCE = 0.005
NUMERICAL_STEP = 1e-5

INITIAL_Q = np.zeros(4)


# ============================================================
# TEST CASES
# ============================================================

TESTS = [

    # ========================================================
    # A — VALID INTERIOR
    # ========================================================

    ("A01", "VALID_INTERIOR",
     0.85, -0.30, 0.385, "ACCEPTED"),

    ("A02", "VALID_INTERIOR",
     0.80, -0.20, 0.470, "ACCEPTED"),

    ("A03", "VALID_INTERIOR",
     0.90, -0.40, 0.550, "ACCEPTED"),

    ("A04", "VALID_INTERIOR",
     0.75, -0.15, 0.400, "ACCEPTED"),

    ("A05", "VALID_INTERIOR",
     1.00, -0.50, 0.350, "ACCEPTED"),


    # ========================================================
    # B — EXACT WORKSPACE BOUNDARIES
    # ========================================================

    ("B01", "EXACT_BOUNDARY",
     0.70, -0.30, 0.385, "ACCEPTED"),

    ("B02", "EXACT_BOUNDARY",
     1.05, -0.30, 0.385, "ACCEPTED"),

    ("B03", "EXACT_BOUNDARY",
     0.85, -0.60, 0.385, "ACCEPTED"),

    ("B04", "EXACT_BOUNDARY",
     0.85, 0.00, 0.385, "ACCEPTED"),

    ("B05", "EXACT_BOUNDARY",
     0.85, -0.30, 0.300, "ACCEPTED"),

    ("B06", "EXACT_BOUNDARY",
     0.85, -0.30, 0.600, "ACCEPTED"),


    # ========================================================
    # C — EXACTLY 1 mm OUTSIDE
    # ========================================================

    ("C01", "OUTSIDE_1MM",
     0.699, -0.30, 0.385,
     "REJECTED_WORKSPACE"),

    ("C02", "OUTSIDE_1MM",
     1.051, -0.30, 0.385,
     "REJECTED_WORKSPACE"),

    ("C03", "OUTSIDE_1MM",
     0.85, -0.601, 0.385,
     "REJECTED_WORKSPACE"),

    ("C04", "OUTSIDE_1MM",
     0.85, 0.001, 0.385,
     "REJECTED_WORKSPACE"),

    ("C05", "OUTSIDE_1MM",
     0.85, -0.30, 0.299,
     "REJECTED_WORKSPACE"),

    ("C06", "OUTSIDE_1MM",
     0.85, -0.30, 0.601,
     "REJECTED_WORKSPACE"),


    # ========================================================
    # D — KINEMATICALLY REACHABLE BUT POLICY-FORBIDDEN
    #
    # Established during exploratory scan.
    # Safety layer should reject before IK.
    # ========================================================

    ("D01", "REACHABLE_POLICY_FORBIDDEN",
     1.10, -0.30, 0.40,
     "REJECTED_WORKSPACE"),

    ("D02", "REACHABLE_POLICY_FORBIDDEN",
     1.20, -0.30, 0.40,
     "REJECTED_WORKSPACE"),

    ("D03", "REACHABLE_POLICY_FORBIDDEN",
     1.30, -0.30, 0.40,
     "REJECTED_WORKSPACE"),

    ("D04", "REACHABLE_POLICY_FORBIDDEN",
     0.85, 0.10, 0.40,
     "REJECTED_WORKSPACE"),


    # ========================================================
    # E — OUTSIDE + UNREACHABLE/EXTREME
    # ========================================================

    ("E01", "VERIFIED_UNREACHABLE",
     1.40, -0.30, 0.40,
     "REJECTED_WORKSPACE"),

    ("E02", "VERIFIED_UNREACHABLE",
     1.50, -0.30, 0.40,
     "REJECTED_WORKSPACE"),

    ("E03", "EXTREME_UNSAFE",
     2.00, -0.30, 0.40,
     "REJECTED_WORKSPACE"),

    ("E04", "EXTREME_UNSAFE",
     0.85, -0.30, 1.50,
     "REJECTED_WORKSPACE"),
]


OUTPUT_FILE = (
    Path(__file__).resolve().parent
    / "workspace_rejection_results.csv"
)


# ============================================================
# RESET ROBOT
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
    print("        ARIA-4 FINAL WORKSPACE SAFETY BENCHMARK")
    print("============================================================")

    print(
        f"Total test cases : {len(TESTS)}"
    )

    print(
        "Safety architecture:"
    )

    print(
        "Requested XYZ -> Workspace Gate -> DLS"
    )

    print("============================================================")


    # ========================================================
    # CONNECT
    # ========================================================

    client = RemoteAPIClient()

    sim = client.require("sim")

    print(
        "\nConnected to CoppeliaSim."
    )


    joints = [

        sim.getObject(path)

        for path in JOINT_PATHS

    ]


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

    start_time = time.time()


    # ========================================================
    # TEST LOOP
    # ========================================================

    for (
        test_id,
        category,
        x,
        y,
        z,
        expected_status
    ) in TESTS:


        print("\n============================================================")

        print(
            f"{test_id} | {category}"
        )

        print(
            f"XYZ = "
            f"({x:.3f}, "
            f"{y:.3f}, "
            f"{z:.3f})"
        )

        print(
            f"Expected = "
            f"{expected_status}"
        )

        print("============================================================")


        # ====================================================
        # WORKSPACE GATE
        # ====================================================

        workspace = (
            WorkspaceValidator.validate(
                x,
                y,
                z
            )
        )


        # ====================================================
        # REJECT BEFORE IK
        # ====================================================

        if not workspace[
            "accepted"
        ]:

            actual_status = (
                "REJECTED_WORKSPACE"
            )

            dls_executed = False

            dls_success = False

            iterations = 0

            final_error_mm = (
                float("nan")
            )

            print(
                "Workspace gate : REJECTED"
            )

            print(
                "DLS execution  : BLOCKED"
            )

            for reason in workspace[
                "reasons"
            ]:

                print(
                    f"Reason         : "
                    f"{reason}"
                )


        # ====================================================
        # INSIDE WORKSPACE → RUN DLS
        # ====================================================

        else:

            print(
                "Workspace gate : PASSED"
            )

            reset_robot(
                sim,
                client,
                joints
            )

            dls_executed = True


            try:

                result = dls.solve(
                    [x, y, z]
                )

                dls_success = bool(
                    result["success"]
                )

                iterations = int(
                    result["iterations"]
                )

                final_error_mm = (
                    result["error"]
                    *
                    1000.0
                )


                if dls_success:

                    actual_status = (
                        "ACCEPTED"
                    )

                else:

                    actual_status = (
                        "REJECTED_UNREACHABLE"
                    )


            except Exception as error:

                print(
                    f"DLS ERROR: {error}"
                )

                dls_success = False

                iterations = 0

                final_error_mm = (
                    float("nan")
                )

                actual_status = (
                    "SOLVER_ERROR"
                )


        # ====================================================
        # CLASSIFICATION
        # ====================================================

        correct = (
            actual_status
            ==
            expected_status
        )


        print(
            f"Actual status  : "
            f"{actual_status}"
        )

        print(
            f"Correct        : "
            f"{correct}"
        )


        results.append({

            "test_id":
                test_id,

            "category":
                category,

            "target_x":
                x,

            "target_y":
                y,

            "target_z":
                z,

            "expected_status":
                expected_status,

            "workspace_pass":
                workspace["accepted"],

            "dls_executed":
                dls_executed,

            "dls_success":
                dls_success,

            "iterations":
                iterations,

            "final_error_mm":
                final_error_mm,

            "actual_status":
                actual_status,

            "classification_correct":
                correct,
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
    # METRICS
    # ========================================================

    total = len(results)

    correct_count = sum(

        1 for r in results

        if r[
            "classification_correct"
        ]
    )


    accuracy = (

        correct_count
        /
        total
        *
        100.0
    )


    # --------------------------------------------------------
    # Unsafe = anything expected to be rejected
    # --------------------------------------------------------

    unsafe = [

        r for r in results

        if r["expected_status"]
        != "ACCEPTED"

    ]


    unsafe_accepted = [

        r for r in unsafe

        if r["actual_status"]
        == "ACCEPTED"

    ]


    false_acceptance_rate = (

        len(unsafe_accepted)
        /
        len(unsafe)
        *
        100.0

        if unsafe

        else 0.0
    )


    # --------------------------------------------------------
    # Valid targets
    # --------------------------------------------------------

    valid = [

        r for r in results

        if r["expected_status"]
        == "ACCEPTED"

    ]


    valid_rejected = [

        r for r in valid

        if r["actual_status"]
        != "ACCEPTED"

    ]


    false_rejection_rate = (

        len(valid_rejected)
        /
        len(valid)
        *
        100.0

        if valid

        else 0.0
    )


    # --------------------------------------------------------
    # Ensure rejected targets never entered IK
    # --------------------------------------------------------

    rejected_cases = [

        r for r in results

        if r["expected_status"]
        == "REJECTED_WORKSPACE"

    ]


    blocked_before_ik = [

        r for r in rejected_cases

        if not r["dls_executed"]

    ]


    pre_ik_block_rate = (

        len(blocked_before_ik)
        /
        len(rejected_cases)
        *
        100.0

        if rejected_cases

        else 0.0
    )


    # ========================================================
    # PRINT SUMMARY
    # ========================================================

    elapsed = (
        time.time()
        -
        start_time
    )


    print("\n\n============================================================")
    print("              FINAL SAFETY RESULTS")
    print("============================================================")

    print(
        f"Total tests                   : "
        f"{total}"
    )

    print(
        f"Correct classifications       : "
        f"{correct_count}"
    )

    print(
        f"Overall classification accuracy: "
        f"{accuracy:.2f}%"
    )

    print(
        f"False acceptance rate         : "
        f"{false_acceptance_rate:.2f}%"
    )

    print(
        f"False rejection rate          : "
        f"{false_rejection_rate:.2f}%"
    )

    print(
        f"Unsafe targets blocked pre-IK : "
        f"{pre_ik_block_rate:.2f}%"
    )

    print(
        f"Total benchmark time          : "
        f"{elapsed:.3f} s"
    )

    print(
        f"\nResults saved to:\n"
        f"{OUTPUT_FILE}"
    )

    print("============================================================")


    # ========================================================
    # CATEGORY SUMMARY
    # ========================================================

    categories = sorted(
        set(
            r["category"]
            for r in results
        )
    )


    print("\nCATEGORY RESULTS")
    print("------------------------------------------------------------")


    for category in categories:

        category_results = [

            r for r in results

            if r["category"]
            == category

        ]

        category_correct = sum(

            1 for r in category_results

            if r[
                "classification_correct"
            ]
        )


        print(

            f"{category:<30} : "

            f"{category_correct}/"
            f"{len(category_results)} correct"
        )


    print("------------------------------------------------------------")


    # ========================================================
    # RESTORE
    # ========================================================

    reset_robot(
        sim,
        client,
        joints
    )

    print(
        "\nARIA-4 restored to zero configuration."
    )


if __name__ == "__main__":

    main()