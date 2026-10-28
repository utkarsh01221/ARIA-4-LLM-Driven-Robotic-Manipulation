from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import csv
import math
import time
from pathlib import Path


# ============================================================
# TEST TARGETS
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
# PATHS
# ============================================================

JOINT_PATHS = [

    "/base_link/Joint1",

    "/base_link/Joint1/Link1/Joint2",

    "/base_link/Joint1/Link1/Joint2/"
    "Link2/Joint3",

    "/base_link/Joint1/Link1/Joint2/"
    "Link2/Joint3/Link3/Joint4",
]


SCRIPT_PATH = (
    "/SimIK_Verification"
)


OUTPUT_FILE = (
    Path(__file__).resolve().parent
    / "simik_unreachable_verification.csv"
)


# ============================================================
# RESET ROBOT
# ============================================================

def reset_robot(
    sim,
    client,
    joints
):

    for joint in joints:

        sim.setJointPosition(
            joint,
            0.0
        )

    for _ in range(3):

        client.step()

        time.sleep(0.01)


# ============================================================
# MAIN
# ============================================================

def main():

    print("\n============================================================")
    print("      ARIA-4 simIK REACHABILITY CROSS-VERIFICATION")
    print("============================================================")

    print(
        f"Targets : {len(TARGETS)}"
    )

    print(
        "Convergence threshold for comparison: 5 mm"
    )

    print("============================================================")


    # ========================================================
    # CONNECT
    # ========================================================

    print(
        "\nConnecting to CoppeliaSim..."
    )

    client = RemoteAPIClient()

    sim = client.require(
        "sim"
    )

    print(
        "Connected."
    )


    # ========================================================
    # HANDLES
    # ========================================================

    joints = [

        sim.getObject(path)

        for path in JOINT_PATHS
    ]


    script = sim.getObject(
        SCRIPT_PATH
    )


    results = []


    # ========================================================
    # TEST LOOP
    # ========================================================

    for (
        test_number,
        target_data
    ) in enumerate(
        TARGETS,
        start=1
    ):

        name, x, y, z = target_data


        print("\n============================================================")

        print(
            f"TEST {test_number}/{len(TARGETS)}"
        )

        print(
            f"Target : {name}"
        )

        print(
            f"XYZ    : "
            f"({x:.3f}, "
            f"{y:.3f}, "
            f"{z:.3f})"
        )

        print("============================================================")


        # ====================================================
        # STANDARDIZED INITIAL CONDITION
        # ====================================================

        reset_robot(
            sim,
            client,
            joints
        )


        # ====================================================
        # CALL LUA simIK
        # ====================================================

        start = time.time()


        try:

            output = sim.callScriptFunction(
                "solveTarget",
                script,
                {
                    "x": float(x),
                    "y": float(y),
                    "z": float(z)
                }
            )


            elapsed = (
                time.time()
                -
                start
            )


            error_m = float(
                output["error"]
            )

            error_mm = (
                error_m
                *
                1000.0
            )


            # -----------------------------------------------
            # Use Cartesian residual as common comparison
            # criterion.
            # -----------------------------------------------

            converged_5mm = (
                error_m <= 0.005
            )


            print(
                f"simIK result code : "
                f"{output['result']}"
            )

            print(
                f"Final tip         : "
                f"("
                f"{output['tip_x']:.6f}, "
                f"{output['tip_y']:.6f}, "
                f"{output['tip_z']:.6f}"
                f")"
            )

            print(
                f"Final error       : "
                f"{error_mm:.3f} mm"
            )

            print(
                f"<= 5 mm           : "
                f"{converged_5mm}"
            )


            results.append({

                "test":
                    test_number,

                "target_name":
                    name,

                "target_x":
                    x,

                "target_y":
                    y,

                "target_z":
                    z,

                "simik_result_code":
                    output["result"],

                "simik_flags":
                    output["flags"],

                "tip_x":
                    output["tip_x"],

                "tip_y":
                    output["tip_y"],

                "tip_z":
                    output["tip_z"],

                "final_error_m":
                    error_m,

                "final_error_mm":
                    error_mm,

                "converged_5mm":
                    converged_5mm,

                "call_time_s":
                    elapsed,
            })


        except Exception as error:

            print(
                "\nsimIK VERIFICATION ERROR"
            )

            print(
                error
            )


            results.append({

                "test":
                    test_number,

                "target_name":
                    name,

                "target_x":
                    x,

                "target_y":
                    y,

                "target_z":
                    z,

                "simik_result_code":
                    -999,

                "simik_flags":
                    -999,

                "tip_x":
                    float("nan"),

                "tip_y":
                    float("nan"),

                "tip_z":
                    float("nan"),

                "final_error_m":
                    float("nan"),

                "final_error_mm":
                    float("nan"),

                "converged_5mm":
                    False,

                "call_time_s":
                    0.0,
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

    print("\n============================================================")
    print("             simIK VERIFICATION SUMMARY")
    print("============================================================")


    for result in results:

        if math.isfinite(
            result["final_error_mm"]
        ):

            print(

                f"{result['target_name']:<20} | "

                f"Error = "
                f"{result['final_error_mm']:.3f} mm | "

                f"<=5mm = "
                f"{result['converged_5mm']}"
            )

        else:

            print(

                f"{result['target_name']:<20} | "

                "ERROR"
            )


    print(
        f"\nResults saved to:\n"
        f"{OUTPUT_FILE}"
    )

    print("============================================================")


    # ========================================================
    # RESTORE
    # ========================================================

    reset_robot(
        sim,
        client,
        joints
    )

    print(
        "\nRobot restored to zero configuration."
    )


if __name__ == "__main__":

    main()