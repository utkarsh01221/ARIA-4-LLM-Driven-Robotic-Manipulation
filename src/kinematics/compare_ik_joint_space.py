import csv
import math
from pathlib import Path

import numpy as np


BASE_DIR = Path(__file__).resolve().parent

DLS_FILE = BASE_DIR / "dls_batch_results.csv"
SIMIK_FILE = BASE_DIR / "simik_batch_results.csv"

OUTPUT_FILE = BASE_DIR / "ik_joint_comparison.csv"


def wrapped_angle_difference(q1, q2):
    """
    Shortest signed angular difference.
    Result is in [-pi, pi].
    """

    return math.atan2(
        math.sin(q1 - q2),
        math.cos(q1 - q2)
    )


def load_csv(path):

    with open(
        path,
        "r",
        newline=""
    ) as file:

        return list(
            csv.DictReader(file)
        )


def find_joint_key(row, joint_number):

    candidates = [
        f"q{joint_number}_rad",
        f"q{joint_number}",
        f"joint{joint_number}_rad",
        f"joint_{joint_number}_rad",
    ]

    for key in candidates:

        if key in row:
            return key

    raise KeyError(
        f"Could not find joint {joint_number} "
        f"column in CSV.\n"
        f"Available columns: {list(row.keys())}"
    )


def main():

    dls_rows = load_csv(DLS_FILE)
    simik_rows = load_csv(SIMIK_FILE)

    if len(dls_rows) != len(simik_rows):

        raise RuntimeError(
            "DLS and simIK result files contain "
            "different numbers of tests."
        )

    if not dls_rows:

        raise RuntimeError(
            "Input CSV files are empty."
        )

    dls_joint_keys = [
        find_joint_key(
            dls_rows[0],
            i
        )
        for i in range(1, 5)
    ]

    simik_joint_keys = [
        find_joint_key(
            simik_rows[0],
            i
        )
        for i in range(1, 5)
    ]

    results = []

    print("\n============================================================")
    print("        ARIA-4 DLS vs simIK JOINT-SPACE COMPARISON")
    print("============================================================")

    for index, (dls, simik) in enumerate(
        zip(dls_rows, simik_rows),
        start=1
    ):

        q_dls = np.array(
            [
                float(dls[key])
                for key in dls_joint_keys
            ]
        )

        q_simik = np.array(
            [
                float(simik[key])
                for key in simik_joint_keys
            ]
        )

        delta = np.array(
            [
                wrapped_angle_difference(
                    q_dls[i],
                    q_simik[i]
                )
                for i in range(4)
            ]
        )

        joint_error_rad = (
            np.linalg.norm(delta)
        )

        joint_error_deg = (
            math.degrees(joint_error_rad)
        )

        delta_deg = np.degrees(delta)

        print(
            f"\nTest {index:02d}"
        )

        print(
            "DLS   (deg):",
            np.round(
                np.degrees(q_dls),
                4
            )
        )

        print(
            "simIK (deg):",
            np.round(
                np.degrees(q_simik),
                4
            )
        )

        print(
            "Δq    (deg):",
            np.round(
                delta_deg,
                4
            )
        )

        print(
            f"||Δq|| = "
            f"{joint_error_deg:.6f} deg"
        )

        result = {
            "test": index,
        }

        for i in range(4):

            result[
                f"dls_q{i+1}_rad"
            ] = q_dls[i]

            result[
                f"simik_q{i+1}_rad"
            ] = q_simik[i]

            result[
                f"delta_q{i+1}_rad"
            ] = delta[i]

            result[
                f"delta_q{i+1}_deg"
            ] = delta_deg[i]

        result[
            "joint_space_error_rad"
        ] = joint_error_rad

        result[
            "joint_space_error_deg"
        ] = joint_error_deg

        results.append(result)

    errors_deg = np.array(
        [
            r["joint_space_error_deg"]
            for r in results
        ]
    )

    print("\n============================================================")
    print("             JOINT-SPACE SUMMARY")
    print("============================================================")

    print(
        f"Tests               : {len(results)}"
    )

    print(
        f"Mean ||Δq||         : "
        f"{np.mean(errors_deg):.6f} deg"
    )

    print(
        f"RMS ||Δq||          : "
        f"{np.sqrt(np.mean(errors_deg ** 2)):.6f} deg"
    )

    print(
        f"Minimum ||Δq||      : "
        f"{np.min(errors_deg):.6f} deg"
    )

    print(
        f"Maximum ||Δq||      : "
        f"{np.max(errors_deg):.6f} deg"
    )

    print(
        f"Standard deviation  : "
        f"{np.std(errors_deg):.6f} deg"
    )

    fieldnames = list(
        results[0].keys()
    )

    with open(
        OUTPUT_FILE,
        "w",
        newline=""
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames
        )

        writer.writeheader()
        writer.writerows(results)

    print(
        f"\nResults saved to:\n{OUTPUT_FILE}"
    )


if __name__ == "__main__":
    main()