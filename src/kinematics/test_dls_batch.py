from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from dls_ik import DLSInverseKinematics
import csv
import math
import time

JOINT_PATHS = [
    "/base_link/Joint1",
    "/base_link/Joint1/Link1/Joint2",
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3",
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3/Link3/Joint4",
]

TIP_PATH = (
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3/"
    "Link3/Joint4/Link4/IK_Tip"
)

DAMPING = 0.5
MAX_ITERATIONS = 100
TOLERANCE = 0.005
NUMERICAL_STEP = 1e-5

TARGETS = [
    (0.800, -0.250, 0.385),
    (0.800, -0.200, 0.385),
    (0.750, -0.300, 0.385),
    (0.900, -0.150, 0.400),
    (0.850, -0.350, 0.420),
    (0.780, -0.150, 0.400),
    (0.820, -0.300, 0.400),
    (0.740, -0.200, 0.410),
    (0.880, -0.250, 0.390),
    (0.900, -0.300, 0.420),
    (0.760, -0.350, 0.390),
    (0.840, -0.150, 0.420),
    (0.780, -0.300, 0.430),
    (0.860, -0.200, 0.410),
    (0.750, -0.250, 0.400),
    (0.820, -0.350, 0.430),
    (0.880, -0.350, 0.400),
    (0.760, -0.150, 0.420),
    (0.840, -0.300, 0.390),
    (0.900, -0.200, 0.430),
]

CSV_FILE = "dls_batch_results.csv"


def validate_targets():
    for i, target in enumerate(TARGETS, 1):
        if len(target) != 3 or not all(math.isfinite(v) for v in target):
            raise ValueError(f"Invalid target {i}: {target}")

        x, y, z = target

        if not (0.70 <= x <= 1.05):
            raise ValueError(f"Target {i} X={x:.3f} outside safe X range.")
        if not (-0.60 <= y <= 0.00):
            raise ValueError(f"Target {i} Y={y:.3f} outside safe Y range.")
        if not (0.30 <= z <= 0.50):
            raise ValueError(f"Target {i} Z={z:.3f} outside tested Z range.")


def save_results(results):
    fields = [
        "test", "target_x", "target_y", "target_z",
        "success", "iterations",
        "final_error_m", "final_error_mm", "time_s",
        "q1_rad", "q1_deg", "q2_rad", "q2_deg",
        "q3_rad", "q3_deg", "q4_rad", "q4_deg",
    ]

    with open(CSV_FILE, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)

    print(f"\nResults saved to: {CSV_FILE}")


def print_summary(results, total_time):
    successful = [r for r in results if r["success"]]
    valid_errors = [r["final_error_m"] for r in results if math.isfinite(r["final_error_m"])]
    valid_iterations = [r["iterations"] for r in results if r["iterations"] > 0]
    valid_times = [r["time_s"] for r in results if r["time_s"] > 0]

    total = len(results)
    success_count = len(successful)

    success_rate = success_count / total * 100 if total else 0
    mean_error = sum(valid_errors) / len(valid_errors) if valid_errors else 0
    mean_success_error = (
        sum(r["final_error_m"] for r in successful) / len(successful)
        if successful else 0
    )
    mean_iterations = (
        sum(valid_iterations) / len(valid_iterations)
        if valid_iterations else 0
    )
    mean_time = sum(valid_times) / len(valid_times) if valid_times else 0

    print("\n============================================================")
    print("                  DLS BATCH TEST SUMMARY")
    print("============================================================")
    print(f"Total tests             : {total}")
    print(f"Successful tests        : {success_count}")
    print(f"Failed tests            : {total - success_count}")
    print(f"Success rate            : {success_rate:.2f}%")
    print(f"Mean final error        : {mean_error * 1000:.3f} mm")
    print(f"Mean successful error   : {mean_success_error * 1000:.3f} mm")
    if valid_errors:
        print(f"Minimum final error     : {min(valid_errors) * 1000:.3f} mm")
        print(f"Maximum final error     : {max(valid_errors) * 1000:.3f} mm")
    print(f"Mean iterations         : {mean_iterations:.2f}")
    print(f"Mean solver time        : {mean_time:.3f} s")
    print(f"Total batch time        : {total_time:.3f} s")
    print("============================================================")

    print("\nPer-test results:")
    print(f"{'Test':<6}{'Success':<10}{'Iter':<8}{'Error(mm)':<12}{'Time(s)':<10}")
    print("-" * 46)

    for r in results:
        error_text = (
            f"{r['final_error_mm']:.3f}"
            if math.isfinite(r["final_error_mm"])
            else "N/A"
        )
        print(
            f"{r['test']:<6}"
            f"{str(r['success']):<10}"
            f"{r['iterations']:<8}"
            f"{error_text:<12}"
            f"{r['time_s']:<10.3f}"
        )


def main():
    print("============================================================")
    print("              ARIA-4 DLS BATCH EXPERIMENT")
    print("============================================================")
    print(f"Number of targets    : {len(TARGETS)}")
    print(f"Damping lambda       : {DAMPING}")
    print(f"Maximum iterations   : {MAX_ITERATIONS}")
    print(f"Tolerance            : {TOLERANCE} m")
    print(f"Jacobian step        : {NUMERICAL_STEP}")
    print("============================================================")

    validate_targets()
    print("\nTarget validation passed.")
    print("\nConnecting to CoppeliaSim...")

    client = RemoteAPIClient()
    sim = client.require("sim")
    print("Connected to CoppeliaSim.")

    dls = DLSInverseKinematics(
        sim=sim,
        client=client,
        joint_paths=JOINT_PATHS,
        tip_path=TIP_PATH,
        damping=DAMPING,
        max_iterations=MAX_ITERATIONS,
        tolerance=TOLERANCE,
        numerical_step=NUMERICAL_STEP,
    )

    results = []
    batch_start = time.time()

    for test_number, target in enumerate(TARGETS, 1):
        print("\n============================================================")
        print(f"                    TEST {test_number:02d}/{len(TARGETS)}")
        print("============================================================")
        print(f"Target = ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")

        try:
            result = dls.solve(target)
            q = result["joint_positions"]

            results.append({
                "test": test_number,
                "target_x": target[0],
                "target_y": target[1],
                "target_z": target[2],
                "success": result["success"],
                "iterations": result["iterations"],
                "final_error_m": result["error"],
                "final_error_mm": result["error"] * 1000.0,
                "time_s": result["time"],
                "q1_rad": q[0], "q1_deg": math.degrees(q[0]),
                "q2_rad": q[1], "q2_deg": math.degrees(q[1]),
                "q3_rad": q[2], "q3_deg": math.degrees(q[2]),
                "q4_rad": q[3], "q4_deg": math.degrees(q[3]),
            })

        except Exception as e:
            print("\nDLS TEST ERROR")
            print(f"Test {test_number} failed: {e}")

            results.append({
                "test": test_number,
                "target_x": target[0],
                "target_y": target[1],
                "target_z": target[2],
                "success": False,
                "iterations": 0,
                "final_error_m": float("nan"),
                "final_error_mm": float("nan"),
                "time_s": 0.0,
                "q1_rad": float("nan"), "q1_deg": float("nan"),
                "q2_rad": float("nan"), "q2_deg": float("nan"),
                "q3_rad": float("nan"), "q3_deg": float("nan"),
                "q4_rad": float("nan"), "q4_deg": float("nan"),
            })

    total_time = time.time() - batch_start

    save_results(results)
    print_summary(results, total_time)

    print("\n============================================================")
    print("                 BATCH EXPERIMENT COMPLETE")
    print("============================================================")


if __name__ == "__main__":
    main()