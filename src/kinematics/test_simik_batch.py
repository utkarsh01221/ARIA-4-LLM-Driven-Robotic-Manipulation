from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pathlib import Path
import csv
import time
import math


# ============================================================
# SAME 20 TARGETS USED FOR DLS EXPERIMENT
# ============================================================

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


# ============================================================
# EXPERIMENT PARAMETERS
# ============================================================

TOLERANCE = 0.005       # 5 mm
TIMEOUT = 20.0          # seconds

CSV_FILE = Path(__file__).resolve().parent / "simik_batch_results.csv"


# ============================================================
# CONNECT TO COPPELIASIM
# ============================================================

print("========================================")
print("       ARIA-4 simIK BATCH TEST")
print("========================================")

client = RemoteAPIClient()
sim = client.require("sim")

target = sim.getObject("/IK_Target")

tip = sim.getObject(
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3/Link3/Joint4/Link4/IK_Tip"
)

joints = [
    sim.getObject("/base_link/Joint1"),
    sim.getObject("/base_link/Joint1/Link1/Joint2"),
    sim.getObject("/base_link/Joint1/Link1/Joint2/Link2/Joint3"),
    sim.getObject("/base_link/Joint1/Link1/Joint2/Link2/Link3/Joint4"),
]

print("Connected to CoppeliaSim")
print(f"Target handle : {target}")
print(f"Tip handle    : {tip}")
print(f"Joints        : {len(joints)}")
print(f"Tolerance     : {TOLERANCE * 1000:.1f} mm")
print(f"Timeout       : {TIMEOUT:.1f} s")


# ============================================================
# FUNCTIONS
# ============================================================

def get_tip_position():
    return sim.getObjectPosition(tip, -1)


def get_target_position():
    return sim.getObjectPosition(target, -1)


def calculate_error(target_position, tip_position):

    dx = target_position[0] - tip_position[0]
    dy = target_position[1] - tip_position[1]
    dz = target_position[2] - tip_position[2]

    return math.sqrt(dx * dx + dy * dy + dz * dz)


def get_joint_positions():

    return [
        sim.getJointPosition(joint)
        for joint in joints
    ]


def rad_to_deg(value):
    return value * 180.0 / math.pi


# ============================================================
# START SIMULATION
# ============================================================

sim.startSimulation()

time.sleep(1)

client.step()


# ============================================================
# CSV HEADER
# ============================================================

fieldnames = [
    "test",
    "target_x",
    "target_y",
    "target_z",
    "success",
    "convergence_steps",
    "final_error_m",
    "final_error_mm",
    "time_s",
    "q1_rad",
    "q2_rad",
    "q3_rad",
    "q4_rad",
    "q1_deg",
    "q2_deg",
    "q3_deg",
    "q4_deg",
]


with open(
    CSV_FILE,
    "w",
    newline="",
    encoding="utf-8"
) as csvfile:

    writer = csv.DictWriter(
        csvfile,
        fieldnames=fieldnames
    )

    writer.writeheader()


    # ========================================================
    # BATCH TEST
    # ========================================================

    results = []

    print("\n============================================================")
    print("                 STARTING 20-TEST BATCH")
    print("============================================================")


    for index, target_position in enumerate(TARGETS, start=1):

        print("\n--------------------------------------------")
        print(f"TEST {index:02d} / {len(TARGETS)}")
        print("--------------------------------------------")

        x, y, z = target_position

        print(
            f"Target = "
            f"({x:.3f}, {y:.3f}, {z:.3f})"
        )


        # ----------------------------------------------------
        # Set IK target
        # ----------------------------------------------------

        sim.setObjectPosition(
            target,
            -1,
            [x, y, z]
        )

        client.step()


        # ----------------------------------------------------
        # Wait for simIK convergence
        # ----------------------------------------------------

        start_time = time.time()

        success = False
        convergence_steps = 0
        final_error = None

        while True:

            client.step()

            tip_position = get_tip_position()

            final_error = calculate_error(
                target_position,
                tip_position
            )

            convergence_steps += 1

            if final_error <= TOLERANCE:

                success = True
                break


            if time.time() - start_time >= TIMEOUT:

                break


            time.sleep(0.01)


        elapsed = time.time() - start_time


        # ----------------------------------------------------
        # Final values
        # ----------------------------------------------------

        tip_position = get_tip_position()

        final_error = calculate_error(
            target_position,
            tip_position
        )

        q = get_joint_positions()


        # ----------------------------------------------------
        # Print result
        # ----------------------------------------------------

        print(
            f"Success       : {success}"
        )

        print(
            f"Steps         : {convergence_steps}"
        )

        print(
            f"Final error   : "
            f"{final_error * 1000:.3f} mm"
        )

        print(
            f"Time          : "
            f"{elapsed:.3f} s"
        )

        print(
            f"q = "
            f"[{q[0]:.4f}, "
            f"{q[1]:.4f}, "
            f"{q[2]:.4f}, "
            f"{q[3]:.4f}]"
        )


        # ----------------------------------------------------
        # Save result immediately
        # ----------------------------------------------------

        row = {
            "test": index,

            "target_x": x,
            "target_y": y,
            "target_z": z,

            "success": success,

            "convergence_steps": convergence_steps,

            "final_error_m": final_error,

            "final_error_mm":
                final_error * 1000.0,

            "time_s": elapsed,

            "q1_rad": q[0],
            "q2_rad": q[1],
            "q3_rad": q[2],
            "q4_rad": q[3],

            "q1_deg": rad_to_deg(q[0]),
            "q2_deg": rad_to_deg(q[1]),
            "q3_deg": rad_to_deg(q[2]),
            "q4_deg": rad_to_deg(q[3]),
        }

        writer.writerow(row)
        csvfile.flush()

        results.append(row)


        # ----------------------------------------------------
        # Small pause before next target
        # ----------------------------------------------------

        time.sleep(0.5)


# ============================================================
# STOP SIMULATION
# ============================================================

sim.stopSimulation()


# ============================================================
# SUMMARY
# ============================================================

successful = [
    r for r in results
    if r["success"]
]

errors = [
    r["final_error_mm"]
    for r in results
]

times = [
    r["time_s"]
    for r in results
]

steps = [
    r["convergence_steps"]
    for r in results
]


print("\n============================================================")
print("                  simIK BATCH TEST SUMMARY")
print("============================================================")

print(
    f"Total tests             : {len(results)}"
)

print(
    f"Successful tests        : {len(successful)}"
)

print(
    f"Failed tests            : "
    f"{len(results) - len(successful)}"
)

print(
    f"Success rate            : "
    f"{len(successful) / len(results) * 100:.2f}%"
)

print(
    f"Mean final error        : "
    f"{sum(errors) / len(errors):.3f} mm"
)

print(
    f"Minimum final error     : "
    f"{min(errors):.3f} mm"
)

print(
    f"Maximum final error     : "
    f"{max(errors):.3f} mm"
)

print(
    f"Mean convergence steps  : "
    f"{sum(steps) / len(steps):.2f}"
)

print(
    f"Mean execution time     : "
    f"{sum(times) / len(times):.3f} s"
)

print(
    f"Total batch time        : "
    f"{sum(times):.3f} s"
)

print("============================================================")

print("\nPer-test results:")
print(
    "Test  Success   Steps   Error(mm)   Time(s)"
)
print("----------------------------------------------")

for r in results:

    print(
        f"{r['test']:<5}"
        f"{str(r['success']):<10}"
        f"{r['convergence_steps']:<8}"
        f"{r['final_error_mm']:<12.3f}"
        f"{r['time_s']:.3f}"
    )


print("\n============================================================")
print("             simIK BATCH EXPERIMENT COMPLETE")
print("============================================================")

print(
    f"Results saved to:\n{CSV_FILE}"
)