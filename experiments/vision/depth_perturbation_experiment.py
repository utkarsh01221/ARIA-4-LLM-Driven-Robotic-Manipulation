import pandas as pd
import numpy as np
import math
from pathlib import Path


# ============================================================
# ARIA-4 CAMERA PARAMETERS
# ============================================================

FX = 443.405
FY = 443.405

CX = 256.0
CY = 256.0

CAMERA_X = 1.2
CAMERA_Y = -0.2


BASE_DIR = Path(__file__).resolve().parent

INPUT_FILE = (
    BASE_DIR
    / "vision_baseline_results.csv"
)

OUTPUT_FILE = (
    BASE_DIR
    / "depth_perturbation_results.csv"
)


# ============================================================
# CONTROLLED DEPTH ERRORS
#
# Values stored in millimetres
# ============================================================

DEPTH_OFFSETS_MM = [
    -20,
    -10,
    -5,
    -2,
    0,
    2,
    5,
    10,
    20,
]


# ============================================================
# PIXEL -> WORLD XY
# ============================================================

def pixel_to_world_xy(
    u,
    v,
    depth
):

    x_camera = -(
        (u - CX)
        *
        depth
        /
        FX
    )

    y_camera = -(
        (v - CY)
        *
        depth
        /
        FY
    )

    x_world = (
        CAMERA_X
        +
        x_camera
    )

    y_world = (
        CAMERA_Y
        -
        y_camera
    )

    return (
        x_world,
        y_world
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("=" * 72)
    print("       ARIA-4 CONTROLLED DEPTH PERTURBATION EXPERIMENT")
    print("=" * 72)

    baseline = pd.read_csv(
        INPUT_FILE
    )

    baseline = baseline[
        baseline["detected"] == True
    ].copy()


    print(
        f"\nDetected baseline objects : "
        f"{len(baseline)}"
    )

    print(
        f"Depth offsets (mm)        : "
        f"{DEPTH_OFFSETS_MM}"
    )


    results = []


    # ========================================================
    # EACH OBJECT
    # ========================================================

    for _, row in baseline.iterrows():

        color = row["color"]

        u = float(
            row["pixel_u"]
        )

        v = float(
            row["pixel_v"]
        )

        baseline_depth = float(
            row["camera_depth_m"]
        )

        gt_x = float(
            row["gt_x"]
        )

        gt_y = float(
            row["gt_y"]
        )


        print()
        print("-" * 72)

        print(
            f"OBJECT: {color.upper()}"
        )

        print(
            f"Centroid       = "
            f"({u:.3f}, {v:.3f})"
        )

        print(
            f"Baseline depth = "
            f"{baseline_depth:.6f} m"
        )


        # ====================================================
        # DEPTH PERTURBATIONS
        # ====================================================

        for offset_mm in DEPTH_OFFSETS_MM:

            offset_m = (
                offset_mm
                /
                1000.0
            )


            perturbed_depth = (
                baseline_depth
                +
                offset_m
            )


            estimated_x, estimated_y = (
                pixel_to_world_xy(
                    u,
                    v,
                    perturbed_depth
                )
            )


            error_x_mm = (
                estimated_x
                -
                gt_x
            ) * 1000.0


            error_y_mm = (
                estimated_y
                -
                gt_y
            ) * 1000.0


            error_xy_mm = math.sqrt(
                error_x_mm ** 2
                +
                error_y_mm ** 2
            )


            results.append({

                "color":
                    color,

                "depth_offset_mm":
                    offset_mm,

                "baseline_depth_m":
                    baseline_depth,

                "perturbed_depth_m":
                    perturbed_depth,

                "pixel_u":
                    u,

                "pixel_v":
                    v,

                "gt_x":
                    gt_x,

                "gt_y":
                    gt_y,

                "estimated_x":
                    estimated_x,

                "estimated_y":
                    estimated_y,

                "error_x_mm":
                    error_x_mm,

                "error_y_mm":
                    error_y_mm,

                "error_xy_mm":
                    error_xy_mm,
            })


    # ========================================================
    # SAVE
    # ========================================================

    results_df = pd.DataFrame(
        results
    )


    results_df.to_csv(
        OUTPUT_FILE,
        index=False
    )


    # ========================================================
    # SUMMARY
    # ========================================================

    results_df[
        "absolute_depth_offset_mm"
    ] = results_df[
        "depth_offset_mm"
    ].abs()


    summary = (

        results_df

        .groupby(
            "absolute_depth_offset_mm"
        )

        ["error_xy_mm"]

        .agg([
            "mean",
            "min",
            "max"
        ])

        .reset_index()
    )


    print()
    print("=" * 72)
    print("                  DEPTH ERROR SUMMARY")
    print("=" * 72)

    print()

    print(
        summary.to_string(
            index=False
        )
    )


    # ========================================================
    # POSITIVE / NEGATIVE SUMMARY
    # ========================================================

    print()
    print("=" * 72)
    print("             SIGNED DEPTH PERTURBATION RESULTS")
    print("=" * 72)


    signed_summary = (

        results_df

        .groupby(
            "depth_offset_mm"
        )

        ["error_xy_mm"]

        .agg([
            "mean",
            "min",
            "max"
        ])

        .reset_index()
    )


    print()

    print(
        signed_summary.to_string(
            index=False
        )
    )


    print()

    print(
        f"Total perturbation trials : "
        f"{len(results_df)}"
    )

    print(
        f"\nResults saved to:\n"
        f"{OUTPUT_FILE}"
    )

    print("=" * 72)


if __name__ == "__main__":
    main()