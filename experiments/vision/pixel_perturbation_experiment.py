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
    / "pixel_perturbation_results.csv"
)


# ============================================================
# CONTROLLED PIXEL OFFSETS
# ============================================================

PIXEL_OFFSETS = [
    -10,
    -5,
    -2,
    -1,
    0,
    1,
    2,
    5,
    10,
]


# ============================================================
# PIXEL -> WORLD
# Same model as baseline experiment
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
    print("       ARIA-4 CONTROLLED PIXEL PERTURBATION EXPERIMENT")
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
        f"Pixel offsets             : "
        f"{PIXEL_OFFSETS}"
    )


    results = []


    # ========================================================
    # EACH OBJECT
    # ========================================================

    for _, row in baseline.iterrows():

        color = row["color"]

        u0 = float(
            row["pixel_u"]
        )

        v0 = float(
            row["pixel_v"]
        )

        depth = float(
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
            f"Baseline centroid = "
            f"({u0:.3f}, {v0:.3f})"
        )
        print(
            f"Depth             = "
            f"{depth:.6f} m"
        )


        # ====================================================
        # U-AXIS PERTURBATION
        # ====================================================

        for offset in PIXEL_OFFSETS:

            u = (
                u0
                +
                offset
            )

            v = v0


            vision_x, vision_y = (
                pixel_to_world_xy(
                    u,
                    v,
                    depth
                )
            )


            error_x_mm = (
                vision_x
                -
                gt_x
            ) * 1000.0


            error_y_mm = (
                vision_y
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

                "axis":
                    "U",

                "pixel_offset":
                    offset,

                "perturbed_u":
                    u,

                "perturbed_v":
                    v,

                "depth_m":
                    depth,

                "gt_x":
                    gt_x,

                "gt_y":
                    gt_y,

                "estimated_x":
                    vision_x,

                "estimated_y":
                    vision_y,

                "error_x_mm":
                    error_x_mm,

                "error_y_mm":
                    error_y_mm,

                "error_xy_mm":
                    error_xy_mm,
            })


        # ====================================================
        # V-AXIS PERTURBATION
        # ====================================================

        for offset in PIXEL_OFFSETS:

            u = u0

            v = (
                v0
                +
                offset
            )


            vision_x, vision_y = (
                pixel_to_world_xy(
                    u,
                    v,
                    depth
                )
            )


            error_x_mm = (
                vision_x
                -
                gt_x
            ) * 1000.0


            error_y_mm = (
                vision_y
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

                "axis":
                    "V",

                "pixel_offset":
                    offset,

                "perturbed_u":
                    u,

                "perturbed_v":
                    v,

                "depth_m":
                    depth,

                "gt_x":
                    gt_x,

                "gt_y":
                    gt_y,

                "estimated_x":
                    vision_x,

                "estimated_y":
                    vision_y,

                "error_x_mm":
                    error_x_mm,

                "error_y_mm":
                    error_y_mm,

                "error_xy_mm":
                    error_xy_mm,
            })


    # ========================================================
    # DATAFRAME
    # ========================================================

    results_df = pd.DataFrame(
        results
    )


    results_df.to_csv(
        OUTPUT_FILE,
        index=False
    )


    # ========================================================
    # SUMMARY BY ABSOLUTE PIXEL OFFSET
    # ========================================================

    results_df[
        "absolute_pixel_offset"
    ] = results_df[
        "pixel_offset"
    ].abs()


    print()
    print("=" * 72)
    print("                  PERTURBATION SUMMARY")
    print("=" * 72)


    summary = (

        results_df

        .groupby(
            "absolute_pixel_offset"
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
        summary.to_string(
            index=False
        )
    )


    # ========================================================
    # ERROR INCREASE RELATIVE TO ZERO-OFFSET BASELINE
    # ========================================================

    print()
    print("=" * 72)
    print("        PIXEL DISPLACEMENT SCALE BY OBJECT")
    print("=" * 72)


    for _, row in baseline.iterrows():

        color = row["color"]

        depth = float(
            row["camera_depth_m"]
        )


        mm_per_pixel_x = (
            depth
            /
            FX
            *
            1000.0
        )


        mm_per_pixel_y = (
            depth
            /
            FY
            *
            1000.0
        )


        print(
            f"{color:<8} | "
            f"depth={depth:.6f} m | "
            f"X={mm_per_pixel_x:.3f} mm/pixel | "
            f"Y={mm_per_pixel_y:.3f} mm/pixel"
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