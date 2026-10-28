import pandas as pd
import numpy as np
import math
from pathlib import Path


# ============================================================
# CAMERA
# ============================================================

FX = 443.405
FY = 443.405

CX = 256.0
CY = 256.0

CAMERA_X = 1.2
CAMERA_Y = -0.2


# ============================================================
# EXPERIMENT
# ============================================================

TRIALS_PER_OBJECT = 100

RANDOM_SEED = 42


NOISE_LEVELS = {

    "L0_BASELINE": {
        "pixel_sigma": 0.0,
        "depth_sigma_mm": 0.0,
    },

    "L1_LOW": {
        "pixel_sigma": 0.5,
        "depth_sigma_mm": 2.0,
    },

    "L2_MODERATE": {
        "pixel_sigma": 1.0,
        "depth_sigma_mm": 5.0,
    },

    "L3_HIGH": {
        "pixel_sigma": 2.0,
        "depth_sigma_mm": 10.0,
    },

    "L4_SEVERE": {
        "pixel_sigma": 5.0,
        "depth_sigma_mm": 20.0,
    },
}


BASE_DIR = Path(__file__).resolve().parent

INPUT_FILE = (
    BASE_DIR
    / "vision_baseline_results.csv"
)

OUTPUT_FILE = (
    BASE_DIR
    / "monte_carlo_perception_results.csv"
)


# ============================================================
# PROJECTION
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

    return x_world, y_world


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("=" * 72)
    print("       ARIA-4 MONTE CARLO PERCEPTION ROBUSTNESS")
    print("=" * 72)

    print(
        f"Random seed       : {RANDOM_SEED}"
    )

    print(
        f"Trials/object/level: {TRIALS_PER_OBJECT}"
    )


    rng = np.random.default_rng(
        RANDOM_SEED
    )


    baseline = pd.read_csv(
        INPUT_FILE
    )


    baseline = baseline[
        baseline["detected"] == True
    ].copy()


    results = []


    # ========================================================
    # NOISE LEVEL
    # ========================================================

    for level, noise in NOISE_LEVELS.items():

        pixel_sigma = (
            noise["pixel_sigma"]
        )

        depth_sigma_mm = (
            noise["depth_sigma_mm"]
        )


        print()
        print("-" * 72)

        print(
            f"{level}"
        )

        print(
            f"Pixel sigma = "
            f"{pixel_sigma:.2f} px"
        )

        print(
            f"Depth sigma = "
            f"{depth_sigma_mm:.2f} mm"
        )


        # ====================================================
        # OBJECT
        # ====================================================

        for _, row in baseline.iterrows():

            color = row["color"]

            u0 = float(
                row["pixel_u"]
            )

            v0 = float(
                row["pixel_v"]
            )

            depth0 = float(
                row["camera_depth_m"]
            )

            gt_x = float(
                row["gt_x"]
            )

            gt_y = float(
                row["gt_y"]
            )


            # ================================================
            # TRIALS
            # ================================================

            for trial in range(
                1,
                TRIALS_PER_OBJECT + 1
            ):

                # --------------------------------------------
                # Random perturbations
                # --------------------------------------------

                du = rng.normal(
                    0.0,
                    pixel_sigma
                )

                dv = rng.normal(
                    0.0,
                    pixel_sigma
                )

                dz_mm = rng.normal(
                    0.0,
                    depth_sigma_mm
                )


                u = (
                    u0
                    +
                    du
                )

                v = (
                    v0
                    +
                    dv
                )

                depth = (
                    depth0
                    +
                    dz_mm / 1000.0
                )


                # --------------------------------------------
                # Projection
                # --------------------------------------------

                estimated_x, estimated_y = (
                    pixel_to_world_xy(
                        u,
                        v,
                        depth
                    )
                )


                # --------------------------------------------
                # Error
                # --------------------------------------------

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

                    "noise_level":
                        level,

                    "color":
                        color,

                    "trial":
                        trial,

                    "pixel_sigma":
                        pixel_sigma,

                    "depth_sigma_mm":
                        depth_sigma_mm,

                    "du_px":
                        du,

                    "dv_px":
                        dv,

                    "dz_mm":
                        dz_mm,

                    "perturbed_u":
                        u,

                    "perturbed_v":
                        v,

                    "perturbed_depth_m":
                        depth,

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

    df = pd.DataFrame(
        results
    )


    df.to_csv(
        OUTPUT_FILE,
        index=False
    )


    # ========================================================
    # SUMMARY
    # ========================================================

    print()
    print("=" * 72)
    print("                    MONTE CARLO SUMMARY")
    print("=" * 72)


    for level in NOISE_LEVELS:

        subset = df[
            df["noise_level"]
            ==
            level
        ]


        errors = subset[
            "error_xy_mm"
        ].to_numpy()


        mean_error = np.mean(
            errors
        )


        rms_error = math.sqrt(
            np.mean(
                errors ** 2
            )
        )


        median_error = np.median(
            errors
        )


        p95_error = np.percentile(
            errors,
            95
        )


        max_error = np.max(
            errors
        )


        within_5 = (
            np.mean(
                errors <= 5.0
            )
            *
            100.0
        )


        within_10 = (
            np.mean(
                errors <= 10.0
            )
            *
            100.0
        )


        print()
        print(
            f"{level}"
        )

        print(
            f"  Trials       : "
            f"{len(errors)}"
        )

        print(
            f"  Mean error   : "
            f"{mean_error:.3f} mm"
        )

        print(
            f"  RMS error    : "
            f"{rms_error:.3f} mm"
        )

        print(
            f"  Median error : "
            f"{median_error:.3f} mm"
        )

        print(
            f"  95th pct.    : "
            f"{p95_error:.3f} mm"
        )

        print(
            f"  Maximum      : "
            f"{max_error:.3f} mm"
        )

        print(
            f"  <= 5 mm      : "
            f"{within_5:.2f}%"
        )

        print(
            f"  <= 10 mm     : "
            f"{within_10:.2f}%"
        )


    print()
    print("=" * 72)

    print(
        f"Total trials : "
        f"{len(df)}"
    )

    print(
        f"\nResults saved to:\n"
        f"{OUTPUT_FILE}"
    )

    print("=" * 72)


if __name__ == "__main__":
    main()