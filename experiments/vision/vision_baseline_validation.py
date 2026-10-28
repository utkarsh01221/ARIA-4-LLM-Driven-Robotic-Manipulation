from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import numpy as np
import cv2
import csv
import math
import time
from pathlib import Path


# ============================================================
# ARIA-4 CAMERA PARAMETERS
# ============================================================

IMAGE_WIDTH = 512
IMAGE_HEIGHT = 512

FX = 443.405
FY = 443.405

CX = 256.0
CY = 256.0


# Camera world position
CAMERA_X = 1.2
CAMERA_Y = -0.2
CAMERA_Z = 1.5


# ============================================================
# OBJECTS
# ============================================================

OBJECTS = {

    "red": "/red_cube",

    "green": "/green_cube",

    "blue": "/blue_cube",

    "yellow": "/yellow_cube",
}


VISION_SENSOR_PATH = "/visionSensor"


OUTPUT_FILE = (
    Path(__file__).resolve().parent
    / "vision_baseline_results.csv"
)


# ============================================================
# HSV COLOR RANGES
#
# These are intentionally broad initial ranges.
# We will inspect the actual detection results before tuning.
# ============================================================

HSV_RANGES = {

    "red": [
        (
            np.array([0, 80, 70]),
            np.array([10, 255, 255])
        ),

        (
            np.array([170, 80, 70]),
            np.array([179, 255, 255])
        ),
    ],

    "green": [
        (
            np.array([35, 60, 50]),
            np.array([90, 255, 255])
        ),
    ],

    "blue": [
        (
            np.array([90, 60, 50]),
            np.array([140, 255, 255])
        ),
    ],

    "yellow": [
        (
            np.array([20, 80, 80]),
            np.array([35, 255, 255])
        ),
    ],
}


# ============================================================
# GET VISION SENSOR IMAGE
# ============================================================

def get_camera_image(sim, sensor):

    image, resolution = sim.getVisionSensorImg(
        sensor
    )

    image = np.frombuffer(
        image,
        dtype=np.uint8
    )

    image = image.reshape(
        resolution[1],
        resolution[0],
        3
    )

    # CoppeliaSim image is vertically flipped
    image = cv2.flip(
        image,
        0
    )

    return image


# ============================================================
# COLOR DETECTION
# ============================================================

def detect_color_centroid(
    image_rgb,
    color_name
):

    # CoppeliaSim gives RGB.
    # OpenCV HSV conversion therefore uses RGB2HSV.

    hsv = cv2.cvtColor(
        image_rgb,
        cv2.COLOR_RGB2HSV
    )


    mask_total = np.zeros(
        hsv.shape[:2],
        dtype=np.uint8
    )


    for lower, upper in HSV_RANGES[
        color_name
    ]:

        mask = cv2.inRange(
            hsv,
            lower,
            upper
        )

        mask_total = cv2.bitwise_or(
            mask_total,
            mask
        )


    # Mild cleanup
    kernel = np.ones(
        (5, 5),
        np.uint8
    )

    mask_total = cv2.morphologyEx(
        mask_total,
        cv2.MORPH_OPEN,
        kernel
    )

    mask_total = cv2.morphologyEx(
        mask_total,
        cv2.MORPH_CLOSE,
        kernel
    )


    contours, _ = cv2.findContours(
        mask_total,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )


    if not contours:

        return None


    # Largest region for this color
    contour = max(
        contours,
        key=cv2.contourArea
    )


    area = cv2.contourArea(
        contour
    )


    if area < 20:

        return None


    moments = cv2.moments(
        contour
    )


    if moments["m00"] == 0:

        return None


    u = (
        moments["m10"]
        /
        moments["m00"]
    )

    v = (
        moments["m01"]
        /
        moments["m00"]
    )


    return {

        "u": float(u),

        "v": float(v),

        "area_px":
            float(area),

        "mask":
            mask_total,

        "contour":
            contour
    }


# ============================================================
# PIXEL -> WORLD XY
#
# ARIA-4 downward-facing camera model
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
    print("=" * 68)
    print("       ARIA-4 VISION BASELINE LOCALIZATION VALIDATION")
    print("=" * 68)

    print(
        "Metric: vision-estimated XY versus "
        "CoppeliaSim ground-truth XY"
    )

    print("=" * 68)


    # ========================================================
    # CONNECT
    # ========================================================

    client = RemoteAPIClient()

    sim = client.require(
        "sim"
    )

    print(
        "\nConnected to CoppeliaSim."
    )


    sensor = sim.getObject(
        VISION_SENSOR_PATH
    )


    object_handles = {

        color:
            sim.getObject(path)

        for color, path
        in OBJECTS.items()
    }


    # ========================================================
    # ALLOW CAMERA TO UPDATE
    # ========================================================

    for _ in range(5):

        client.step()

        time.sleep(0.02)


    # ========================================================
    # CAPTURE ONE IMAGE
    # ========================================================

    image = get_camera_image(
        sim,
        sensor
    )


    print(
        f"Captured image shape: "
        f"{image.shape}"
    )


    # Visualization copy
    display = cv2.cvtColor(
        image,
        cv2.COLOR_RGB2BGR
    )


    results = []


    # ========================================================
    # EACH COLORED OBJECT
    # ========================================================

    for color, handle in object_handles.items():

        print()
        print("-" * 68)

        print(
            f"OBJECT: {color.upper()}"
        )


        # ----------------------------------------------------
        # Ground truth
        # ----------------------------------------------------

        gt = np.array(

            sim.getObjectPosition(
                handle,
                -1
            ),

            dtype=float
        )


        x_gt = gt[0]
        y_gt = gt[1]
        z_gt = gt[2]


        print(
            "Ground truth XYZ : "
            f"({x_gt:.6f}, "
            f"{y_gt:.6f}, "
            f"{z_gt:.6f}) m"
        )


        # ----------------------------------------------------
        # Vision detection
        # ----------------------------------------------------

        detection = detect_color_centroid(
            image,
            color
        )


        if detection is None:

            print(
                "Vision detection : FAILED"
            )


            results.append({

                "color":
                    color,

                "detected":
                    False,

                "pixel_u":
                    float("nan"),

                "pixel_v":
                    float("nan"),

                "contour_area_px":
                    0.0,

                "gt_x":
                    x_gt,

                "gt_y":
                    y_gt,

                "gt_z":
                    z_gt,

                "camera_depth_m":
                    float("nan"),

                "vision_x":
                    float("nan"),

                "vision_y":
                    float("nan"),

                "error_x_mm":
                    float("nan"),

                "error_y_mm":
                    float("nan"),

                "error_xy_mm":
                    float("nan"),
            })

            continue


        u = detection["u"]
        v = detection["v"]


        # ----------------------------------------------------
        # CLEAN BASELINE DEPTH
        #
        # Use the ground-truth object height only to establish
        # the baseline pixel-to-world transformation.
        #
        # Later experiments perturb depth independently.
        # ----------------------------------------------------

        depth = (
            CAMERA_Z
            -
            z_gt
        )


        x_vision, y_vision = (
            pixel_to_world_xy(
                u,
                v,
                depth
            )
        )


        # ----------------------------------------------------
        # ERROR
        # ----------------------------------------------------

        error_x = (
            x_vision
            -
            x_gt
        )


        error_y = (
            y_vision
            -
            y_gt
        )


        error_xy = math.sqrt(
            error_x ** 2
            +
            error_y ** 2
        )


        error_x_mm = (
            error_x
            *
            1000.0
        )


        error_y_mm = (
            error_y
            *
            1000.0
        )


        error_xy_mm = (
            error_xy
            *
            1000.0
        )


        print(
            f"Pixel centroid    : "
            f"({u:.3f}, {v:.3f})"
        )

        print(
            f"Depth used       : "
            f"{depth:.6f} m"
        )

        print(
            "Vision XY         : "
            f"({x_vision:.6f}, "
            f"{y_vision:.6f}) m"
        )

        print(
            f"X error           : "
            f"{error_x_mm:+.3f} mm"
        )

        print(
            f"Y error           : "
            f"{error_y_mm:+.3f} mm"
        )

        print(
            f"XY localization   : "
            f"{error_xy_mm:.3f} mm"
        )


        # ----------------------------------------------------
        # DRAW DETECTION
        # ----------------------------------------------------

        cv2.drawContours(
            display,
            [detection["contour"]],
            -1,
            (255, 255, 255),
            2
        )


        cv2.circle(
            display,
            (
                int(round(u)),
                int(round(v))
            ),
            5,
            (255, 255, 255),
            -1
        )


        cv2.putText(
            display,
            color,
            (
                int(round(u)) + 8,
                int(round(v)) - 8
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )


        results.append({

            "color":
                color,

            "detected":
                True,

            "pixel_u":
                u,

            "pixel_v":
                v,

            "contour_area_px":
                detection[
                    "area_px"
                ],

            "gt_x":
                x_gt,

            "gt_y":
                y_gt,

            "gt_z":
                z_gt,

            "camera_depth_m":
                depth,

            "vision_x":
                x_vision,

            "vision_y":
                y_vision,

            "error_x_mm":
                error_x_mm,

            "error_y_mm":
                error_y_mm,

            "error_xy_mm":
                error_xy_mm,
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

    detected_results = [

        result

        for result in results

        if result["detected"]
    ]


    print()
    print("=" * 68)
    print("                   BASELINE SUMMARY")
    print("=" * 68)


    print(
        f"Objects detected : "
        f"{len(detected_results)}/"
        f"{len(results)}"
    )


    if detected_results:

        errors = np.array(

            [
                result["error_xy_mm"]

                for result
                in detected_results
            ],

            dtype=float
        )


        rms = math.sqrt(
            np.mean(
                errors ** 2
            )
        )


        print(
            f"Mean XY error    : "
            f"{np.mean(errors):.3f} mm"
        )

        print(
            f"RMS XY error     : "
            f"{rms:.3f} mm"
        )

        print(
            f"Minimum XY error : "
            f"{np.min(errors):.3f} mm"
        )

        print(
            f"Maximum XY error : "
            f"{np.max(errors):.3f} mm"
        )


    print(
        f"\nCSV saved to:\n"
        f"{OUTPUT_FILE}"
    )


    # ========================================================
    # SAVE ANNOTATED IMAGE
    # ========================================================

    image_path = (
        Path(__file__).resolve().parent
        /
        "vision_baseline_detection.png"
    )


    cv2.imwrite(
        str(image_path),
        display
    )


    print(
        f"\nAnnotated image saved to:\n"
        f"{image_path}"
    )

    print("=" * 68)


if __name__ == "__main__":

    main()