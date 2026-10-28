from vision import VisionSystem
from vision_detector import VisionDetector
from pixel_to_world import PixelToWorld

import math


class VisionObjectManager:

    COLORS = {
        "red",
        "blue",
        "green",
        "yellow"
    }

    # --------------------------------------------------
    # Vision safety limits
    # --------------------------------------------------

    MIN_AREA = 500

    MIN_DEPTH = 0.5
    MAX_DEPTH = 3.0

    # Conservative workspace limits for the
    # current CoppeliaSim scene.
    #
    # These are X/Y limits only.
    WORKSPACE_X = (0.70, 1.05)
    WORKSPACE_Y = (-0.60, 0.00)

    def __init__(self):

        print("\n========================================")
        print("     INITIALIZING VISION OBJECT MANAGER")
        print("========================================")

        self.vision = VisionSystem()
        self.detector = VisionDetector()
        self.localizer = PixelToWorld()

        print("Vision object manager ready")

        print("========================================")

    # --------------------------------------------------
    # Validate color
    # --------------------------------------------------

    def validate_color(self, color):

        color = color.lower().strip()

        if color not in self.COLORS:
            raise ValueError(
                f"Unsupported object color: {color}"
            )

        return color

    # --------------------------------------------------
    # Capture one complete vision frame
    # --------------------------------------------------

    def capture_frame(self):

        # RGB image
        image = self.vision.get_image()

        # Color detections
        detections = self.detector.detect_all(
            image
        )

        # Metric depth image
        depth_image = (
            self.localizer.get_depth_image()
        )

        return detections, depth_image

    # --------------------------------------------------
    # Workspace validation
    # --------------------------------------------------

    def validate_position(self, color, position):

        x = position[0]
        y = position[1]

        if not (
            self.WORKSPACE_X[0]
            <= x
            <= self.WORKSPACE_X[1]
        ):
            raise RuntimeError(
                f"{color.upper()} X position "
                f"{x:.4f} m is outside the "
                f"safe workspace."
            )

        if not (
            self.WORKSPACE_Y[0]
            <= y
            <= self.WORKSPACE_Y[1]
        ):
            raise RuntimeError(
                f"{color.upper()} Y position "
                f"{y:.4f} m is outside the "
                f"safe workspace."
            )

    # --------------------------------------------------
    # Get one object's vision position
    # --------------------------------------------------

    def get_position(self, color):

        color = self.validate_color(color)

        print(
            f"\nVISION: Searching for "
            f"{color.upper()} object..."
        )

        detections, depth_image = (
            self.capture_frame()
        )

        # ----------------------------------------------
        # Check whether color was detected
        # ----------------------------------------------

        if color not in detections:

            raise RuntimeError(
                f"VISION: {color.upper()} object "
                f"was not detected."
            )

        detection = detections[color]

        # ----------------------------------------------
        # Check detection area
        # ----------------------------------------------

        area = detection["area"]

        if area < self.MIN_AREA:

            raise RuntimeError(
                f"VISION: {color.upper()} detection "
                f"area too small: {area:.1f}"
            )

        # ----------------------------------------------
        # Pixel center
        # ----------------------------------------------

        u, v = detection["center"]

        # ----------------------------------------------
        # Check pixel
        # ----------------------------------------------

        height, width = depth_image.shape

        if not (
            0 <= u < width
            and
            0 <= v < height
        ):

            raise RuntimeError(
                f"VISION: Invalid pixel "
                f"({u}, {v})"
            )

        # ----------------------------------------------
        # Depth
        # ----------------------------------------------

        depth = float(
            depth_image[v, u]
        )

        if not math.isfinite(depth):

            raise RuntimeError(
                f"VISION: Invalid depth "
                f"for {color.upper()}"
            )

        if (
            depth < self.MIN_DEPTH
            or
            depth > self.MAX_DEPTH
        ):

            raise RuntimeError(
                f"VISION: Unsafe depth "
                f"{depth:.4f} m for "
                f"{color.upper()}"
            )

        # ----------------------------------------------
        # Pixel → Camera → World
        # ----------------------------------------------

        camera_point, world_point = (
            self.localizer.pixel_to_world(
                u,
                v,
                depth
            )
        )

        # ----------------------------------------------
        # Safety validation
        # ----------------------------------------------

        self.validate_position(
            color,
            world_point
        )

        # ----------------------------------------------
        # Print result
        # ----------------------------------------------

        print(
            f"Detected {color.upper()}"
        )

        print(
            f"Pixel       : ({u}, {v})"
        )

        print(
            f"Area        : {area:.1f}"
        )

        print(
            f"Depth       : {depth:.4f} m"
        )

        print(
            f"Camera XYZ  : "
            f"({camera_point[0]:.4f}, "
            f"{camera_point[1]:.4f}, "
            f"{camera_point[2]:.4f})"
        )

        print(
            f"World XYZ   : "
            f"({world_point[0]:.4f}, "
            f"{world_point[1]:.4f}, "
            f"{world_point[2]:.4f})"
        )

        print(
            "VISION: Position validated"
        )

        return world_point

    # --------------------------------------------------
    # Get X/Y only
    # --------------------------------------------------

    def get_xy(self, color):

        position = self.get_position(
            color
        )

        return position[0], position[1]

    # --------------------------------------------------
    # Check whether object exists visually
    # --------------------------------------------------

    def exists(self, color):

        color = self.validate_color(color)

        detections, _ = (
            self.capture_frame()
        )

        return color in detections

    # --------------------------------------------------
    # Print all visible objects
    # --------------------------------------------------

    def print_objects(self):

        print("\n========================================")
        print("       VISION OBJECT POSITIONS")
        print("========================================")

        detections, depth_image = (
            self.capture_frame()
        )

        for color in self.COLORS:

            if color not in detections:

                print(
                    f"{color.upper():<8} "
                    f"→ NOT DETECTED"
                )

                continue

            detection = detections[color]

            u, v = detection["center"]

            depth = float(
                depth_image[v, u]
            )

            _, world = (
                self.localizer.pixel_to_world(
                    u,
                    v,
                    depth
                )
            )

            print(
                f"{color.upper():<8} → "
                f"X={world[0]:.4f}, "
                f"Y={world[1]:.4f}, "
                f"Z={world[2]:.4f}"
            )

        print("========================================")

    # --------------------------------------------------
    # Close vision systems
    # --------------------------------------------------

    def close(self):

        try:
            self.vision.close()
        except Exception:
            pass

        try:
            self.localizer.close()
        except Exception:
            pass