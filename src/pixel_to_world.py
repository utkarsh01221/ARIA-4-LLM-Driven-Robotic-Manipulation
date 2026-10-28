from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math


class PixelToWorld:

    def __init__(self):

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.sensor = self.sim.getObject("/visionSensor")

        resolution = self.sim.getVisionSensorResolution(
            self.sensor
        )

        self.width = resolution[0]
        self.height = resolution[1]

        # Read actual camera FOV
        self.fov = self.sim.getObjectFloatParam(
            self.sensor,
            self.sim.visionfloatparam_perspective_angle
        )

        # Camera intrinsics
        self.fx = (
            self.width / 2.0
        ) / math.tan(self.fov / 2.0)

        self.fy = (
            self.height / 2.0
        ) / math.tan(self.fov / 2.0)

        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        print("========================================")
        print("        PIXEL → WORLD LOCALIZATION")
        print("========================================")

        print(
            f"Resolution : "
            f"{self.width} x {self.height}"
        )

        print(
            f"FOV        : "
            f"{math.degrees(self.fov):.3f} deg"
        )

        print(
            f"fx         : "
            f"{self.fx:.3f}"
        )

        print(
            f"fy         : "
            f"{self.fy:.3f}"
        )

        print(
            f"cx         : "
            f"{self.cx:.3f}"
        )

        print(
            f"cy         : "
            f"{self.cy:.3f}"
        )

    def get_depth_image(self):

        # IMPORTANT:
        # options=1 tells CoppeliaSim to return
        # depth directly in metres.
        depth, resolution = (
            self.sim.getVisionSensorDepth(
                self.sensor,
                1
            )
        )

        width = resolution[0]
        height = resolution[1]

        depth = np.frombuffer(
            depth,
            dtype=np.float32
        )

        depth = depth.reshape(
            height,
            width
        )

        # Match RGB image orientation
        depth = np.flipud(depth)

        return depth

    def pixel_to_camera(
        self,
        u,
        v,
        depth
    ):

        # Depth is already in metres.
        z = float(depth)

        x = (
            -(u - self.cx)
            * z
            / self.fx
        )

        y = (
            -(v - self.cy)
            * z
            / self.fy
        )

        return [
            x,
            y,
            z
        ]

    def camera_to_world(
        self,
        camera_point
    ):

        x = camera_point[0]
        y = camera_point[1]
        z = camera_point[2]

        # Actual camera transformation obtained
        # from your CoppeliaSim scene:
        #
        # [ 1   0   0   1.2 ]
        # [ 0  -1   0  -0.2 ]
        # [ 0   0  -1   1.5 ]

        world_x = 1.2 + x
        world_y = -0.2 - y
        world_z = 1.5 - z

        return [
            world_x,
            world_y,
            world_z
        ]

    def pixel_to_world(
        self,
        u,
        v,
        depth
    ):

        camera_point = self.pixel_to_camera(
            u,
            v,
            depth
        )

        world_point = self.camera_to_world(
            camera_point
        )

        return camera_point, world_point

    def close(self):
        pass