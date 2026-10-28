from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2


class VisionSystem:

    def __init__(self):

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.sensor = self.sim.getObject(
            "/visionSensor"
        )

        print("Vision sensor connected")


    # ========================================================
    # GET RGB IMAGE
    # ========================================================

    def get_image(self):

        image, resolution = self.sim.getVisionSensorImg(
            self.sensor
        )

        width = resolution[0]
        height = resolution[1]

        img = np.frombuffer(
            image,
            dtype=np.uint8
        )

        img = img.reshape(
            height,
            width,
            3
        )

        # CoppeliaSim image is vertically flipped
        img = cv2.flip(
            img,
            0
        )

        # RGB → BGR for OpenCV
        img = cv2.cvtColor(
            img,
            cv2.COLOR_RGB2BGR
        )

        return img


    # ========================================================
    # DISPLAY IMAGE
    # ========================================================

    def show_image(self):

        image = self.get_image()

        cv2.imshow(
            "CoppeliaSim Vision",
            image
        )

        cv2.waitKey(1)


    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        cv2.destroyAllWindows()