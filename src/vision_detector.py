import cv2
import numpy as np


class VisionDetector:

    COLORS = {
        "red": [
            ((0, 120, 70), (10, 255, 255)),
            ((170, 120, 70), (180, 255, 255))
        ],

        "blue": [
            ((90, 100, 70), (130, 255, 255))
        ],

        "green": [
            ((35, 80, 70), (85, 255, 255))
        ],

        "yellow": [
            ((20, 100, 70), (35, 255, 255))
        ]
    }

    def __init__(self):
        print("========================================")
        print("       VISION DETECTOR INITIALIZED")
        print("========================================")

    def detect_color(self, image, color):

        color = color.lower()

        if color not in self.COLORS:
            raise ValueError(f"Unsupported color: {color}")

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = None

        for lower, upper in self.COLORS[color]:

            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)

            current_mask = cv2.inRange(
                hsv,
                lower,
                upper
            )

            if mask is None:
                mask = current_mask
            else:
                mask = cv2.bitwise_or(mask, current_mask)

        # Remove small noise
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, mask

        # Select largest contour
        contour = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(contour)

        # Ignore tiny detections
        if area < 100:
            return None, mask

        # Bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)

        M = cv2.moments(contour)

        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
        else:
            center_x = x + w // 2
            center_y = y + h // 2

        detection = {
            "color": color,
            "center": (center_x, center_y),
            "area": area,
            "bbox": (x, y, w, h)
        }

        return detection, mask

    def detect_all(self, image):

        detections = {}

        for color in self.COLORS:

            detection, _ = self.detect_color(
                image,
                color
            )

            if detection is not None:
                detections[color] = detection

        return detections

    def draw_detection(self, image, detection):

        if detection is None:
            return image

        x, y, w, h = detection["bbox"]
        cx, cy = detection["center"]
        color = detection["color"]

        cv2.rectangle(
            image,
            (x, y),
            (x + w, y + h),
            (255, 255, 255),
            2
        )

        cv2.circle(
            image,
            (cx, cy),
            5,
            (255, 255, 255),
            -1
        )

        text = f"{color} ({cx},{cy})"

        cv2.putText(
            image,
            text,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        return image