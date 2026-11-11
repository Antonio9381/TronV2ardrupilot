import logging
import math
from dataclasses import dataclass

log = logging.getLogger("tron.detect")

COLOURS = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "orange": [((10, 120, 80), (22, 255, 255))],
    "yellow": [((22, 100, 100), (35, 255, 255))],
    "green": [((36, 80, 60), (85, 255, 255))],
    "blue": [((95, 120, 60), (130, 255, 255))],
    "purple": [((130, 80, 60), (165, 255, 255))],
}


@dataclass(frozen=True)
class Blob:
    nx: float
    ny: float
    area_frac: float


def pixel_to_body(blob, alt_m, hfov_deg, aspect):
    half_w = alt_m * math.tan(math.radians(hfov_deg) / 2)
    half_h = half_w / aspect
    return -blob.ny * half_h, blob.nx * half_w


class ColourDetector:
    def __init__(self, colour, min_area_frac=0.002):
        import cv2
        import numpy as np
        if colour not in COLOURS:
            raise ValueError(f"unknown colour {colour}")
        self._cv2 = cv2
        self._np = np
        self.colour = colour
        self.min_area_frac = min_area_frac
        self._ranges = [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in COLOURS[colour]]
        self._kernel = np.ones((5, 5), np.uint8)

    def find(self, frame_bgr):
        cv2 = self._cv2
        h, w = frame_bgr.shape[:2]
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame_bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = None
        for lo, hi in self._ranges:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < self.min_area_frac * w * h:
            return None
        mom = cv2.moments(best)
        if mom["m00"] == 0:
            return None
        cx, cy = mom["m10"] / mom["m00"], mom["m01"] / mom["m00"]
        return Blob(nx=(cx - w / 2) / (w / 2), ny=(cy - h / 2) / (h / 2), area_frac=area / (w * h))


class OpenCVCamera:
    def __init__(self, index=0):
        import cv2
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise RuntimeError(f"camera {index} did not open")

    def read(self):
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self):
        self._cap.release()


class PiCamera:
    def __init__(self, size=(1280, 720)):
        import cv2
        from picamera2 import Picamera2
        self._cv2 = cv2
        self._cam = Picamera2()
        self._cam.configure(self._cam.create_video_configuration(main={"size": size, "format": "RGB888"}))
        self._cam.start()

    def read(self):
        return self._cam.capture_array()

    def close(self):
        self._cam.stop()


class Locator:
    def __init__(self, camera, detector, hfov_deg):
        self.camera = camera
        self.detector = detector
        self.hfov_deg = hfov_deg

    def locate(self, alt_m):
        frame = self.camera.read()
        if frame is None:
            log.warning("camera returned no frame")
            return None
        blob = self.detector.find(frame)
        if blob is None:
            return None
        h, w = frame.shape[:2]
        return pixel_to_body(blob, max(alt_m, 0.5), self.hfov_deg, w / h)

    def close(self):
        self.camera.close()


class NullLocator:
    def locate(self, alt_m):
        return None

    def close(self):
        pass


def build_locator(camera_cfg, colour):
    if colour is None or camera_cfg.source == "none":
        return NullLocator()
    if camera_cfg.source == "opencv":
        camera = OpenCVCamera(camera_cfg.index)
    elif camera_cfg.source == "picamera2":
        camera = PiCamera()
    else:
        raise ValueError(f"unknown camera source {camera_cfg.source}")
    return Locator(camera, ColourDetector(colour, camera_cfg.min_area_frac), camera_cfg.hfov_deg)
