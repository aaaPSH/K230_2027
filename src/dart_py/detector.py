"""OpenCV green-light detector for CanMV K230."""
import cv2
from ulab import numpy as np


class Detector:
    def __init__(
        self,
        hsv_low=(45, 80, 120),
        hsv_high=(85, 255, 255),
        min_area=3.0,
        max_area=5000.0,
        min_circularity=0.2,
        max_aspect_ratio_diff=0.5,
    ):
        self.hsv_low = _np_u8(hsv_low)
        self.hsv_high = _np_u8(hsv_high)
        self.min_area = float(min_area)
        self.max_area = float(max_area)
        self.min_circularity = float(min_circularity)
        self.max_aspect_ratio_diff = float(max_aspect_ratio_diff)

    def detect(self, image):
        """Detect the target in a CanMV image.Image or RGB888 ndarray."""
        image_np = _as_ndarray(image)
        if image_np is None:
            return _lost_result()

        hsv_image = cv2.cvtColor(image_np, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_image, self.hsv_low, self.hsv_high)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        best_contour = None
        best_stats = None
        best_score = 0.0
        for contour in contours:
            stats = self._candidate_stats(contour)
            if stats is None:
                continue
            score = stats["area"]
            if score > best_score:
                best_score = score
                best_contour = contour
                best_stats = stats

        if best_contour is None:
            return _lost_result()
        return _result_from_contour(best_contour, best_stats)

    def _candidate_stats(self, contour):
        area = cv2.contourArea(contour)
        if area < self.min_area:
            return None
        if self.max_area > 0 and area > self.max_area:
            return None

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return None

        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < self.min_circularity:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return None

        aspect_ratio_diff = abs(w - h) / max(w, h)
        if aspect_ratio_diff > self.max_aspect_ratio_diff:
            return None

        return {
            "area": float(area),
            "bbox": (x, y, w, h),
            "circularity": float(circularity),
        }


def _as_ndarray(image):
    if hasattr(image, "to_numpy_ref"):
        return image.to_numpy_ref()
    return image


def _np_u8(values):
    return np.array(values, dtype=np.uint8)


def _result_from_contour(contour, stats):
    x, y, w, h = stats["bbox"]
    moments = cv2.moments(contour)
    if moments["m00"] != 0:
        cx = float(moments["m10"] / moments["m00"])
        cy = float(moments["m01"] / moments["m00"])
    else:
        cx = float(x + 0.5 * w)
        cy = float(y + 0.5 * h)

    return {
        "detected": True,
        "x": cx,
        "y": cy,
        "bbox": (x, y, w, h),
        "area": float(stats["area"]),
        "circularity": float(stats["circularity"]),
    }


def _lost_result():
    return {
        "detected": False,
        "x": -1.0,
        "y": -1.0,
        "bbox": None,
        "area": 0.0,
        "circularity": 0.0,
    }
