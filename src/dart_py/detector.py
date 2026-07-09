"""OpenCV green-light detector for CanMV K230."""
import cv2
from ulab import numpy as np


class Detector:
    def __init__(
        self,
        hsv_low=(45, 80, 120),
        hsv_high=(85, 255, 255),
        min_area=10.0,
        max_area=5000.0,
        min_circularity=0.2,
        max_aspect_ratio_diff=0.5,
        median_kernel=3,
        green_min=120,
        min_gr_ratio=1.35,
        min_gb_ratio=1.35,
        min_ratio_pass_ratio=0.45,
        min_ratio_pixels=3,
        max_sample_pixels=128,
        ratio_epsilon=1.0,
        thresholds_low=None,
        thresholds_high=None,
        rgb_threshold=None,
        fallback_min_area=None,
        kernel_size=None,
        min_green_delta=None,
        min_green_ratio=None,
    ):
        if thresholds_low is not None:
            hsv_low = thresholds_low
        if thresholds_high is not None:
            hsv_high = thresholds_high
        if kernel_size is not None:
            median_kernel = kernel_size
        if min_green_ratio is not None:
            min_ratio_pass_ratio = min_green_ratio

        self.hsv_low = _np_u8(hsv_low)
        self.hsv_high = _np_u8(hsv_high)
        self.min_area = float(min_area)
        self.max_area = float(max_area)
        self.min_circularity = float(min_circularity)
        self.max_aspect_ratio_diff = float(max_aspect_ratio_diff)
        self.green_min = int(green_min)
        self.min_gr_ratio = float(min_gr_ratio)
        self.min_gb_ratio = float(min_gb_ratio)
        self.min_ratio_pass_ratio = float(min_ratio_pass_ratio)
        self.min_ratio_pixels = int(min_ratio_pixels)
        self.max_sample_pixels = int(max_sample_pixels)
        self.ratio_epsilon = float(ratio_epsilon)
        if self.min_ratio_pixels < 1:
            self.min_ratio_pixels = 1
        if self.max_sample_pixels < 1:
            self.max_sample_pixels = 1
        if self.ratio_epsilon < 1.0:
            self.ratio_epsilon = 1.0

        self.median_kernel = int(median_kernel)
        if self.median_kernel < 1:
            self.median_kernel = 1
        if self.median_kernel % 2 == 0:
            self.median_kernel += 1

    def detect(self, image):
        """Detect the target in a CanMV image.Image or RGB888 ndarray."""
        image_np = _as_ndarray(image)
        if image_np is None:
            return _lost_result()

        hsv_image = cv2.cvtColor(image_np, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_image, self.hsv_low, self.hsv_high)
        if self.median_kernel > 1:
            mask = cv2.medianBlur(mask, self.median_kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        best_contour = None
        best_stats = None
        best_score = 0.0
        for contour in contours:
            stats = self._candidate_stats(image_np, mask, contour)
            if stats is None:
                continue
            score = stats["area"] * stats["ratio_pass"]
            if score > best_score:
                best_score = score
                best_contour = contour
                best_stats = stats

        if best_contour is None:
            return _lost_result()
        return _result_from_contour(best_contour, best_stats)

    def _candidate_stats(self, image_np, mask, contour):
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

        ratio_stats = self._green_ratio_stats(image_np, mask, (x, y, w, h))
        if ratio_stats is None:
            return None

        return {
            "area": float(area),
            "bbox": (x, y, w, h),
            "circularity": float(circularity),
            "ratio_pass": ratio_stats["ratio_pass"],
            "gr_ratio": ratio_stats["gr_ratio"],
            "gb_ratio": ratio_stats["gb_ratio"],
            "ratio_pixels": ratio_stats["ratio_pixels"],
        }

    def _green_ratio_stats(self, image_np, mask, bbox):
        x, y, w, h = bbox
        shape = image_np.shape
        height = int(shape[0])
        width = int(shape[1])
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(width, x + w)
        y1 = min(height, y + h)
        if x0 >= x1 or y0 >= y1:
            return None

        sample_w = x1 - x0
        sample_h = y1 - y0
        step = 1
        while (
            ((sample_w + step - 1) // step)
            * ((sample_h + step - 1) // step)
            > self.max_sample_pixels
        ):
            step += 1

        ratio_pixels = 0
        pass_pixels = 0
        gr_sum = 0.0
        gb_sum = 0.0
        for py in range(y0, y1, step):
            image_row = image_np[py]
            mask_row = mask[py]
            for px in range(x0, x1, step):
                if int(mask_row[px]) == 0:
                    continue
                pixel = image_row[px]
                r = int(pixel[0])
                g = int(pixel[1])
                b = int(pixel[2])
                if g < self.green_min:
                    continue

                gr_ratio = g / max(r, self.ratio_epsilon)
                gb_ratio = g / max(b, self.ratio_epsilon)
                ratio_pixels += 1
                if gr_ratio >= self.min_gr_ratio and gb_ratio >= self.min_gb_ratio:
                    pass_pixels += 1
                    gr_sum += gr_ratio
                    gb_sum += gb_ratio

        if ratio_pixels < self.min_ratio_pixels or pass_pixels <= 0:
            return None

        ratio_pass = pass_pixels / ratio_pixels
        if ratio_pass < self.min_ratio_pass_ratio:
            return None

        return {
            "ratio_pass": float(ratio_pass),
            "gr_ratio": float(gr_sum / pass_pixels),
            "gb_ratio": float(gb_sum / pass_pixels),
            "ratio_pixels": int(ratio_pixels),
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
        "green_ratio": float(stats["ratio_pass"]),
        "gr_ratio": float(stats["gr_ratio"]),
        "gb_ratio": float(stats["gb_ratio"]),
        "ratio_pixels": int(stats["ratio_pixels"]),
        "circularity": float(stats["circularity"]),
    }


def _lost_result():
    return {
        "detected": False,
        "x": -1.0,
        "y": -1.0,
        "bbox": None,
        "area": 0.0,
        "green_ratio": 0.0,
        "gr_ratio": 0.0,
        "gb_ratio": 0.0,
        "ratio_pixels": 0,
        "circularity": 0.0,
    }
