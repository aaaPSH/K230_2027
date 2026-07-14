"""OpenCV detector parameters."""

DETECTOR_CONFIG = {
    "hsv_low": (45, 80, 120),
    "hsv_high": (85, 255, 255),
    "min_area": 3.0,
    "max_area": 5000.0,
    "min_circularity": 0.2,
    "max_aspect_ratio_diff": 0.5,
}
