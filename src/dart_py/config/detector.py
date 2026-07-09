"""OpenCV detector parameters."""

DETECTOR_CONFIG = {
    "hsv_low": (45, 80, 120),
    "hsv_high": (85, 255, 255),
    "min_area": 3.0,
    "max_area": 5000.0,
    "min_circularity": 0.2,
    "max_aspect_ratio_diff": 0.5,
    "median_kernel": 3,
    "green_min": 120,
    "min_gr_ratio": 2.0,
    "min_gb_ratio": 2.0,
    "min_ratio_pass_ratio": 0.45,
    "min_ratio_pixels": 3,
    "max_sample_pixels": 128,
}
