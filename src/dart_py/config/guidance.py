"""Guidance-law and command-generation parameters."""

COMMON_KALMAN = {
    "angle_variance": 0.05,
    "rate_variance": 1.0,
    "process_angle_variance": 0.0001,
    "process_rate_variance": 0.02,
    "measurement_angle_variance": 0.0025,
    "measurement_rate_variance": 0.1,
}

YAW_KALMAN = {
    "angle_variance": 0.05,
    "rate_variance": 1.0,
    "process_angle_variance": 0.0001,
    "process_rate_variance": 0.02,
    "measurement_angle_variance": 0.0025,
    "measurement_rate_variance": 10.0,
}

PITCH_KALMAN = {
    "angle_variance": 0.05,
    "rate_variance": 1.0,
    "process_angle_variance": 0.0001,
    "process_rate_variance": 0.02,
    "measurement_angle_variance": 0.0025,
    "measurement_rate_variance": 0.1,
}

GUIDANCE_CONFIG = {
    "navigation_ratio": 3.0,
    "yaw_navigation_ratio": 3.0,
    "pitch_navigation_ratio": 3.0,
    "closing_velocity": 15.0,
    "yaw_closing_velocity": 14.0,
    "pitch_closing_velocity": 14.0,
    "yaw_angle_control_gain": 0.0,
    "pitch_angle_control_gain": 0.0,
    "rate_filter_alpha": -1.0,
    "use_kalman_filter": True,
    "kalman": COMMON_KALMAN,
    "yaw_kalman": YAW_KALMAN,
    "pitch_kalman": PITCH_KALMAN,
    "max_overload_g": 0.5,
    "yaw_max_overload_g": 0.5,
    "pitch_max_overload_g": 0.5,
    "roll_compensation": True,
    "roll_sign": -1.0,
    "yaw_output_sign": -1.0,
    "pitch_output_sign": 1.0,
    "max_dt_sec": 0.2,
    "publish_zero_on_lost": True,
}
