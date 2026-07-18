"""Guidance-law and command-generation parameters."""

COMMON_KALMAN = {
    # 首帧角度直接来自视觉量测，使用较小初始方差；速度保持较大不确定性，
    # 配合连续两帧差分快速建立高速飞行初始 LOS rate。
    "angle_variance": 0.0001,
    "rate_variance": 10.0,
    # 状态为 [相对 LOS 角, 惯性 LOS 角速度]；陀螺补偿作为已知控制输入。
    # 此值是惯性 LOS 角加速度白噪声的谱密度。
    "process_accel_variance": 0.2,
    # 检测质心的像素标准差；运行时通过相机内参换算为角度量测方差。
    "measurement_noise_px": 1.0,
    # 差分角速度只用于两帧初始化，不作为后续 Kalman 伪独立量测。
    "max_initial_rate_rad_s": 10.0,
    "innovation_gate_sigma": 4.0,
}

YAW_KALMAN = {
    "angle_variance": 0.0001,
    "rate_variance": 10.0,
    "process_accel_variance": 0.2,
    "measurement_noise_px": 1.0,
    "max_initial_rate_rad_s": 10.0,
    "innovation_gate_sigma": 4.0,
}

PITCH_KALMAN = {
    "angle_variance": 0.0001,
    "rate_variance": 10.0,
    "process_accel_variance": 0.2,
    "measurement_noise_px": 1.0,
    "max_initial_rate_rad_s": 10.0,
    "innovation_gate_sigma": 4.0,
}

GUIDANCE_CONFIG = {
    "navigation_ratio": 3.0,
    "yaw_navigation_ratio": 3.0,
    "pitch_navigation_ratio": 3.0,
    "closing_velocity": 15.0,
    "yaw_closing_velocity": 14.0,
    "pitch_closing_velocity": 14.0,
    # 公共 LOS 位置反馈增益，单位为 rad/s per rad。
    "position_to_rate_gain": 0.0,
    "yaw_angle_control_gain": 0.0,
    # 两轴角度反馈均会叠加到 PN 的 LOS 角速度项。
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
    # 过载已经是镖体坐标系分量：+y 向右，+z 向下。
    # 0 表示关闭斜率限制；台架确认执行机构限速后再设为正值。
    "yaw_max_slew_g_s": 0.0,
    "pitch_max_slew_g_s": 0.0,
    # 视觉短时丢失最多预测 100 ms，超时后制导无效并输出零过载。
    "max_prediction_time_s": 0.04,
    "max_dt_sec": 0.2,
}
