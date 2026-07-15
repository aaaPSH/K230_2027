"""Camera parameters for CanMV K230."""

CAMERA_CONFIG = {
    "sensor_id": 2,
    "width": 320,
    "height": 240,
    "fps": 90,
    "channel_id": 0,
    "hmirror": False,
    "vflip": False,
    "auto_exposure": False,
    "exposure_us": 100,
    "analog_gain": 5.0,
    # 相机内参矩阵 K。当前数值与原 65°×40° 视场角配置等价，保证迁移前后
    # 制导增益不突变；完成实机标定后应直接替换为标定结果。
    "camera_matrix": [
        [251.149692, 0.0, 160.0],
        [0.0, 329.697290, 120.0],
        [0.0, 0.0, 1.0],
    ],
    # 仅作为未提供内参矩阵时的兼容回退。
    "fov_x_deg": 65.0,
    "fov_y_deg": 40.0,
    "fx": None,
    "fy": None,
    "cx": None,
    "cy": None,
}
