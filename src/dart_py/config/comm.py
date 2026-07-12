"""通信、IMU 和 GPIO 参数。

默认 IMU 传输使用换行符分帧的 UART 流。
仅在需要不同的物理传输或报文协议时，在 ``comm.make_imu_interface``
中使用 ``ImuInterface`` 的子类。
"""

COMM_CONFIG = {
    "enabled": False,
    "debug_print": False,
    "imu": {
        "enabled": True,
        "transport": "uart",
        # CanMV K230 上可供应用使用的 UART 为 UART1/2/4。
        # 下面为 UART1 常用的 TX=9、RX=10；请按实际开发板接线修改。
        "uart_id": "UART1",
        "tx_pin": 9,
        "rx_pin": 10,
        "baudrate": 115200,
        "bits": 8,
        "parity": "none",
        "stop": 1,
        "timeout": 0,
        # 每次发送一个以换行符结尾的报文：
        # ay,ax,az,pitch,roll,yaw
        # 加速度按协议 ay,ax,az 发送，映射后为机体系 [ax,ay,az]。
        # 后三项按 pitch,roll,yaw 发送，映射后为 [roll,pitch,yaw]。
        "packet_format": "csv",
        "csv_fields": ["ay", "ax", "az", "pitch", "roll", "yaw"],
        "accel_fields": ["ax", "ay", "az"],
        "gyro_fields": ["roll", "pitch", "yaw"],
        # 若发送端已经输出 rad/s，则改为 "rad_s"。
        "gyro_unit": "rad_s",
        # "arrival" 使用 K230 UART 接收时间，因此与图像时间戳共享
        # 同一时钟。仅当发送方已同步时使用 "packet"。
        "timestamp_source": "arrival",
        "max_line_bytes": 128,
        "max_pending_packets": 32,
    },
    "attitude": {
        # 1 kHz 采样率，与相机/推理帧率无关。
        "sample_period_us": 1000,
        "max_dt_us": 5000,
        # 在 1 kHz 默认值下覆盖 512 ms，包含检测器延迟。
        "history_size": 512,
        # 启动后采集静止样本，用于基于重力的初始滚转角和陀螺仪零偏估计。
        # 采集完成后开始积分。
        "initial_roll_samples": 32,
        "stationary_gyro_rad_s": 0.15,
        "estimate_gyro_bias": True,
        # 典型加速度计输出比力，即静止时为 -重力。
        # 如果 IMU 驱动已返回重力矢量，则设为 +1.0。
        "accel_gravity_sign": -1.0,
        "roll_axis": 0,
        # state_at() 选择距图像时间戳最近的样本。
        # 当绝对时间戳差超过此值时拒绝匹配。
        "max_match_error_us": 100000,
    },
    "gpio": {
        # 确定开发板接线后，启用并声明引脚，例如：
        # {"name": "launch_enable", "pin": 12, "pull": "down"}。
        "enabled": False,
        "pins": [],
    },
}
