"""通信、IMU 和 GPIO 参数。

默认 IMU 传输使用固定长度的 UART 二进制帧。
"""

COMM_CONFIG = {
    "enabled": False,
    "debug_print": False,
    # 通过 UART4 输出关键制导调试数据，不使用控制台打印。
    "debug_uart": {
        "enabled": True,
        "uart_id": "UART4",
        "tx_pin": 29,
        "rx_pin": 31,
        # RS-485 收发控制脚：DE=26，/RE=32。
        "de_pin": 26,
        "re_pin": 32,
        # 常见 RS-485 芯片的 /RE 为低电平有效。
        "re_active_low": True,
        "baudrate": 115200,
        "bits": 8,
        "parity": "none",
        "stop": 1,
        "timeout": 0,
        # 50 ms 约为 20 Hz，适合 115200 波特率。
        "debug_period_ms": 50,
    },
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
        # 固定帧：36 字节，小端 IEEE-754 float32，共 9 个值。
        # 流中没有帧头/校验时，发送端必须从帧边界开始发送；UART 丢字节后
        # 无法自动恢复对齐。
        "packet_format": "binary_float32_le",
        "frame_bytes": 36,
        # 发送顺序：镖体系 ay,ax,az,gy,gx,gz；后三项含义尚未提供，先保留。
        "binary_fields": [
            "ay", "ax", "az", "gy", "gx", "gz",
            "reserved_0", "reserved_1", "reserved_2",
        ],
        "accel_fields": ["ax", "ay", "az"],
        "gyro_fields": ["gx", "gy", "gz"],
        # 从 IMU 装配坐标转换到镖体系。符号/轴向未确认前使用单位阵；
        # 确认后只修改矩阵，例如某轴反向可将对应对角元素设为 -1.0。
        "accel_to_body": [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        "gyro_to_body": [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        # 当前按 rad/s 解释；若发送端输出度/秒，改为 "deg_s"。
        "gyro_unit": "rad_s",
        # "arrival" 使用 K230 UART 接收时间，因此与图像时间戳共享
        # 同一时钟。仅当发送方已同步时使用 "packet"。
        "timestamp_source": "arrival",
        "max_line_bytes": 128,
        "max_pending_packets": 32,
    },
    "attitude": {
        # UART 约 10 Hz，但使用最近一次角速度在 1 kHz 线程中持续积分。
        "sample_period_us": 1000,
        "max_dt_us": 5000,
        # 在 1 kHz 默认值下覆盖 512 ms，包含检测器延迟。
        "history_size": 512,
        # 启动后采集静止样本，用于基于重力的初始滚转角和陀螺仪零偏估计。
        # 采集完成后开始积分。
        "initial_roll_samples": 10,
        "stationary_gyro_rad_s": 0.15,
        "estimate_gyro_bias": True,
        # 仅在最近一帧 UART 数据不超过 250 ms 时外推，避免断流无限积分。
        "hold_last_gyro": True,
        "max_hold_gyro_us": 250000,
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
