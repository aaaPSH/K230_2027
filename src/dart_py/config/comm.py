"""普通 UART 和 IMU 参数。

默认 IMU 传输使用固定长度的 UART 二进制帧。
"""

COMM_CONFIG = {
    # 输出 FlightLogger 的全部字段，但频率受 diagnostics.period_ms 限制。
    "debug_print": True,
    # 仅在需要逐帧查看下发指令时启用；通常应保持关闭。
    "command_debug_print": False,
    "diagnostics": {
        # debug_print 控制是否启用；此处仅配置输出周期。
        "enabled": False,
        "period_ms": 1000,
    },
    "flight_log": {
        # 赛前可保持关闭；调参、飞行测试和赛后复盘时设为 True。
        "enabled": True,
        "directory": "/data/logs",
        # 缓冲 60 帧后写入 SD 卡，避免逐帧文件 I/O 拖慢视觉回路。
        "flush_interval_frames": 60,
        # 后台写线程轮询缓冲区的间隔；主视觉线程不执行日志文件写入。
        "writer_poll_ms": 10,
        # 程序退出时等待后台写完最后一批日志的最长时间。
        "close_timeout_ms": 5000,
        "file_prefix": "flight",
    },
    "keyframe": {
        # 保存带可视化叠加的 JPEG；文件名含帧号和图像时间戳，可对应 CSV。
        "enabled": True,
        "directory": "/data/logs/keyframes",
        # 不依赖识别状态，每 60 帧保存一张，避免逐帧写入图片。
        "interval_frames": 60,
        # 最多保留 4 个等待 JPEG 编码/写盘的图像副本，避免占用过多内存。
        "max_pending_frames": 4,
        "writer_poll_ms": 10,
        "close_timeout_ms": 5000,
        # 排错阶段输出关键帧启用、入队、保存成功及后台写入错误。
        "debug_print": False,
    },
    "imu": {
        "enabled": True,
        "transport": "uart",
        "uart_id": "UART1",
        "tx_pin": 3,
        "rx_pin": 4,
        # 仅波特率可配置，其余串口参数固定为 8N1。
        "baudrate": 115200,
        # 固定帧：36 字节，小端 IEEE-754 float32，共 9 个值。
        # 前 6 个值为 ax,ay,az,gx,gy,gz，后 3 个值暂作为保留字段。
        "accel_to_body": [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
        "gyro_to_body": [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
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
        # 10 Hz IMU 最多容忍约 1.5 个报文周期；超过后禁止非零制导输出。
        "max_source_age_us": 150000,
        # 典型加速度计输出比力，即静止时为 -重力。
        # 如果 IMU 驱动已返回重力矢量，则设为 +1.0。
        "accel_gravity_sign": -1.0,
        "roll_axis": 0,
        # state_at() 选择距图像时间戳最近的样本。
        # 当绝对时间戳差超过此值时拒绝匹配。
        "max_match_error_us": 100000,
    },
}
