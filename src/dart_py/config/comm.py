"""普通 UART 和 IMU 参数。

默认 IMU 传输使用固定长度的 UART 二进制帧。
"""

COMM_CONFIG = {
    # 控制台调试按功能独立开启，默认全部关闭。
    "console": {
        # 低频输出 FPS、各处理阶段耗时和 IMU 队列状态。
        "runtime": False,
        # 低频输出 FlightLogger 的全部遥测字段。
        "telemetry": False,
        # 低频单独输出 IMU 姿态、角速度、时效和故障状态。
        "imu": False,
        # 逐帧输出最终下发指令，频率高，仅在台架排错时开启。
        "command": False,
        # 输出飞行日志启动、停用和后台写入错误。
        "flight_log": False,
        # 输出关键帧入队、丢弃、保存和后台写入错误。
        "keyframe": False,
        # 输出主循环异常；开启后未处理异常还会显示 traceback。
        "errors": False,
    },
    "command": {
        "enabled": True,
        "uart_id": "UART1",
        "tx_pin": 3,
        "rx_pin": 4,
        "baudrate": 115200,
        # 下发帧：5A A5 + IMU 横向轴(float32) + IMU 法向轴(float32) + 累加校验和。
        # 当前 imu_to_body 为 body=[imu_y, imu_x, -imu_z]，因此使用 IMU x/z。
        "lateral_imu_axis": 0,
        "normal_imu_axis": 2,
    },
    "diagnostics": {
        # runtime、telemetry 和 imu 共用此输出周期。
        "period_ms": 1000,
    },
    "flight_log": {
        # 赛前可保持关闭；调参、飞行测试和赛后复盘时设为 True。
        "enabled": True,
        "directory": "/data/logs",
        # 每帧记录 54 个核心及 Kalman 调参字段，缓冲 60 帧后交给后台写入。
        "flush_interval_frames": 60,
        # 后台写线程轮询缓冲区的间隔；主视觉线程不执行日志文件写入。
        "writer_poll_ms": 10,
        # 后台最多积压 8 个批次；写盘跟不上时丢弃日志，不阻塞制导。
        "max_pending_buffers": 8,
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
        # 前 6 个值为 ax,ay,az,gx,gy,gz，第 7、8 个值保留，第 9 个值为
        # 小端 float32 帧尾字节 00 00 80 7F。
        # 连续达到该数量的坏帧才标记 IMU 失效，单帧错误不会停止姿态线程。
        "max_consecutive_invalid_frames": 20,
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
