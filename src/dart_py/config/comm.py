"""Communication, IMU, and GPIO parameters.

The default IMU transport is a newline-framed UART stream.  Use an
``ImuInterface`` subclass in ``comm.make_imu_interface`` only for a different
physical transport or packet protocol.
"""

COMM_CONFIG = {
    "enabled": False,
    "debug_print": False,
    "imu": {
        "enabled": True,
        "transport": "uart",
        # UART1/2/4 are available to applications on CanMV K230.  Configure
        # the selected UART's RX/TX pins through the board's IOMUX setup.
        "uart_id": "UART1",
        "baudrate": 921600,
        "bits": 8,
        "parity": "none",
        "stop": 1,
        # One newline-terminated packet at a time:
        # gx,gy,gz,ax,ay,az,gpio_bits\n
        # gyro: rad/s; accel: any consistent unit (normally m/s^2).
        "packet_format": "csv",
        "csv_fields": ["gx", "gy", "gz", "ax", "ay", "az", "gpio_bits"],
        # "arrival" is the K230 UART receive time and therefore shares the
        # image timestamp clock.  Use "packet" only for a synchronized sender.
        "timestamp_source": "arrival",
        "max_line_bytes": 128,
        "max_pending_packets": 32,
    },
    "attitude": {
        # 1 kHz is independent of the camera/inference frame rate.
        "sample_period_us": 1000,
        "max_dt_us": 5000,
        # Covers 512 ms at the 1 kHz default, including detector latency.
        "history_size": 512,
        # Average still samples after boot for gravity-based initial roll and
        # a gyro bias estimate.  Integration starts once this is complete.
        "initial_roll_samples": 32,
        "stationary_gyro_rad_s": 0.15,
        "estimate_gyro_bias": True,
        # Typical accelerometers report specific force, i.e. -gravity at rest.
        # Set to +1.0 if the IMU driver already returns a gravity vector.
        "accel_gravity_sign": -1.0,
        "roll_axis": 0,
        # state_at() chooses the sample nearest to the image timestamp.  Reject
        # it when the absolute timestamp difference exceeds this value.
        "max_match_error_us": 100000,
    },
    "gpio": {
        # Set enabled and declare pins once board wiring is known, e.g.
        # {"name": "launch_enable", "pin": 12, "pull": "down"}.
        "enabled": False,
        "pins": [],
    },
}
