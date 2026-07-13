"""普通 UART IMU 通信接口。"""

IMU_FRAME_BYTES = 36
IMU_FLOAT_COUNT = 9


class ImuInterface:
    """单帧原始 IMU 样本的输入接口。

    子类应返回 ``None`` 或包含 ``gyro_b`` 和 ``accel_b`` 的字典。
    两者均为机体坐标系向量，顺序为 ``[x 前, y 右, z 下]``。
    陀螺仪单位必须为 rad/s。可选的 ``timestamp_us`` 必须使用
    ``time.ticks_us()`` 的时间基准。
    """

    def __init__(self, config=None):
        self.config = config or {}

    def read(self):
        """返回 None 或 {'gyro_b': [p, q, r], 'accel_b': [ax, ay, az]}。"""
        return None

    def deinit(self):
        """释放输入设备资源。"""
        return None


class SerialImuReader(ImuInterface):
    """从 UART 流中读取并转换 IMU 报文。

    协议固定为 36 字节的小端 IEEE-754 ``float32 * 9``。发送顺序为::

        ax,ay,az,gx,gy,gz,reserved_0,reserved_1,reserved_2

    接收数据先按 ``accel=[ax,ay,az]``、``gyro=[gx,gy,gz]`` 组织，再通过
    可配置装配矩阵转换为制导使用的镖体系 ``accel_b``、``gyro_b``。

    UART 波特率从配置读取，其余参数固定为 8N1；报文时间戳使用 K230
    收到完整帧的时间。
    """

    def __init__(self, config=None):
        ImuInterface.__init__(self, config)
        self.uart = initialize_imu_uart(self.config)
        self.accel_to_body = _axis_transform(
            self.config.get("accel_to_body", _identity_matrix()),
            "accel_to_body",
        )
        self.gyro_to_body = _axis_transform(
            self.config.get("gyro_to_body", _identity_matrix()),
            "gyro_to_body",
        )
        self.max_pending_packets = max(
            1,
            int(self.config.get("max_pending_packets", 32)),
        )
        self._rx_buffer = bytearray()
        self._pending_packets = []
        self.invalid_packet_count = 0

    @property
    def pending_packet_count(self):
        return len(self._pending_packets)

    def read(self):
        self._read_available()
        if not self._pending_packets:
            return None
        return self._pending_packets.pop(0)

    def deinit(self):
        if self.uart is not None and hasattr(self.uart, "deinit"):
            self.uart.deinit()
        self.uart = None

    def _read_available(self):
        if self.uart is None:
            return
        data = self.uart.read()
        if not data:
            return
        if isinstance(data, str):
            data = data.encode()
        self._rx_buffer.extend(data)

        while len(self._rx_buffer) >= IMU_FRAME_BYTES:
            frame = bytes(self._rx_buffer[:IMU_FRAME_BYTES])
            del self._rx_buffer[:IMU_FRAME_BYTES]
            self._append_packet(frame)

    def _append_packet(self, frame):
        try:
            packet = self._parse_packet(frame)
        except (TypeError, ValueError, IndexError):
            self.invalid_packet_count += 1
            return
        if packet is None:
            self.invalid_packet_count += 1
            return
        self._pending_packets.append(packet)
        if len(self._pending_packets) > self.max_pending_packets:
            # 保留最新样本，防止图像匹配滞后。
            del self._pending_packets[0]

    def _parse_packet(self, frame):
        record = self._parse_binary_frame(frame)
        accel_b = [record["ax"], record["ay"], record["az"]]
        gyro_b = [record["gx"], record["gy"], record["gz"]]
        accel_b = _mat_vec_mul(self.accel_to_body, accel_b)
        gyro_b = _mat_vec_mul(self.gyro_to_body, gyro_b)

        sample = {
            "timestamp_us": _ticks_us(),
            "gyro_b": gyro_b,
            "accel_b": accel_b,
            "uart_fields": record,
        }
        return sample

    def _parse_binary_frame(self, frame):
        if len(frame) != IMU_FRAME_BYTES:
            raise ValueError("UART binary frame length is invalid")
        values = _unpack_float32_le(frame, IMU_FLOAT_COUNT)
        return {
            "ax": values[0],
            "ay": values[1],
            "az": values[2],
            "gx": values[3],
            "gy": values[4],
            "gz": values[5],
            "reserved_0": values[6],
            "reserved_1": values[7],
            "reserved_2": values[8],
        }


def make_default_imu_interface(config):
    return ImuInterface(config)


def make_imu_interface(config):
    """创建配置指定的 IMU 传输实例。"""
    imu_config = config.get("imu", {})
    if imu_config.get("enabled", False):
        if imu_config.get("transport", "uart") == "uart":
            return SerialImuReader(imu_config)
        raise ValueError("unsupported IMU transport")
    return make_default_imu_interface(config)


def initialize_imu_uart(config):
    """创建波特率可配置、其余参数固定为 8N1 的 IMU UART。"""
    return _initialize_uart(
        config,
        baudrate=int(config.get("baudrate", 115200)),
        bits=8,
        parity="none",
        stop=1,
        timeout=0,
    )


def _initialize_uart(config, baudrate, bits, parity, stop, timeout=None):
    """配置引脚并创建 UART。"""
    try:
        from machine import UART
    except ImportError:
        raise RuntimeError("machine.UART is required for the UART link")

    uart_id, uart_name = _resolve_uart_id(UART, config.get("uart_id", "UART1"))
    _configure_uart_pins(config, uart_name)

    if bits == 8:
        bits = getattr(UART, "EIGHTBITS", bits)
    if parity == "none":
        parity = getattr(UART, "PARITY_NONE", parity)
    if stop == 1:
        stop = getattr(UART, "STOPBITS_ONE", stop)

    uart_kwargs = {
        "baudrate": baudrate,
        "bits": bits,
        "parity": parity,
        "stop": stop,
    }
    if timeout is not None:
        uart_kwargs["timeout"] = timeout
    try:
        return UART(uart_id, **uart_kwargs)
    except TypeError:
        # Older CanMV firmware does not expose the optional timeout argument.
        uart_kwargs.pop("timeout", None)
        return UART(uart_id, **uart_kwargs)


def _resolve_uart_id(UART, uart_id):
    if isinstance(uart_id, str):
        resolved = getattr(UART, uart_id, None)
        if resolved is None:
            raise ValueError("invalid UART id: {}".format(uart_id))
        return resolved, uart_id

    for name in ("UART1", "UART2", "UART3", "UART4"):
        if getattr(UART, name, None) == uart_id:
            return uart_id, name
    raise ValueError("invalid UART id")


def _configure_uart_pins(config, uart_name):
    tx_pin = config.get("tx_pin")
    rx_pin = config.get("rx_pin")
    if tx_pin is None and rx_pin is None:
        return
    try:
        from machine import FPIOA
    except ImportError:
        raise RuntimeError("machine.FPIOA is required to configure UART pins")

    fpioa = FPIOA()
    if tx_pin is not None:
        tx_function = config.get("tx_function", "{}_TXD".format(uart_name))
        fpioa.set_function(
            int(tx_pin),
            _fpioa_function(fpioa, FPIOA, tx_function),
            ie=0,
            oe=1,
        )
    if rx_pin is not None:
        rx_function = config.get("rx_function", "{}_RXD".format(uart_name))
        fpioa.set_function(
            int(rx_pin),
            _fpioa_function(fpioa, FPIOA, rx_function),
            ie=1,
            oe=0,
        )


def _fpioa_function(fpioa, FPIOA, name):
    function = getattr(FPIOA, name, None)
    if function is None:
        function = getattr(fpioa, name, None)
    if function is None:
        raise ValueError("unsupported FPIOA function: {}".format(name))
    return function


def _ticks_us():
    try:
        import time
        if hasattr(time, "ticks_us"):
            return time.ticks_us()
        return int(time.time() * 1000000)
    except Exception:
        return 0


def _identity_matrix():
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _axis_transform(matrix, name):
    if matrix is None or len(matrix) != 3:
        raise ValueError("{} must be a 3x3 matrix".format(name))
    result = []
    for row in matrix:
        if len(row) != 3:
            raise ValueError("{} must be a 3x3 matrix".format(name))
        try:
            result.append([float(row[0]), float(row[1]), float(row[2])])
        except (TypeError, ValueError, IndexError):
            raise ValueError("{} contains a non-numeric value".format(name))
    return result


def _mat_vec_mul(matrix, vector):
    return [
        matrix[row][0] * vector[0]
        + matrix[row][1] * vector[1]
        + matrix[row][2] * vector[2]
        for row in range(3)
    ]


def _unpack_float32_le(frame, count):
    try:
        import ustruct as struct
    except ImportError:
        import struct
    return struct.unpack("<{}f".format(count), frame)
