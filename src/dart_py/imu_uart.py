"""普通 UART IMU 通信接口。"""
import math

IMU_FRAME_BYTES = 36
IMU_FLOAT_COUNT = 9
IMU_DATA_FLOAT_COUNT = 8
IMU_FRAME_TAIL = b"\x00\x00\x80\x7f"
IMU_PAYLOAD_BYTES = IMU_DATA_FLOAT_COUNT * 4


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

        ax,ay,az,gx,gy,gz,reserved_0, reserved_1, frame_tail

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
        self.max_consecutive_invalid_frames = max(
            1,
            int(self.config.get("max_consecutive_invalid_frames", 20)),
        )
        self._rx_buffer = bytearray()
        self._pending_packets = []
        self.invalid_packet_count = 0
        self.consecutive_invalid_count = 0
        self.imu_fault = False
        self.last_invalid_error = None

    @property
    def pending_packet_count(self):
        return len(self._pending_packets)

    def read(self):
        """由姿态线程调用；读取 UART 并返回一个已解析的最新样本。"""
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

        # 当前下位机协议没有独立帧头，使用 JustFloat 帧尾进行流式重同步。
        # 不能按固定 36 字节切片，否则丢一个字节后所有后续帧都会错位。
        while True:
            tail_index = self._rx_buffer.find(IMU_FRAME_TAIL)
            if tail_index < 0:
                # 保留一个最长候选帧所需的尾部数据，等待下一次 read() 补全。
                # 只保留帧尾前缀会丢掉已收到的半帧有效载荷。
                keep_bytes = IMU_PAYLOAD_BYTES + len(IMU_FRAME_TAIL) - 1
                if len(self._rx_buffer) > keep_bytes:
                    self._rx_buffer = self._rx_buffer[-keep_bytes:]
                return

            if tail_index < IMU_PAYLOAD_BYTES:
                # 帧尾前没有足够的 32 字节数据，丢弃这段不完整/乱码并继续找。
                del self._rx_buffer[:tail_index + len(IMU_FRAME_TAIL)]
                continue

            frame_end = tail_index + len(IMU_FRAME_TAIL)
            frame = bytes(self._rx_buffer[tail_index - IMU_PAYLOAD_BYTES:frame_end])
            del self._rx_buffer[:frame_end]
            self._append_packet(frame)

    def _append_packet(self, frame):
        try:
            packet = self._parse_packet(frame)
        except (TypeError, ValueError, IndexError) as exc:
            self._record_invalid_frame(exc)
            return
        if packet is None:
            self._record_invalid_frame(ValueError("parser returned an empty packet"))
            return
        self.consecutive_invalid_count = 0
        self.imu_fault = False
        self.last_invalid_error = None
        self._pending_packets.append(packet)
        if len(self._pending_packets) > self.max_pending_packets:
            # 保留最新样本，防止图像匹配滞后。
            del self._pending_packets[0]

    def _record_invalid_frame(self, error):
        """记录坏帧并保留运行；连续异常达到阈值时标记 IMU 失效。"""
        self.invalid_packet_count += 1
        self.consecutive_invalid_count += 1
        self.last_invalid_error = str(error)
        if self.consecutive_invalid_count >= self.max_consecutive_invalid_frames:
            self.imu_fault = True

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
        # 帧尾按原始字节校验；00 00 80 7F 作为小端 float32 是 +Inf，不能
        # 与前八个传感器数据一起执行有限值检查。
        if frame[IMU_DATA_FLOAT_COUNT * 4:] != IMU_FRAME_TAIL:
            raise ValueError("UART binary frame tail is invalid")
        values = _unpack_float32_le(
            frame[:IMU_DATA_FLOAT_COUNT * 4],
            IMU_DATA_FLOAT_COUNT,
        )
        if not all(_is_finite(value) for value in values):
            raise ValueError("UART binary frame contains non-finite float")
        return {
            "ax": values[0],
            "ay": values[1],
            "az": values[2],
            "gx": values[3],
            "gy": values[4],
            "gz": values[5],
            "reserved_0": values[6],
            "reserved_1": values[7],
            "frame_tail": IMU_FRAME_TAIL,
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


def _is_finite(value):
    """拒绝 NaN/Inf，避免非法 IMU 数据污染姿态积分。"""
    try:
        return math.isfinite(value)
    except AttributeError:
        return value == value and value != float("inf") and value != -float("inf")
