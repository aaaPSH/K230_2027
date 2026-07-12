"""硬件通信接口。

IMU 协议仍是应用相关的，但其数据契约由高频姿态工作线程共享。
当 CanMV 的 ``machine.Pin`` API 可用时，可以直接配置 GPIO 输入。
"""


class LowerComputerInterface:
    """最终过载指令的输出接口。"""

    def __init__(self, config=None):
        self.config = config or {}

    def send_overload(self, command):
        """向下位机发送一个指令字典。

        默认实现为空操作，返回 True，以便在物理链路接线之前，
        引导回路可以在裸 K230/IDE 环境中运行。
        """
        return True


class GyroInterface:
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


# ``ImuInterface`` 是推荐名称。在公开 API 中保留 GyroInterface 作为别名，
# 以便现有的 IMU 驱动无需立即重命名。
ImuInterface = GyroInterface


class UartImuInterface(GyroInterface):
    """从 UART 流中接收 IMU 报文。

    默认协议为固定长度二进制帧：小端 IEEE-754 ``float32 * 9``，共 36 字节。
    前六个 float 的发送顺序为::

        ay,ax,az,gy,gx,gz,reserved_0,reserved_1,reserved_2

    字段映射会先重排为 ``accel=[ax,ay,az]``、``gyro=[gx,gy,gz]``，再通过
    可配置装配矩阵转换为制导使用的镖体系 ``accel_b``、``gyro_b``。
    当 ``packet_format`` 设为 ``"json"`` 时也接受 JSON 行，例如::

        {"gyro_b":[gx,gy,gz],"accel_b":[ax,ay,az],"gpio":{"armed":1}}\\n

    默认情况下，报文时间戳在 K230 收到完整 UART 帧时生成。
    这使得图像和传感器数据共享同一时间基准。
    仅当发送方的 ``timestamp_us`` 已与 K230 的 ``time.ticks_us()``
    时钟同步时，才选择 ``timestamp_source="packet"``。
    """

    def __init__(self, config=None):
        GyroInterface.__init__(self, config)
        self.uart = initialize_uart(self.config)

        self.packet_format = self.config.get("packet_format", "csv").lower()
        self.timestamp_source = self.config.get("timestamp_source", "arrival")
        self.csv_fields = self.config.get(
            "csv_fields",
            ["ay", "ax", "az", "gy", "gx", "gz"],
        )
        self.accel_fields = self.config.get("accel_fields", ["ax", "ay", "az"])
        self.gyro_fields = self.config.get("gyro_fields", ["gx", "gy", "gz"])
        self.gyro_unit = self.config.get("gyro_unit", "rad_s")
        self.accel_to_body = _axis_transform(
            self.config.get("accel_to_body", _identity_matrix()),
            "accel_to_body",
        )
        self.gyro_to_body = _axis_transform(
            self.config.get("gyro_to_body", _identity_matrix()),
            "gyro_to_body",
        )
        self.frame_bytes = int(self.config.get("frame_bytes", 0))
        self.binary_fields = self.config.get(
            "binary_fields",
            [
                "ay", "ax", "az", "gy", "gx", "gz",
                "reserved_0", "reserved_1", "reserved_2",
            ],
        )
        if self.packet_format == "binary_float32_le":
            if self.frame_bytes <= 0:
                self.frame_bytes = len(self.binary_fields) * 4
            if self.frame_bytes != len(self.binary_fields) * 4:
                raise ValueError("binary frame_bytes must equal 4 * field count")
        self.max_line_bytes = max(32, int(self.config.get("max_line_bytes", 256)))
        self.max_pending_packets = max(
            1,
            int(self.config.get("max_pending_packets", 32)),
        )
        self._rx_buffer = bytearray()
        self._pending_packets = []
        self.invalid_packet_count = 0

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

        if self.packet_format == "binary_float32_le":
            self._extract_binary_frames()
            return

        while True:
            line_end = self._rx_buffer.find(b"\n")
            if line_end < 0:
                break
            line = bytes(self._rx_buffer[:line_end]).strip()
            del self._rx_buffer[: line_end + 1]
            if line:
                self._append_packet(line)

        # 防止损坏/无换行的报文永久消耗堆内存。
        if len(self._rx_buffer) > self.max_line_bytes:
            self._rx_buffer = bytearray()
            self.invalid_packet_count += 1

    def _extract_binary_frames(self):
        while len(self._rx_buffer) >= self.frame_bytes:
            frame = bytes(self._rx_buffer[:self.frame_bytes])
            del self._rx_buffer[:self.frame_bytes]
            self._append_packet(frame)

    def _append_packet(self, line):
        try:
            packet = self._parse_packet(line)
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

    def _parse_packet(self, line):
        arrival_timestamp_us = _ticks_us()
        if self.packet_format == "json":
            record = _json_loads(line)
        elif self.packet_format == "csv":
            record = self._parse_csv(line)
        elif self.packet_format == "binary_float32_le":
            record = self._parse_binary_frame(line)
        else:
            raise ValueError("unsupported UART IMU packet_format")
        if not isinstance(record, dict):
            raise ValueError("UART IMU packet must decode to a dictionary")

        gyro_b = _packet_vector(record, "gyro_b", self.gyro_fields)
        accel_b = _packet_vector(record, "accel_b", self.accel_fields)
        if gyro_b is None or accel_b is None:
            raise ValueError("UART IMU packet has no complete IMU vector")
        accel_b = _mat_vec_mul(self.accel_to_body, accel_b)
        gyro_b = _mat_vec_mul(self.gyro_to_body, gyro_b)
        gyro_b = _gyro_to_rad_s(gyro_b, self.gyro_unit)

        packet_timestamp_us = _packet_integer(record.get("timestamp_us"))
        timestamp_us = arrival_timestamp_us
        if self.timestamp_source == "packet" and packet_timestamp_us is not None:
            timestamp_us = packet_timestamp_us

        sample = {
            "timestamp_us": timestamp_us,
            "gyro_b": gyro_b,
            "accel_b": accel_b,
        }
        if self.packet_format == "binary_float32_le":
            # Preserve all nine values for consumers that need the three
            # protocol fields not used by the roll/guidance path yet.
            sample["uart_fields"] = record
        if packet_timestamp_us is not None:
            sample["packet_timestamp_us"] = packet_timestamp_us
        if "gpio" in record:
            sample["gpio"] = record["gpio"]
        elif "gpio_bits" in record:
            gpio_bits = _packet_integer(record["gpio_bits"])
            if gpio_bits is not None:
                sample["gpio"] = {"gpio_bits": gpio_bits}
        return sample

    def _parse_csv(self, line):
        try:
            values = line.decode().strip().split(",")
        except (AttributeError, UnicodeError):
            raise ValueError("UART IMU CSV must be UTF-8")
        if len(values) < len(self.csv_fields):
            raise ValueError("UART IMU CSV field count is too small")
        record = {}
        for index, name in enumerate(self.csv_fields):
            if name:
                record[name] = values[index].strip()
        return record

    def _parse_binary_frame(self, frame):
        if len(frame) != self.frame_bytes:
            raise ValueError("UART binary frame length is invalid")
        values = _unpack_float32_le(frame, len(self.binary_fields))
        return {
            name: values[index]
            for index, name in enumerate(self.binary_fields)
            if name
        }


class GPIOInterface:
    """IMU 采样时同步捕获的 GPIO 状态的输入接口。"""

    def __init__(self, config=None):
        self.config = config or {}

    def read(self):
        """返回 None 或类似 {'launch_enable': True} 的字典。"""
        return None


class MachineGPIOInterface(GPIOInterface):
    """通过 CanMV 的 ``machine.Pin`` API 读取命名的数字量输入。"""

    def __init__(self, config=None):
        GPIOInterface.__init__(self, config)
        try:
            from machine import Pin
        except ImportError:
            self._pins = []
            return

        self._pins = []
        for item in self.config.get("pins", []):
            if not isinstance(item, dict) or "pin" not in item:
                continue
            pull = None
            pull_name = item.get("pull")
            if pull_name == "up":
                pull = getattr(Pin, "PULL_UP", None)
            elif pull_name == "down":
                pull = getattr(Pin, "PULL_DOWN", None)
            try:
                if pull is None:
                    pin = Pin(item["pin"], Pin.IN)
                else:
                    pin = Pin(item["pin"], Pin.IN, pull=pull)
                self._pins.append(
                    (
                        item.get("name", "gpio_{}".format(item["pin"])),
                        pin,
                        bool(item.get("active_low", False)),
                    )
                )
            except Exception:
                # 一个引脚的配置错误不应导致其他输入失效。
                continue

    def read(self):
        values = {}
        for name, pin, active_low in self._pins:
            value = bool(pin.value())
            values[name] = not value if active_low else value
        return values


class ConsoleLowerComputerInterface(LowerComputerInterface):
    """可选的调试发送器，将指令字典打印到控制台。"""

    def send_overload(self, command):
        print(
            "cmd detected={} pitch_g={:.3f} yaw_g={:.3f} fps={:.1f}".format(
                command.get("detected", False),
                command.get("pitch_overload_g", 0.0),
                command.get("yaw_overload_g", 0.0),
                command.get("fps", 0.0),
            )
        )
        return True


class DebugUartLowerComputerInterface(LowerComputerInterface):
    """通过独立 UART 发送关键制导数据的 JSON 调试报文。"""

    def __init__(self, config=None):
        LowerComputerInterface.__init__(self, config)
        self.period_ms = max(0, int(self.config.get("debug_period_ms", 50)))
        self._last_output_ms = None
        self.uart = initialize_uart(self.config)
        self.rs485 = _initialize_rs485_pins(self.config)

    def send_overload(self, command):
        if not _debug_due(self):
            return True
        payload = _json_dumps(_debug_payload(command)) + "\n"
        if isinstance(payload, str):
            payload = payload.encode()

        _set_rs485_transmit(self.rs485, True)
        try:
            self.uart.write(payload)
            flush = getattr(self.uart, "flush", None)
            if callable(flush):
                flush()
        finally:
            _set_rs485_transmit(self.rs485, False)
        return True

    def deinit(self):
        if self.uart is not None and hasattr(self.uart, "deinit"):
            self.uart.deinit()
        self.uart = None
        self.rs485 = None


def make_lower_computer_interface(config):
    debug_uart_config = config.get("debug_uart", {})
    if debug_uart_config.get("enabled", False):
        return DebugUartLowerComputerInterface(debug_uart_config)
    if config.get("debug_print", False):
        return ConsoleLowerComputerInterface(config)
    return LowerComputerInterface(config)


def _debug_due(interface):
    """按配置限频，避免调试输出阻塞图像主循环。"""
    now_ms = _ticks_ms()
    if (
        interface._last_output_ms is not None
        and interface.period_ms > 0
        and now_ms - interface._last_output_ms < interface.period_ms
    ):
        return False
    interface._last_output_ms = now_ms
    return True


def _debug_payload(command):
    """提取需要通过串口发送的关键调试字段。"""
    fields = (
        "target_x", "target_y",
        "yaw", "pitch", "yaw_rate", "pitch_rate",
        "gyro_x", "gyro_y", "gyro_z",
        "yaw_cmd_g", "pitch_cmd_g", "roll",
    )
    return {name: command.get(name, 0.0) for name in fields}


def make_gyro_interface(config):
    return GyroInterface(config)


def make_imu_interface(config):
    """创建配置指定的 IMU 传输实例。"""
    imu_config = config.get("imu", {})
    if imu_config.get("enabled", False):
        if imu_config.get("transport", "uart") == "uart":
            return UartImuInterface(imu_config)
        raise ValueError("unsupported IMU transport")
    return make_gyro_interface(config)


def make_gpio_interface(config):
    gpio_config = config.get("gpio", {})
    if not gpio_config.get("enabled", False):
        return GPIOInterface(gpio_config)
    return MachineGPIOInterface(gpio_config)


def initialize_uart(config):
    """Configure UART IOMUX pins and create a UART from one config mapping.

    ``tx_pin`` and ``rx_pin`` are optional.  When supplied, their FPIOA
    functions are derived from ``uart_id`` (or overridden by
    ``tx_function``/``rx_function``).  This keeps pin assignment and UART
    parameters in the same ``COMM_CONFIG['imu']`` section.
    """
    try:
        from machine import UART
    except ImportError:
        raise RuntimeError("machine.UART is required for the UART IMU link")

    uart_id, uart_name = _resolve_uart_id(UART, config.get("uart_id", "UART1"))
    _configure_uart_pins(config, uart_name)

    bits = config.get("bits", 8)
    if bits == 8:
        bits = getattr(UART, "EIGHTBITS", bits)
    parity = config.get("parity", "none")
    if parity == "none":
        parity = getattr(UART, "PARITY_NONE", parity)
    stop = config.get("stop", 1)
    if stop == 1:
        stop = getattr(UART, "STOPBITS_ONE", stop)

    uart_kwargs = {
        "baudrate": int(config.get("baudrate", 921600)),
        "bits": bits,
        "parity": parity,
        "stop": stop,
    }
    if "timeout" in config:
        uart_kwargs["timeout"] = int(config["timeout"])
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


def _initialize_rs485_pins(config):
    """初始化可选的 RS-485 DE 和 /RE 控制脚。"""
    de_pin = config.get("de_pin")
    re_pin = config.get("re_pin")
    if de_pin is None and re_pin is None:
        return None
    try:
        from machine import Pin
    except ImportError:
        raise RuntimeError("machine.Pin is required for RS-485 direction control")

    re_active_low = bool(config.get("re_active_low", True))
    pins = {"de": None, "re": None, "re_active_low": re_active_low}
    if de_pin is not None:
        pins["de"] = Pin(int(de_pin), Pin.OUT)
        pins["de"].value(0)
    if re_pin is not None:
        pins["re"] = Pin(int(re_pin), Pin.OUT)
        pins["re"].value(_rs485_receive_re_value(re_active_low))
    return pins


def _set_rs485_transmit(pins, transmitting):
    """切换 RS-485 收发方向，/RE 按低电平有效处理。"""
    if pins is None:
        return
    de = pins.get("de")
    re = pins.get("re")
    re_active_low = pins.get("re_active_low", True)
    if transmitting:
        if re is not None:
            re.value(_rs485_transmit_re_value(re_active_low))
        if de is not None:
            de.value(1)
    else:
        if de is not None:
            de.value(0)
        if re is not None:
            re.value(_rs485_receive_re_value(re_active_low))


def _rs485_receive_re_value(active_low):
    return 0 if active_low else 1


def _rs485_transmit_re_value(active_low):
    return 1 if active_low else 0


def _fpioa_function(fpioa, FPIOA, name):
    function = getattr(FPIOA, name, None)
    if function is None:
        function = getattr(fpioa, name, None)
    if function is None:
        raise ValueError("unsupported FPIOA function: {}".format(name))
    return function


def _ticks_ms():
    try:
        import time
        if hasattr(time, "ticks_ms"):
            return time.ticks_ms()
        return int(time.time() * 1000)
    except Exception:
        return 0


def _ticks_us():
    try:
        import time
        if hasattr(time, "ticks_us"):
            return time.ticks_us()
        return int(time.time() * 1000000)
    except Exception:
        return 0


def _json_loads(line):
    try:
        import ujson as json
    except ImportError:
        import json
    if not isinstance(line, str):
        line = line.decode()
    return json.loads(line)


def _json_dumps(value):
    try:
        import ujson as json
    except ImportError:
        import json
    return json.dumps(value)


def _packet_vector(record, vector_name, scalar_names):
    vector = record.get(vector_name)
    if vector is not None:
        try:
            if len(vector) >= 3:
                return [float(vector[0]), float(vector[1]), float(vector[2])]
        except (TypeError, ValueError, IndexError):
            return None
    try:
        return [float(record[name]) for name in scalar_names]
    except (KeyError, TypeError, ValueError):
        return None


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


def _packet_integer(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _unpack_float32_le(frame, count):
    try:
        import ustruct as struct
    except ImportError:
        import struct
    return struct.unpack("<{}f".format(count), frame)


def _gyro_to_rad_s(gyro_b, unit):
    if unit == "rad_s":
        return gyro_b
    if unit == "deg_s":
        return [value * 0.017453292519943295 for value in gyro_b]
    raise ValueError("unsupported gyro_unit: {}".format(unit))
