"""制导指令输出接口。"""
import math

try:
    import ustruct as struct
except ImportError:
    import struct


COMMAND_FRAME_HEAD = b"\x5a\xa5"
COMMAND_FRAME_BYTES = 11
COMMAND_BAUDRATE = 115200


class OverloadCommandOutput:
    """最终过载指令的输出接口。"""

    def __init__(self, config=None):
        self.config = config or {}
        self.console_output = bool(self.config.get("console_output", False))

    def send_overload(self, command):
        """向下位机发送一个指令字典。默认实现为空操作。"""
        return True

    def deinit(self):
        """释放输出设备资源。"""
        return None


class ConsoleCommandOutput(OverloadCommandOutput):
    """可选的调试输出，将指令字典打印到控制台。"""

    def send_overload(self, command):
        if self.console_output:
            _print_command(command)
        return True


class SerialCommandOutput(OverloadCommandOutput):
    """通过 UART 发送下位机要求的 11 字节制导指令帧。"""

    def __init__(self, config=None, uart=None):
        OverloadCommandOutput.__init__(self, config)
        self._owns_uart = uart is None
        self.uart = uart if uart is not None else _initialize_command_uart(self.config)
        imu_to_body = self.config.get("imu_to_body", _identity_matrix())
        self.body_to_imu = _transpose_matrix(imu_to_body)
        self.lateral_imu_axis = _axis_index(
            self.config.get("lateral_imu_axis", 0), "lateral_imu_axis"
        )
        self.normal_imu_axis = _axis_index(
            self.config.get("normal_imu_axis", 2), "normal_imu_axis"
        )
        if self.lateral_imu_axis == self.normal_imu_axis:
            raise ValueError("lateral_imu_axis and normal_imu_axis must differ")

    def send_overload(self, command):
        """将镖体过载转换到 IMU 坐标后发送横向/法向两个配置轴。"""
        command = command or {}
        if command.get("guidance_valid", False):
            body_y = command.get(
                "body_y_overload_g", command.get("yaw_overload_g", 0.0)
            )
            body_z = command.get(
                "body_z_overload_g", command.get("pitch_overload_g", 0.0)
            )
        else:
            # 无有效制导结果时向下位机发送零指令，避免沿用上一帧指令。
            body_y = 0.0
            body_z = 0.0
        imu_command = _mat_vec_mul(self.body_to_imu, [0.0, body_y, body_z])
        ny = imu_command[self.lateral_imu_axis]
        nz = imu_command[self.normal_imu_axis]
        frame = pack_command_frame(ny, nz)
        if self.uart is None:
            return False
        self.uart.write(frame)
        if self.console_output:
            _print_command(command)
        return True

    def deinit(self):
        # IMU 接收和指令发送可能复用同一个 UART，不能由输出端关闭共享资源。
        if self._owns_uart and self.uart is not None and hasattr(self.uart, "deinit"):
            self.uart.deinit()
        self.uart = None


def pack_command_frame(ny, nz):
    """构造下位机 11 字节帧：5A A5、两个小端 float32 和 8 位累加和。"""
    ny = _finite_float(ny, "ny")
    nz = _finite_float(nz, "nz")
    frame = bytearray(COMMAND_FRAME_HEAD)
    frame.extend(struct.pack("<2f", ny, nz))
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


def _finite_float(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError("{} must be a finite number".format(name))
    if not math.isfinite(value):
        raise ValueError("{} must be a finite number".format(name))
    return value


def _initialize_command_uart(config):
    """创建 USART1 对应的 UART；参数固定为 115200、8N1。"""
    try:
        from imu_uart import initialize_imu_uart
    except ImportError:
        raise RuntimeError("imu_uart.initialize_imu_uart is required for command UART")
    uart_config = config.copy()
    uart_config["baudrate"] = int(config.get("baudrate", COMMAND_BAUDRATE))
    uart_config.setdefault("uart_id", "UART1")
    return initialize_imu_uart(uart_config)


def _identity_matrix():
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _axis_index(value, name):
    try:
        value = int(value)
    except (TypeError, ValueError):
        raise ValueError("{} must be 0, 1, or 2".format(name))
    if value < 0 or value > 2:
        raise ValueError("{} must be 0, 1, or 2".format(name))
    return value


def _transpose_matrix(matrix):
    if matrix is None or len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("imu_to_body must be a 3x3 matrix")
    return [[float(matrix[col][row]) for col in range(3)] for row in range(3)]


def _mat_vec_mul(matrix, vector):
    return [
        matrix[row][0] * vector[0]
        + matrix[row][1] * vector[1]
        + matrix[row][2] * vector[2]
        for row in range(3)
    ]


def make_lower_computer_interface(config, uart=None):
    command_config = config.get("command", {}).copy()
    if not command_config.get("enabled", True):
        return OverloadCommandOutput(config)
    # 下位机使用 IMU 坐标；复用 IMU 输入侧已经配置好的装配矩阵。
    if "imu_to_body" not in command_config:
        command_config["imu_to_body"] = config.get("imu", {}).get(
            "accel_to_body", _identity_matrix()
        )
    # 当前默认 body=[imu_y, imu_x, -imu_z]，所以镖体 y/z 过载对应 IMU x/z。
    command_config.setdefault("lateral_imu_axis", 0)
    command_config.setdefault("normal_imu_axis", 2)
    command_config["console_output"] = bool(
        config.get("console", {}).get("command", False)
    )
    return SerialCommandOutput(command_config or config, uart=uart)


def _print_command(command):
    """输出一帧精简指令调试信息。"""
    print(
        "cmd detected={} predicted={} valid={} pitch_g={:.3f} "
        "yaw_g={:.3f} fps={:.1f}".format(
            command.get("detected", False),
            command.get("predicted", False),
            command.get("guidance_valid", False),
            command.get("pitch_overload_g", 0.0),
            command.get("yaw_overload_g", 0.0),
            command.get("fps", 0.0),
        )
    )
