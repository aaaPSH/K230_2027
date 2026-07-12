"""Hardware communication interfaces.

The IMU protocol remains application-specific, but its data contract is shared
by the high-rate attitude worker.  GPIO input can be configured directly when
the CanMV ``machine.Pin`` API is available.
"""


class LowerComputerInterface:
    """Output interface for final overload commands."""

    def __init__(self, config=None):
        self.config = config or {}

    def send_overload(self, command):
        """Send one command dictionary to the lower computer.

        Default implementation is a no-op and returns True so the guidance loop
        can run on a bare K230/IDE setup before the physical link is wired.
        """
        return True


class GyroInterface:
    """Input interface for one raw IMU sample.

    Subclasses should return ``None`` or a dictionary containing ``gyro_b``
    and ``accel_b``.  Both are body-frame vectors ordered as
    ``[x forward, y right, z down]``.  Gyro units must be rad/s.  The optional
    ``timestamp_us`` must use ``time.ticks_us()``'s time base.
    """

    def __init__(self, config=None):
        self.config = config or {}

    def read(self):
        """Return None or {'gyro_b': [p, q, r], 'accel_b': [ax, ay, az]}."""
        return None


# ``ImuInterface`` is the preferred name.  Keep GyroInterface as an alias in
# the public API so existing IMU drivers do not need an immediate rename.
ImuInterface = GyroInterface


class UartImuInterface(GyroInterface):
    """Receive newline-framed IMU packets from a UART stream.

    The default CSV packet is compact enough for a high-rate link::

        gx,gy,gz,ax,ay,az,gpio_bits\\n

    Gyro values are rad/s; acceleration values use any consistent unit.  JSON
    lines are also accepted when ``packet_format`` is ``"json"``; e.g.::

        {"gyro_b":[gx,gy,gz],"accel_b":[ax,ay,az],"gpio":{"armed":1}}\\n

    By default packet timestamps are created on K230 when a complete UART line
    arrives.  This gives the image and sensor data a shared time base.  Select
    ``timestamp_source="packet"`` only when the sender's ``timestamp_us`` has
    already been synchronized to K230's ``time.ticks_us()`` clock.
    """

    def __init__(self, config=None):
        GyroInterface.__init__(self, config)
        try:
            from machine import UART
        except ImportError:
            raise RuntimeError("machine.UART is required for the UART IMU link")

        uart_id = self.config.get("uart_id", 1)
        if isinstance(uart_id, str):
            uart_id = getattr(UART, uart_id, None)
        if uart_id is None:
            raise ValueError("invalid UART id")

        bits = self.config.get("bits", 8)
        if bits == 8:
            bits = getattr(UART, "EIGHTBITS", bits)
        parity = self.config.get("parity", "none")
        if parity == "none":
            parity = getattr(UART, "PARITY_NONE", parity)
        stop = self.config.get("stop", 1)
        if stop == 1:
            stop = getattr(UART, "STOPBITS_ONE", stop)
        self.uart = UART(
            uart_id,
            baudrate=int(self.config.get("baudrate", 921600)),
            bits=bits,
            parity=parity,
            stop=stop,
        )

        self.packet_format = self.config.get("packet_format", "csv").lower()
        self.timestamp_source = self.config.get("timestamp_source", "arrival")
        self.csv_fields = self.config.get(
            "csv_fields",
            ["gx", "gy", "gz", "ax", "ay", "az", "gpio_bits"],
        )
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

        while True:
            line_end = self._rx_buffer.find(b"\n")
            if line_end < 0:
                break
            line = bytes(self._rx_buffer[:line_end]).strip()
            del self._rx_buffer[: line_end + 1]
            if line:
                self._append_packet(line)

        # Do not allow a damaged/no-newline packet to consume the heap forever.
        if len(self._rx_buffer) > self.max_line_bytes:
            self._rx_buffer = bytearray()
            self.invalid_packet_count += 1

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
            # Retain the most recent samples so image matching cannot lag.
            del self._pending_packets[0]

    def _parse_packet(self, line):
        arrival_timestamp_us = _ticks_us()
        if self.packet_format == "json":
            record = _json_loads(line)
        elif self.packet_format == "csv":
            record = self._parse_csv(line)
        else:
            raise ValueError("unsupported UART IMU packet_format")
        if not isinstance(record, dict):
            raise ValueError("UART IMU packet must decode to a dictionary")

        gyro_b = _packet_vector(record, "gyro_b", ("gx", "gy", "gz"))
        accel_b = _packet_vector(record, "accel_b", ("ax", "ay", "az"))
        if gyro_b is None or accel_b is None:
            raise ValueError("UART IMU packet has no complete IMU vector")

        packet_timestamp_us = _packet_integer(record.get("timestamp_us"))
        timestamp_us = arrival_timestamp_us
        if self.timestamp_source == "packet" and packet_timestamp_us is not None:
            timestamp_us = packet_timestamp_us

        sample = {
            "timestamp_us": timestamp_us,
            "gyro_b": gyro_b,
            "accel_b": accel_b,
        }
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


class GPIOInterface:
    """Input interface for GPIO state captured with each IMU sample."""

    def __init__(self, config=None):
        self.config = config or {}

    def read(self):
        """Return None or a dictionary such as {'launch_enable': True}."""
        return None


class MachineGPIOInterface(GPIOInterface):
    """Read named digital inputs through CanMV's ``machine.Pin`` API."""

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
                # One bad pin declaration should not disable other inputs.
                continue

    def read(self):
        values = {}
        for name, pin, active_low in self._pins:
            value = bool(pin.value())
            values[name] = not value if active_low else value
        return values


class ConsoleLowerComputerInterface(LowerComputerInterface):
    """Optional debug sender that prints the command dictionary."""

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


def make_lower_computer_interface(config):
    if config.get("debug_print", False):
        return ConsoleLowerComputerInterface(config)
    return LowerComputerInterface(config)


def make_gyro_interface(config):
    return GyroInterface(config)


def make_imu_interface(config):
    """Create the configured IMU transport."""
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


def _packet_integer(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None
