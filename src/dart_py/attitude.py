"""High-rate IMU/GPIO sampling and timestamped roll-state lookup.

The camera/guidance loop must not read an IMU directly: inference and display
can delay that loop by many milliseconds.  ``AttitudeWorker`` owns the IMU and
GPIO reads, integrates roll in its own thread, and keeps a short timestamped
history for the camera loop to query.
"""
import math
import time

try:
    import _thread
except ImportError:
    _thread = None

try:
    import threading
except ImportError:
    threading = None


def ticks_us():
    """Return a monotonic timestamp in microseconds on CanMV and CPython."""
    if hasattr(time, "ticks_us"):
        return time.ticks_us()
    return int(time.time() * 1000000)


def ticks_diff(newer, older):
    """Return ``newer - older`` while handling MicroPython tick wraparound."""
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(newer, older)
    return newer - older


def ticks_add(timestamp, delta_us):
    if hasattr(time, "ticks_add"):
        return time.ticks_add(timestamp, delta_us)
    return timestamp + delta_us


def sleep_us(delay_us):
    if delay_us <= 0:
        return
    if hasattr(time, "sleep_us"):
        time.sleep_us(delay_us)
    else:
        time.sleep(delay_us / 1000000.0)


class _NoopLock:
    def acquire(self):
        return True

    def release(self):
        return True


def _make_lock():
    if _thread is not None and hasattr(_thread, "allocate_lock"):
        return _thread.allocate_lock()
    if threading is not None:
        return threading.Lock()
    return _NoopLock()


def _copy_vector(vector):
    if vector is None:
        return None
    try:
        if len(vector) < 3:
            return None
        return [float(vector[0]), float(vector[1]), float(vector[2])]
    except (TypeError, ValueError, IndexError):
        return None


def _wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class AttitudeWorker:
    """Sample IMU/GPIO at a fixed rate and provide image-time attitude.

    IMU ``read()`` data uses the body coordinate convention already used by
    ``guidance.py``: ``[x forward, y right, z down]``.  It must provide
    ``gyro_b`` in rad/s and ``accel_b`` in m/s^2 (or any consistent unit):

    ``{"gyro_b": [p, q, r], "accel_b": [ax, ay, az], "timestamp_us": ...}``

    The timestamp is optional.  When omitted, the worker timestamps the read
    with its monotonic clock.  All image timestamps passed to ``state_at`` must
    use the same clock.
    """

    def __init__(self, imu_interface, gpio_interface=None, config=None):
        self.imu_interface = imu_interface
        self.gpio_interface = gpio_interface
        self.config = config or {}

        self.sample_period_us = max(100, int(self.config.get("sample_period_us", 1000)))
        self.max_dt_us = max(
            self.sample_period_us,
            int(self.config.get("max_dt_us", self.sample_period_us * 5)),
        )
        self.history_size = max(2, int(self.config.get("history_size", 512)))
        self.initial_roll_samples = max(1, int(self.config.get("initial_roll_samples", 32)))
        self.stationary_gyro_rad_s = float(
            self.config.get("stationary_gyro_rad_s", 0.15)
        )
        self.estimate_gyro_bias = bool(self.config.get("estimate_gyro_bias", True))
        self.accel_gravity_sign = float(self.config.get("accel_gravity_sign", -1.0))
        self.roll_axis = int(self.config.get("roll_axis", 0))
        if self.roll_axis < 0 or self.roll_axis > 2:
            raise ValueError("roll_axis must be 0, 1, or 2")
        self.max_match_error_us = max(
            0,
            int(
                self.config.get(
                    "max_match_error_us",
                    # Compatibility with the first version of this module.
                    self.config.get(
                        "max_lookup_age_us",
                        self.history_size * self.sample_period_us,
                    ),
                )
            ),
        )

        self._history = [None] * self.history_size
        self._history_write = 0
        self._history_count = 0
        self._lock = _make_lock()
        self._running = False
        self._thread_started = False
        self._last_sample_us = None
        self._roll_rad = None
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._init_accel_sum = [0.0, 0.0, 0.0]
        self._init_gyro_sum = [0.0, 0.0, 0.0]
        self._init_count = 0
        self.last_error = None

    @property
    def initialized(self):
        return self._roll_rad is not None

    def start(self):
        """Start the high-rate worker.  Calling it twice is safe."""
        if self._thread_started:
            return
        self._running = True
        self._thread_started = True
        if _thread is not None and hasattr(_thread, "start_new_thread"):
            _thread.start_new_thread(self._run, ())
            return
        if threading is not None:
            worker = threading.Thread(target=self._run)
            worker.daemon = True
            worker.start()
            return

        # CanMV provides _thread.  This fallback retains functional behavior
        # on a minimal port, although sampling then happens only on explicit
        # calls to sample_once().
        self._thread_started = False

    def stop(self):
        """Ask the worker to leave its loop; interfaces are not closed here."""
        self._running = False

    def sample_once(self, timestamp_us=None):
        """Read one IMU/GPIO sample.  Useful for single-threaded test ports."""
        read_timestamp_us = ticks_us() if timestamp_us is None else timestamp_us
        # GPIO is sampled on the same high-rate schedule as the IMU.  Reading
        # it first also keeps a physical input active even during a temporary
        # IMU bus outage.
        local_gpio = self._read_gpio()
        data = self.imu_interface.read() if self.imu_interface is not None else None
        if data is None:
            return None
        if not isinstance(data, dict):
            raise ValueError("IMU read() must return a dictionary or None")

        gyro_b = _copy_vector(data.get("gyro_b", data.get("gyro_rad_s")))
        accel_b = _copy_vector(data.get("accel_b", data.get("accel_mps2")))
        if gyro_b is None or accel_b is None:
            raise ValueError("IMU sample requires three-axis gyro_b and accel_b")

        sample_timestamp_us = data.get("timestamp_us", read_timestamp_us)
        try:
            sample_timestamp_us = int(sample_timestamp_us)
        except (TypeError, ValueError):
            sample_timestamp_us = read_timestamp_us

        gpio = _merge_gpio(local_gpio, data.get("gpio"))
        self._update_roll(sample_timestamp_us, gyro_b, accel_b)
        corrected_gyro_b = self._correct_gyro(gyro_b)
        sample = {
            "timestamp_us": sample_timestamp_us,
            "roll_rad": self._roll_rad,
            "gyro_b": corrected_gyro_b,
            "accel_b": accel_b,
            "gpio": gpio,
            "initialized": self.initialized,
        }
        self._append_history(sample)
        return sample

    def state_at(self, image_timestamp_us, max_age_us=None):
        """Return the IMU/GPIO state closest to an image timestamp.

        The lookup considers samples on both sides of the image time and picks
        the smallest absolute timestamp difference.  This is important for a
        UART stream: the camera can run between two IMU packets.  The result is
        copied so the guidance loop never sees a partially overwritten
        ring-buffer item.
        """
        if image_timestamp_us is None:
            return None
        try:
            image_timestamp_us = int(image_timestamp_us)
        except (TypeError, ValueError):
            return None
        if max_age_us is None:
            max_age_us = self.max_match_error_us

        best_sample = None
        best_delta_us = None
        self._lock.acquire()
        try:
            index = (self._history_write - 1) % self.history_size
            for _ in range(self._history_count):
                sample = self._history[index]
                if sample is not None:
                    delta_us = ticks_diff(sample["timestamp_us"], image_timestamp_us)
                    if best_delta_us is None or abs(delta_us) < abs(best_delta_us):
                        best_sample = sample
                        best_delta_us = delta_us
                index = (index - 1) % self.history_size
        finally:
            self._lock.release()
        if best_sample is None:
            return None

        result = _copy_sample(best_sample)
        # Positive delta means the sensor sample is newer than the image;
        # negative means it was sampled before the image.
        result["timestamp_delta_us"] = best_delta_us
        result["timestamp_error_us"] = abs(best_delta_us)
        result["timestamp_match"] = (
            max_age_us <= 0 or abs(best_delta_us) <= max_age_us
        )
        return result

    def _run(self):
        next_sample_us = ticks_us()
        while self._running:
            try:
                self.sample_once()
                self.last_error = None
            except BaseException as exc:
                # An intermittent I2C/SPI read error must not kill attitude
                # updates permanently.  The main loop can inspect last_error.
                self.last_error = str(exc)

            next_sample_us = ticks_add(next_sample_us, self.sample_period_us)
            remaining_us = ticks_diff(next_sample_us, ticks_us())
            if remaining_us > 0:
                sleep_us(remaining_us)
            else:
                # Do not accumulate a large schedule error after a slow read.
                next_sample_us = ticks_us()

    def _read_gpio(self):
        if self.gpio_interface is None:
            return None
        gpio = self.gpio_interface.read()
        if gpio is None:
            return None
        if isinstance(gpio, dict):
            return gpio.copy()
        return {"value": gpio}

    def _update_roll(self, timestamp_us, gyro_b, accel_b):
        if self._roll_rad is None:
            self._collect_initial_attitude(gyro_b, accel_b)
            self._last_sample_us = timestamp_us
            return

        if self._last_sample_us is not None:
            dt_us = ticks_diff(timestamp_us, self._last_sample_us)
            if 0 < dt_us <= self.max_dt_us:
                roll_rate = gyro_b[self.roll_axis] - self._gyro_bias[self.roll_axis]
                self._roll_rad = _wrap_pi(self._roll_rad + roll_rate * dt_us / 1000000.0)
        self._last_sample_us = timestamp_us

    def _collect_initial_attitude(self, gyro_b, accel_b):
        gyro_norm = math.sqrt(
            gyro_b[0] * gyro_b[0] + gyro_b[1] * gyro_b[1] + gyro_b[2] * gyro_b[2]
        )
        if gyro_norm > self.stationary_gyro_rad_s:
            # Power-on calibration is valid only while the platform is still.
            self._init_accel_sum = [0.0, 0.0, 0.0]
            self._init_gyro_sum = [0.0, 0.0, 0.0]
            self._init_count = 0
            return

        for axis in range(3):
            self._init_accel_sum[axis] += self.accel_gravity_sign * accel_b[axis]
            self._init_gyro_sum[axis] += gyro_b[axis]
        self._init_count += 1
        if self._init_count < self.initial_roll_samples:
            return

        gravity_y = self._init_accel_sum[1]
        gravity_z = self._init_accel_sum[2]
        if abs(gravity_y) + abs(gravity_z) < 1e-9:
            return

        # For body axes x-forward/y-right/z-down, atan2(g_y, g_z) is the
        # conventional roll. guidance.py applies its configurable roll_sign
        # when transforming body LOS into its stabilized frame.
        self._roll_rad = math.atan2(gravity_y, gravity_z)
        if self.estimate_gyro_bias:
            self._gyro_bias = [
                value / self._init_count for value in self._init_gyro_sum
            ]

    def _correct_gyro(self, gyro_b):
        if not self.initialized or not self.estimate_gyro_bias:
            return gyro_b[:]
        return [gyro_b[axis] - self._gyro_bias[axis] for axis in range(3)]

    def _append_history(self, sample):
        self._lock.acquire()
        try:
            self._history[self._history_write] = sample
            self._history_write = (self._history_write + 1) % self.history_size
            if self._history_count < self.history_size:
                self._history_count += 1
        finally:
            self._lock.release()


def _copy_sample(sample):
    result = sample.copy()
    result["gyro_b"] = sample["gyro_b"][:]
    result["accel_b"] = sample["accel_b"][:]
    if sample["gpio"] is not None:
        result["gpio"] = sample["gpio"].copy()
    return result


def _merge_gpio(local_gpio, sensor_gpio):
    """Merge local GPIO pins with optional GPIO fields sent over UART."""
    if local_gpio is None and sensor_gpio is None:
        return None
    result = {}
    if isinstance(local_gpio, dict):
        result.update(local_gpio)
    elif local_gpio is not None:
        result["local_value"] = local_gpio
    if isinstance(sensor_gpio, dict):
        result.update(sensor_gpio)
    elif sensor_gpio is not None:
        result["sensor_value"] = sensor_gpio
    return result
