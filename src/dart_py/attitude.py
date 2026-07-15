"""高频 IMU 采样与带时间戳的滚转姿态查询。

相机/引导回路不应直接读取 IMU：推理和显示可能使该回路延迟数毫秒。
``AttitudeWorker`` 独占 IMU 读取，在自己的线程中积分滚转角，
并维护一份带时间戳的短历史记录供相机回路查询。
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
    """返回 CanMV 和 CPython 上的单调微秒时间戳。"""
    if hasattr(time, "ticks_us"):
        return time.ticks_us()
    return int(time.time() * 1000000)


def ticks_diff(newer, older):
    """返回 ``newer - older``，同时处理 MicroPython 滴答溢出的情况。"""
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
        result = [float(vector[0]), float(vector[1]), float(vector[2])]
        if not all(_is_finite(value) for value in result):
            return None
        return result
    except (TypeError, ValueError, IndexError):
        return None


def _wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def _is_finite(value):
    """兼容精简 MicroPython math 模块的有限数检查。"""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return value == value and value != float("inf") and value != -float("inf")


class AttitudeWorker:
    """以固定速率采样 IMU，并提供图像时刻的姿态信息。

    IMU ``read()`` 数据使用与 ``guidance.py`` 相同的机体坐标系约定：
    ``[x 前, y 右, z 下]``。必须提供 ``gyro_b``（rad/s）和
    ``accel_b``（m/s^2 或任意一致单位）：

    ``{"gyro_b": [p, q, r], "accel_b": [ax, ay, az], "timestamp_us": ...}``

    时间戳是可选的。省略时，工作线程用其单调时钟为读取打上时间戳。
    所有传入 ``state_at`` 的图像时间戳必须使用同一时钟。
    """

    def __init__(self, imu_interface, config=None):
        self.imu_interface = imu_interface
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
        self.hold_last_gyro = bool(self.config.get("hold_last_gyro", True))
        self.max_hold_gyro_us = max(
            self.sample_period_us,
            int(self.config.get("max_hold_gyro_us", 250000)),
        )
        self.roll_axis = int(self.config.get("roll_axis", 0))
        if self.roll_axis < 0 or self.roll_axis > 2:
            raise ValueError("roll_axis must be 0, 1, or 2")
        self.max_match_error_us = max(
            0,
            int(
                self.config.get(
                    "max_match_error_us",
                    # 与本模块第一版的兼容。
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
        self._thread = None
        self._last_sample_us = None
        self._roll_rad = None
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._init_accel_sum = [0.0, 0.0, 0.0]
        self._init_gyro_sum = [0.0, 0.0, 0.0]
        self._init_count = 0
        self._last_gyro_b = None
        self._last_accel_b = None
        self._last_sensor_timestamp_us = None
        self._last_uart_fields = None
        self.last_error = None
        self.last_error_type = None
        self.error_count = 0

    @property
    def initialized(self):
        return self._roll_rad is not None

    def start(self):
        """启动高频工作线程。重复调用是安全的。"""
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
            self._thread = worker
            worker.start()
            return

        # CanMV 提供 _thread。此回退在最小化移植上保留功能行为，
        # 尽管采样只能通过显式调用 sample_once() 来进行。
        self._thread_started = False

    def stop(self):
        """请求工作线程退出循环；此处不关闭接口。"""
        self._running = False

    def join(self, timeout_ms=100):
        """等待 CPython 线程退出；CanMV 线程环境下执行最佳努力停止。"""
        worker = self._thread
        if worker is not None and hasattr(worker, "join"):
            worker.join(max(0, timeout_ms) / 1000.0)

    def sample_once(self, timestamp_us=None):
        """读取一次 IMU 样本。适用于单线程测试移植。"""
        read_timestamp_us = ticks_us() if timestamp_us is None else timestamp_us
        data = self.imu_interface.read() if self.imu_interface is not None else None
        if data is None:
            return self._hold_last_gyro(read_timestamp_us)
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

        self._update_roll(sample_timestamp_us, gyro_b, accel_b)
        corrected_gyro_b = self._correct_gyro(gyro_b)
        self._last_gyro_b = gyro_b[:]
        self._last_accel_b = accel_b[:]
        self._last_sensor_timestamp_us = sample_timestamp_us
        self._last_uart_fields = data.get("uart_fields")
        sample = {
            "timestamp_us": sample_timestamp_us,
            "source_timestamp_us": sample_timestamp_us,
            "roll_rad": self._roll_rad,
            "gyro_b": corrected_gyro_b,
            "accel_b": accel_b,
            "initialized": self.initialized,
            "gyro_held": False,
            "source_age_us": 0,
        }
        if "uart_fields" in data:
            sample["uart_fields"] = data["uart_fields"].copy()
        self._append_history(sample)
        return sample

    def _hold_last_gyro(self, timestamp_us):
        """Integrate with the last UART gyro sample between low-rate packets."""
        if (
            not self.hold_last_gyro
            or not self.initialized
            or self._last_gyro_b is None
            or self._last_sensor_timestamp_us is None
        ):
            return None
        source_age_us = ticks_diff(timestamp_us, self._last_sensor_timestamp_us)
        if source_age_us < 0 or source_age_us > self.max_hold_gyro_us:
            return None

        self._update_roll(timestamp_us, self._last_gyro_b, self._last_accel_b)
        sample = {
            "timestamp_us": timestamp_us,
            "source_timestamp_us": self._last_sensor_timestamp_us,
            "roll_rad": self._roll_rad,
            "gyro_b": self._correct_gyro(self._last_gyro_b),
            "accel_b": self._last_accel_b[:],
            "initialized": True,
            "gyro_held": True,
            "source_age_us": source_age_us,
        }
        if self._last_uart_fields is not None:
            sample["uart_fields"] = self._last_uart_fields.copy()
        self._append_history(sample)
        return sample

    def state_at(self, image_timestamp_us, max_age_us=None):
        """返回距图像时间戳最近的 IMU 状态。

        查询会考虑图像时间两侧的样本，选择绝对时间戳差最小的。
        这对 UART 流很重要：相机可能在两个 IMU 报文之间运行。
        结果经过拷贝，因此引导回路不会看到环形缓冲区中被部分覆盖的条目。
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
        # 为兼容日志和可视化的数值格式，初始化窗口内返回 0.0；调用方必须
        # 以 initialized 字段判断该滚转值是否可用于制导补偿。
        if result.get("roll_rad") is None:
            result["roll_rad"] = 0.0
        # 正差值表示传感器样本比图像更新；
        # 负差值表示样本在图像之前采集。
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
            except Exception as exc:
                # 解析器会吞掉单帧坏数据；这里仅处理真正的线程/接口异常。
                self.last_error = str(exc)
                self.last_error_type = type(exc).__name__
                self.error_count += 1
                self._running = False
                return

            next_sample_us = ticks_add(next_sample_us, self.sample_period_us)
            remaining_us = ticks_diff(next_sample_us, ticks_us())
            if remaining_us > 0:
                sleep_us(remaining_us)
            else:
                # 慢速读取后不累积过大的调度误差。
                next_sample_us = ticks_us()

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
            # 上电校准仅在平台静止时有效。
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

        # 对于 x-前/y-右/z-下 的机体坐标系，atan2(g_y, g_z) 即为常规滚转角。
        # guidance.py 在将机体视线转换为稳定坐标系时，应用其可配置的 roll_sign。
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
    if "uart_fields" in sample:
        result["uart_fields"] = sample["uart_fields"].copy()
    return result
