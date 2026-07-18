"""K230 赛后复盘用的轻量级 CSV 飞行日志。"""
import os
import time

try:
    import _thread
except ImportError:
    _thread = None

try:
    import threading
except ImportError:
    threading = None


_INTEGER_FIELD_INDICES = (0, 1, 9, 10)
_FLAG_FIELD_INDICES = (4, 11, 16, 27, 29, 39, 50, 51, 53)
_TEXT_FIELD_INDICES = (28, 38)


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


def _sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _ticks_diff(newer, older):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(newer, older)
    return newer - older


class FlightLogger:
    """主线程入队、后台线程批量写入的逐帧 CSV 日志。"""

    FIELDS = (
        "frame_index",
        "image_timestamp_us",
        "dt_s",
        "fps",
        "detected",
        "target_x",
        "target_y",
        "target_area",
        "target_circularity",
        "imu_source_age_us",
        "imu_timestamp_error_us",
        "imu_fault",
        "roll_rad",
        "gyro_x_rad_s",
        "gyro_y_rad_s",
        "gyro_z_rad_s",
        "gyro_held",
        "raw_yaw_los_angle_rad",
        "raw_pitch_los_angle_rad",
        "raw_yaw_los_rate_rad_s",
        "raw_pitch_los_rate_rad_s",
        "yaw_los_angle_rad",
        "pitch_los_angle_rad",
        "yaw_los_rate_rad_s",
        "pitch_los_rate_rad_s",
        "gyro_yaw_los_rate_correction_rad_s",
        "gyro_pitch_los_rate_correction_rad_s",
        "filter_reinitialized",
        "yaw_kalman_mode",
        "yaw_kalman_rate_initialized",
        "yaw_kalman_predicted_angle_rad",
        "yaw_kalman_predicted_rate_rad_s",
        "yaw_kalman_innovation_residual_rad",
        "yaw_kalman_innovation_variance_rad2",
        "yaw_kalman_innovation_nis",
        "yaw_kalman_covariance_angle_rad2",
        "yaw_kalman_covariance_angle_rate",
        "yaw_kalman_covariance_rate_rad2_s2",
        "pitch_kalman_mode",
        "pitch_kalman_rate_initialized",
        "pitch_kalman_predicted_angle_rad",
        "pitch_kalman_predicted_rate_rad_s",
        "pitch_kalman_innovation_residual_rad",
        "pitch_kalman_innovation_variance_rad2",
        "pitch_kalman_innovation_nis",
        "pitch_kalman_covariance_angle_rad2",
        "pitch_kalman_covariance_angle_rate",
        "pitch_kalman_covariance_rate_rad2_s2",
        "command_yaw_overload_g",
        "command_pitch_overload_g",
        "guidance_valid",
        "guidance_predicted",
        "prediction_age_s",
        "sensor_valid",
    )

    def __init__(self, config=None):
        config = config or {}
        self.enabled = bool(config.get("enabled", False))
        self.console_output = bool(config.get("console_output", False))
        self.flush_interval_frames = max(
            1,
            int(config.get("flush_interval_frames", 60)),
        )
        self.writer_poll_ms = max(1, int(config.get("writer_poll_ms", 10)))
        self.max_pending_buffers = max(
            1,
            int(config.get("max_pending_buffers", 8)),
        )
        self.close_timeout_ms = max(100, int(config.get("close_timeout_ms", 5000)))
        self.path = None
        self._file = None
        self._active_buffer = []
        self._pending_buffers = []
        self._lock = _make_lock()
        self._running = False
        self._writer_active = False
        self._write_error = None
        self.dropped_row_count = 0
        self._thread = None

        if not self.enabled:
            return

        directory = config.get("directory", "/sdcard/dart_py/logs")
        prefix = config.get("file_prefix", "flight")
        _ensure_directory(directory)
        self.path = "{}/{}_{}.csv".format(
            directory.rstrip("/"),
            prefix,
            _session_token(),
        )
        self._file = open(self.path, "w")
        self._file.write(",".join(self.FIELDS) + "\n")
        self._flush_file()
        self._running = True
        self._writer_active = True
        self._start_writer()
        self._console_print("flight log:", self.path)

    def record(
        self,
        frame_index,
        image_timestamp_us,
        fps,
        detection,
        guidance_result,
        command,
    ):
        if not self.enabled:
            return
        write_error = self._get_write_error()
        if write_error is not None:
            # 飞行日志仅用于诊断，写盘失败时自动停用，不能中断控制链路。
            self.enabled = False
            self._console_print("flight log disabled:", write_error)
            return

        snapshot = self.build_snapshot(
            frame_index,
            image_timestamp_us,
            fps,
            detection,
            guidance_result,
            command,
        )
        self._lock.acquire()
        try:
            # 主线程只提取不可变数值快照；字符串格式化和拼接由后台完成。
            self._active_buffer.append(snapshot)
            full = len(self._active_buffer) >= self.flush_interval_frames
        finally:
            self._lock.release()
        if full:
            self.flush()

    @staticmethod
    def build_row(
        frame_index,
        image_timestamp_us,
        fps,
        detection,
        guidance_result,
        command,
    ):
        """构造与 CSV 表头严格一致的一行遥测数据。"""
        snapshot = FlightLogger.build_snapshot(
            frame_index,
            image_timestamp_us,
            fps,
            detection,
            guidance_result,
            command,
        )
        return _format_snapshot(snapshot)

    @staticmethod
    def build_snapshot(
        frame_index,
        image_timestamp_us,
        fps,
        detection,
        guidance_result,
        command,
    ):
        """提取一帧不可变数值快照，避免后台线程引用可变字典。"""

        detection = detection or {}
        guidance_result = guidance_result or {}
        command = command or {}
        gyro_b = guidance_result.get("sensor_gyro_b") or (None, None, None)

        return (
            frame_index,
            image_timestamp_us,
            command.get("dt"),
            fps,
            detection.get("detected"),
            detection.get("x"),
            detection.get("y"),
            detection.get("area"),
            detection.get("circularity"),
            guidance_result.get("sensor_source_age_us"),
            guidance_result.get("sensor_timestamp_error_us"),
            guidance_result.get("sensor_imu_fault"),
            guidance_result.get("sensor_roll_rad"),
            gyro_b[0],
            gyro_b[1],
            gyro_b[2],
            guidance_result.get("sensor_gyro_held"),
            guidance_result.get("raw_yaw_los_angle_rad"),
            guidance_result.get("raw_pitch_los_angle_rad"),
            guidance_result.get("raw_yaw_los_rate_rad_s"),
            guidance_result.get("raw_pitch_los_rate_rad_s"),
            guidance_result.get("yaw_los_angle_rad"),
            guidance_result.get("pitch_los_angle_rad"),
            guidance_result.get("yaw_los_rate_rad_s"),
            guidance_result.get("pitch_los_rate_rad_s"),
            guidance_result.get("gyro_yaw_los_rate_correction_rad_s"),
            guidance_result.get("gyro_pitch_los_rate_correction_rad_s"),
            guidance_result.get("filter_reinitialized"),
            guidance_result.get("yaw_kalman_mode"),
            guidance_result.get("yaw_kalman_rate_initialized"),
            guidance_result.get("yaw_kalman_predicted_angle_rad"),
            guidance_result.get("yaw_kalman_predicted_rate_rad_s"),
            guidance_result.get("yaw_kalman_innovation_residual_rad"),
            guidance_result.get("yaw_kalman_innovation_variance_rad2"),
            guidance_result.get("yaw_kalman_innovation_nis"),
            guidance_result.get("yaw_kalman_covariance_angle_rad2"),
            guidance_result.get("yaw_kalman_covariance_angle_rate"),
            guidance_result.get("yaw_kalman_covariance_rate_rad2_s2"),
            guidance_result.get("pitch_kalman_mode"),
            guidance_result.get("pitch_kalman_rate_initialized"),
            guidance_result.get("pitch_kalman_predicted_angle_rad"),
            guidance_result.get("pitch_kalman_predicted_rate_rad_s"),
            guidance_result.get("pitch_kalman_innovation_residual_rad"),
            guidance_result.get("pitch_kalman_innovation_variance_rad2"),
            guidance_result.get("pitch_kalman_innovation_nis"),
            guidance_result.get("pitch_kalman_covariance_angle_rad2"),
            guidance_result.get("pitch_kalman_covariance_angle_rate"),
            guidance_result.get("pitch_kalman_covariance_rate_rad2_s2"),
            command.get("yaw_overload_g"),
            command.get("pitch_overload_g"),
            guidance_result.get("guidance_valid"),
            guidance_result.get("predicted"),
            guidance_result.get("prediction_age_s"),
            guidance_result.get("sensor_valid"),
        )

    def flush(self):
        """将当前缓冲交给后台写线程；此方法不执行文件 I/O。"""
        if not self.enabled:
            return
        write_error = self._get_write_error()
        if write_error is not None:
            self.enabled = False
            self._console_print("flight log disabled:", write_error)
            return
        self._lock.acquire()
        try:
            if self._active_buffer:
                if len(self._pending_buffers) >= self.max_pending_buffers:
                    self.dropped_row_count += len(self._active_buffer)
                else:
                    self._pending_buffers.append(self._active_buffer)
                self._active_buffer = []
        finally:
            self._lock.release()

    def close(self):
        if self._file is None:
            return
        # 退出时将最后不足一个批次的数据交给写线程，再等待它写完。
        self._lock.acquire()
        try:
            if self._active_buffer:
                self._pending_buffers.append(self._active_buffer)
                self._active_buffer = []
            self._running = False
        finally:
            self._lock.release()

        deadline_ms = _ticks_ms() + self.close_timeout_ms
        while self._writer_is_active():
            if _ticks_diff(_ticks_ms(), deadline_ms) >= 0:
                self._console_print("flight log writer did not stop before timeout")
                return
            _sleep_ms(self.writer_poll_ms)

        self._file.close()
        self._file = None
        write_error = self._get_write_error()
        if write_error is not None:
            self._console_print("flight log closed with writer error:", write_error)

    def _console_print(self, *values):
        """仅在明确启用总控制台输出时打印。"""
        if self.console_output:
            print(*values)

    def _start_writer(self):
        if _thread is not None and hasattr(_thread, "start_new_thread"):
            _thread.start_new_thread(self._writer_loop, ())
            return
        if threading is not None:
            worker = threading.Thread(target=self._writer_loop)
            worker.daemon = True
            self._thread = worker
            worker.start()
            return
        raise RuntimeError("a thread implementation is required for flight logging")

    def _writer_loop(self):
        try:
            while True:
                buffer = self._take_pending_buffer()
                if buffer is not None:
                    rows = []
                    for snapshot in buffer:
                        rows.append(",".join(_format_snapshot(snapshot)))
                    self._file.write("\n".join(rows) + "\n")
                    self._flush_file()
                    continue
                if not self._is_running():
                    return
                _sleep_ms(self.writer_poll_ms)
        except Exception as exc:
            self._set_write_error(exc)
        finally:
            self._lock.acquire()
            try:
                self._writer_active = False
            finally:
                self._lock.release()

    def _take_pending_buffer(self):
        self._lock.acquire()
        try:
            if not self._pending_buffers:
                return None
            return self._pending_buffers.pop(0)
        finally:
            self._lock.release()

    def _is_running(self):
        self._lock.acquire()
        try:
            return self._running
        finally:
            self._lock.release()

    def _writer_is_active(self):
        self._lock.acquire()
        try:
            return self._writer_active
        finally:
            self._lock.release()

    def _set_write_error(self, exc):
        self._lock.acquire()
        try:
            self._write_error = "{}: {}".format(type(exc).__name__, exc)
            self._running = False
        finally:
            self._lock.release()

    def _get_write_error(self):
        self._lock.acquire()
        try:
            return self._write_error
        finally:
            self._lock.release()

    def _flush_file(self):
        if hasattr(self._file, "flush"):
            self._file.flush()


def _format_snapshot(snapshot):
    """在后台将数值快照转换为与 CSV 字段对应的字符串。"""
    if len(snapshot) != len(FlightLogger.FIELDS):
        raise ValueError("flight log snapshot field count mismatch")

    row = []
    for index in range(len(snapshot)):
        value = snapshot[index]
        if index in _INTEGER_FIELD_INDICES:
            row.append(_integer(value))
        elif index in _FLAG_FIELD_INDICES:
            row.append(_flag(value))
        elif index in _TEXT_FIELD_INDICES:
            row.append(_text(value))
        else:
            row.append(_number(value))
    return tuple(row)


def _ensure_directory(directory):
    try:
        os.mkdir(directory)
    except OSError:
        # 目录已存在时继续；其余问题会在 open() 时直接报错。
        pass


def _session_token():
    if hasattr(time, "ticks_ms"):
        return int(time.ticks_ms())
    return int(time.time() * 1000)


def _number(value):
    if value is None:
        return ""
    try:
        return "{:.6f}".format(float(value))
    except (TypeError, ValueError):
        return ""


def _integer(value):
    if value is None:
        return ""
    try:
        return str(int(value))
    except (TypeError, ValueError):
        return ""


def _flag(value):
    return "1" if value else "0"


def _text(value):
    if value is None:
        return ""
    return str(value).replace(",", ";").replace("\r", " ").replace("\n", " ")
