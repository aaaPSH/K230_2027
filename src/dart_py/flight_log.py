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
        "fps",
        "detected",
        "target_x",
        "target_y",
        "target_area",
        "target_green_ratio",
        "imu_timestamp_us",
        "imu_timestamp_delta_us",
        "imu_timestamp_error_us",
        "imu_timestamp_match",
        "imu_initialized",
        "roll_rad",
        "gyro_x_rad_s",
        "gyro_y_rad_s",
        "gyro_z_rad_s",
        "gyro_held",
        "yaw_los_angle_rad",
        "pitch_los_angle_rad",
        "yaw_los_rate_rad_s",
        "pitch_los_rate_rad_s",
        "yaw_overload_g",
        "pitch_overload_g",
        "command_detected",
        "command_yaw_overload_g",
        "command_pitch_overload_g",

    )

    def __init__(self, config=None):
        config = config or {}
        self.enabled = bool(config.get("enabled", False))
        self.flush_interval_frames = max(
            1,
            int(config.get("flush_interval_frames", 60)),
        )
        self.writer_poll_ms = max(1, int(config.get("writer_poll_ms", 10)))
        self.close_timeout_ms = max(100, int(config.get("close_timeout_ms", 5000)))
        self.path = None
        self._file = None
        self._active_buffer = []
        self._pending_buffers = []
        self._lock = _make_lock()
        self._running = False
        self._writer_active = False
        self._write_error = None
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
        print("flight log:", self.path)

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
        self._raise_write_error()

        row = self.build_row(
            frame_index,
            image_timestamp_us,
            fps,
            detection,
            guidance_result,
            command,
        )
        self._lock.acquire()
        try:
            self._active_buffer.append(",".join(row))
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

        detection = detection or {}
        guidance_result = guidance_result or {}
        command = command or {}
        gyro_b = guidance_result.get("sensor_gyro_b") or (None, None, None)

        row = (
            _integer(frame_index),
            _integer(image_timestamp_us),
            _number(fps),
            _flag(detection.get("detected")),
            _number(detection.get("x")),
            _number(detection.get("y")),
            _number(detection.get("area")),
            _number(detection.get("green_ratio")),
            _integer(guidance_result.get("sensor_timestamp_us")),
            _integer(guidance_result.get("sensor_timestamp_delta_us")),
            _integer(guidance_result.get("sensor_timestamp_error_us")),
            _flag(guidance_result.get("sensor_timestamp_match")),
            _flag(guidance_result.get("sensor_initialized")),
            _number(guidance_result.get("sensor_roll_rad")),
            _number(gyro_b[0]),
            _number(gyro_b[1]),
            _number(gyro_b[2]),
            _flag(guidance_result.get("sensor_gyro_held")),
            _number(guidance_result.get("yaw_los_angle_rad")),
            _number(guidance_result.get("pitch_los_angle_rad")),
            _number(guidance_result.get("yaw_los_rate_rad_s")),
            _number(guidance_result.get("pitch_los_rate_rad_s")),
            _number(guidance_result.get("yaw_overload_g")),
            _number(guidance_result.get("pitch_overload_g")),
            _flag(command.get("detected")),
            _number(command.get("yaw_overload_g")),
            _number(command.get("pitch_overload_g")),
        )
        return row

    def flush(self):
        """将当前缓冲交给后台写线程；此方法不执行文件 I/O。"""
        if not self.enabled:
            return
        self._raise_write_error()
        self._lock.acquire()
        try:
            if self._active_buffer:
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
                raise RuntimeError("flight log writer did not stop before timeout")
            _sleep_ms(self.writer_poll_ms)

        self._file.close()
        self._file = None
        self._raise_write_error()

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
                    self._file.write("\n".join(buffer) + "\n")
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

    def _raise_write_error(self):
        self._lock.acquire()
        try:
            error = self._write_error
        finally:
            self._lock.release()
        if error is not None:
            raise RuntimeError("flight log writer error: {}".format(error))

    def _flush_file(self):
        if hasattr(self._file, "flush"):
            self._file.flush()


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
