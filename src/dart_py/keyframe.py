"""用于赛后复盘的异步关键帧保存。"""
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


def _ticks_add(timestamp, delta_ms):
    if hasattr(time, "ticks_add"):
        return time.ticks_add(timestamp, delta_ms)
    return timestamp + delta_ms


def _ticks_diff(newer, older):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(newer, older)
    return newer - older


class KeyframeSaver:
    """主线程按固定帧间隔复制关键帧，后台线程异步写入 JPEG。"""

    def __init__(self, config=None):
        config = config or {}
        self.enabled = bool(config.get("enabled", False))
        self.directory = config.get("directory", "/sdcard/dart_py/logs/keyframes")
        self.interval_frames = max(1, int(config.get("interval_frames", 60)))
        self.max_pending_frames = max(1, int(config.get("max_pending_frames", 4)))
        self.writer_poll_ms = max(1, int(config.get("writer_poll_ms", 10)))
        self.close_timeout_ms = max(100, int(config.get("close_timeout_ms", 5000)))
        self.debug_print = bool(config.get("debug_print", False))
        self._lock = _make_lock()
        self._pending_frames = []
        self._running = False
        self._writer_active = False
        self._write_error = None
        self._reserved_frame_count = 0
        self.dropped_frame_count = 0
        self._thread = None

        if self.enabled:
            _ensure_directory(self.directory)
            self._running = True
            self._writer_active = True
            self._start_writer()
            self._debug(
                "keyframe enabled directory={} interval_frames={} queue_limit={}".format(
                    self.directory,
                    self.interval_frames,
                    self.max_pending_frames,
                )
            )

    def save_if_needed(self, image, frame_index, image_timestamp_us):
        """按固定帧间隔复制当前带标注图像并交给后台保存。"""
        reservation = self.reserve_if_needed(frame_index, image_timestamp_us)
        return self.save_reserved(image, reservation)

    def reserve_if_needed(self, frame_index, image_timestamp_us):
        """为当前关键帧预留队列位置，返回保存路径或 ``None``。"""
        if not self.enabled:
            return None

        write_error = self._get_write_error()
        if write_error is not None:
            # 关键帧仅用于诊断，写盘失败时自动停用，不能中断制导主循环。
            self.enabled = False
            self._debug("keyframe disabled after writer error={}".format(write_error))
            return None

        if frame_index % self.interval_frames != 0:
            return None

        path = "{}/frame_{:06d}_{}.jpg".format(
            self.directory.rstrip("/"),
            int(frame_index),
            int(image_timestamp_us),
        )
        self._lock.acquire()
        try:
            occupied_count = len(self._pending_frames) + self._reserved_frame_count
            if occupied_count >= self.max_pending_frames:
                # 在复制和绘制图像前确认容量，避免队列满时做无效工作。
                self.dropped_frame_count += 1
                dropped_count = self.dropped_frame_count
            else:
                self._reserved_frame_count += 1
                dropped_count = None
        finally:
            self._lock.release()

        if dropped_count is not None:
            self._debug(
                "keyframe dropped queue_full total_dropped={}".format(
                    dropped_count
                )
            )
            return None
        return path

    def save_reserved(self, image, reservation):
        """复制已预留的图像，并将其提交给后台 JPEG 写线程。"""
        if reservation is None:
            return None

        # 原始图像在下一次 snapshot() 后可能被复用；队列位置已经预留，
        # 因此不会发生复制完成后才因队列已满而丢弃的情况。
        try:
            image_copy = image.copy()
        except Exception:
            self._release_reservation()
            raise

        self._lock.acquire()
        try:
            self._reserved_frame_count -= 1
            if self._write_error is not None:
                pending_count = None
            else:
                self._pending_frames.append((image_copy, reservation))
                pending_count = len(self._pending_frames)
        finally:
            self._lock.release()

        if pending_count is None:
            return None

        self._debug(
            "keyframe queued pending={} path={}".format(
                pending_count,
                reservation,
            )
        )

        return reservation

    def _release_reservation(self):
        self._lock.acquire()
        try:
            if self._reserved_frame_count > 0:
                self._reserved_frame_count -= 1
        finally:
            self._lock.release()

    def close(self):
        """等待后台线程写完排队关键帧。"""
        if not self.enabled:
            return

        self._lock.acquire()
        try:
            self._running = False
        finally:
            self._lock.release()

        deadline_ms = _ticks_add(_ticks_ms(), self.close_timeout_ms)
        while self._writer_is_active():
            if _ticks_diff(_ticks_ms(), deadline_ms) >= 0:
                raise RuntimeError("keyframe writer did not stop before timeout")
            _sleep_ms(self.writer_poll_ms)
        write_error = self._get_write_error()
        if write_error is not None:
            # 此时零过载已由主循环优先发送；这里只保留诊断，不阻断其他清理。
            self._debug("keyframe closed with writer error={}".format(write_error))

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
        raise RuntimeError("a thread implementation is required for keyframe saving")

    def _writer_loop(self):
        try:
            while True:
                frame = self._take_pending_frame()
                if frame is not None:
                    image, path = frame
                    image.save(path)
                    self._debug("keyframe saved path={}".format(path))
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

    def _take_pending_frame(self):
        self._lock.acquire()
        try:
            if not self._pending_frames:
                return None
            return self._pending_frames.pop(0)
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
        error = "{}: {}".format(type(exc).__name__, exc)
        self._lock.acquire()
        try:
            self._write_error = error
            self._running = False
        finally:
            self._lock.release()
        self._debug("keyframe writer error={}".format(error))

    def _get_write_error(self):
        self._lock.acquire()
        try:
            return self._write_error
        finally:
            self._lock.release()

    def _debug(self, message):
        if self.debug_print:
            print(message)


def _ensure_directory(directory):
    """兼容 MicroPython 的逐级目录创建。"""
    current = "/" if directory.startswith("/") else ""
    for part in directory.split("/"):
        if not part:
            continue
        if current == "/":
            current += part
        elif current:
            current += "/" + part
        else:
            current = part
        try:
            os.mkdir(current)
        except OSError:
            # 已存在时继续；无权限等问题会在后台 image.save() 时直接报错。
            pass
