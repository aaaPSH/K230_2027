"""K230 CanMV 入口 - dart 引导程序。"""
import gc
import os
import time
import sys

PROJECT_DIR = "/sdcard/dart_py"

if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from media.sensor import *
from media.display import *
from media.media import *

from attitude import AttitudeWorker, ticks_diff, ticks_us
from command_output import make_lower_computer_interface
from imu_uart import make_imu_interface
from config.camera import CAMERA_CONFIG
from config.comm import COMM_CONFIG
from config.detector import DETECTOR_CONFIG
from config.display import DISPLAY_CONFIG
from config.guidance import GUIDANCE_CONFIG
from detector import Detector
from flight_log import FlightLogger
from guidance import build_overload_command, make_guidance_from_config
from keyframe import KeyframeSaver
from visualization import draw_visualization


# GC 不需要每帧执行；45 帧约对应 90 FPS 下每 0.5 秒执行一次。
GC_COLLECT_INTERVAL_FRAMES = 30


def get_millis():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def millis_diff(now_ms, last_ms):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now_ms, last_ms)
    diff = now_ms - last_ms
    return diff if diff >= 0 else 0


def sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


class RuntimeDiagnostics:
    """默认关闭的低频运行性能汇总。"""

    def __init__(self, config=None):
        config = config or {}
        self.enabled = bool(config.get("enabled", False))
        self.period_ms = max(100, int(config.get("period_ms", 1000)))
        self._last_report_ms = None
        self._frame_count = 0
        self._frame_total_us = 0
        self._detector_total_us = 0
        self._guidance_total_us = 0
        self._gc_total_us = 0
        self._stage_totals_us = {}
        self._latest_state = None

    def start_frame(self):
        return ticks_us()

    def record_frame(self, frame_start_us, gc_us):
        if not self.enabled:
            return
        self._frame_count += 1
        self._frame_total_us += ticks_diff(ticks_us(), frame_start_us)
        self._gc_total_us += gc_us

    def record_detector(self, elapsed_us):
        if self.enabled:
            self._detector_total_us += elapsed_us

    def record_guidance(self, elapsed_us):
        if self.enabled:
            self._guidance_total_us += elapsed_us

    def start_stage(self):
        return ticks_us()

    def record_stage(self, name, stage_start_us):
        """记录一条串行处理链路的耗时，单位为微秒。"""
        if not self.enabled or stage_start_us is None:
            return
        elapsed_us = ticks_diff(ticks_us(), stage_start_us)
        self._stage_totals_us[name] = self._stage_totals_us.get(name, 0) + elapsed_us

    def record_state(
        self,
        frame_index,
        image_timestamp_us,
        fps,
        detection,
        guidance_result,
        command,
    ):
        """保存最近一帧状态，供低频输出与 CSV 使用相同字段。"""
        if not self.enabled:
            return
        self._latest_state = (
            frame_index,
            image_timestamp_us,
            fps,
            detection,
            guidance_result,
            command,
        )

    def report_if_due(
        self,
        now_ms,
        attitude_worker,
        imu_interface,
        fps=None,
        camera_fps=None,
        display_fps=None,
        display_enabled=True,
    ):
        if not self.enabled:
            return
        if (
            self._last_report_ms is not None
            and millis_diff(now_ms, self._last_report_ms) < self.period_ms
        ):
            return
        if self._last_report_ms is None:
            self._last_report_ms = now_ms
            return

        count = max(1, self._frame_count)
        pending = getattr(imu_interface, "pending_packet_count", 0)
        invalid = getattr(imu_interface, "invalid_packet_count", 0)
        error = getattr(attitude_worker, "last_error", None)
        if fps is None:
            fps = self._frame_count * 1000.0 / max(1, millis_diff(now_ms, self._last_report_ms))
        frame_ms = self._frame_total_us / count / 1000.0
        processing_cap_fps = 1000.0 / max(frame_ms, 0.001)
        configured_caps = []
        if camera_fps is not None and camera_fps > 0:
            configured_caps.append(("camera", float(camera_fps)))
        if display_enabled and display_fps is not None and display_fps > 0:
            configured_caps.append(("display", float(display_fps)))
        configured_cap_fps = (
            min([value for _, value in configured_caps])
            if configured_caps
            else processing_cap_fps
        )
        effective_cap_fps = min(processing_cap_fps, configured_cap_fps)
        stage_averages = {
            name: total_us / count / 1000.0
            for name, total_us in self._stage_totals_us.items()
        }
        accounted_ms = sum(stage_averages.values())
        stage_averages["overhead"] = max(0.0, frame_ms - accounted_ms)
        if stage_averages:
            bottleneck_name = max(stage_averages, key=stage_averages.get)
            bottleneck_ms = stage_averages[bottleneck_name]
        else:
            bottleneck_name = "none"
            bottleneck_ms = 0.0
        if processing_cap_fps <= configured_cap_fps:
            cap_source = "processing"
        elif configured_caps:
            cap_source = min(configured_caps, key=lambda item: item[1])[0]
        else:
            cap_source = "processing"
        print(
            "diag fps={:.1f} frame_ms={:.2f} detector_ms={:.2f} "
            "guidance_ms={:.2f} gc_ms={:.2f} imu_pending={} "
            "imu_invalid={} attitude_error={} cap_fps={:.1f} "
            "cap_source={} bottleneck={} bottleneck_ms={:.2f}".format(
                fps,
                frame_ms,
                self._detector_total_us / count / 1000.0,
                self._guidance_total_us / count / 1000.0,
                self._gc_total_us / count / 1000.0,
                pending,
                invalid,
                error or "none",
                effective_cap_fps,
                cap_source,
                bottleneck_name,
                bottleneck_ms,
            )
        )
        if stage_averages:
            stage_text = " ".join(
                "{}={:.2f}ms".format(name, stage_averages[name])
                for name in sorted(stage_averages)
            )
            print(
                "diag_stages {} configured_cap_fps={:.1f} "
                "processing_cap_fps={:.1f}".format(
                    stage_text,
                    configured_cap_fps,
                    processing_cap_fps,
                )
            )
        if self._latest_state is not None:
            row = FlightLogger.build_row(*self._latest_state)
            telemetry = " ".join(
                "{}={}".format(FlightLogger.FIELDS[index], row[index])
                for index in range(len(FlightLogger.FIELDS))
            )
            print("telemetry " + telemetry)
        self._last_report_ms = now_ms
        self._frame_count = 0
        self._frame_total_us = 0
        self._detector_total_us = 0
        self._guidance_total_us = 0
        self._gc_total_us = 0
        self._stage_totals_us = {}


def create_sensor(camera_config):
    sensor = Sensor(
        id=camera_config["sensor_id"],
        width=camera_config["width"],
        height=camera_config["height"],
        fps=camera_config.get("fps", 30),
    )
    sensor.reset()
    sensor.set_hmirror(camera_config.get("hmirror", False))
    sensor.set_vflip(camera_config.get("vflip", False))
    sensor.set_framesize(
        width=camera_config["width"],
        height=camera_config["height"],
        chn=camera_config["channel_id"],
    )
    sensor.set_pixformat(Sensor.RGB888, chn=camera_config["channel_id"])
    sensor.auto_exposure(camera_config.get("auto_exposure", False))
    return sensor


def apply_runtime_camera_settings(sensor, camera_config):
    exposure_us = camera_config.get("exposure_us")
    if exposure_us is not None:
        sensor.exposure(exposure_us)
    analog_gain = camera_config.get("analog_gain")
    if analog_gain is not None:
        sensor.again(analog_gain)


def init_display(display_config):
    if not display_config.get("enabled", True):
        return
    display_type = getattr(Display, display_config.get("type", "VIRT"))
    Display.init(
        display_type,
        width=display_config["width"],
        height=display_config["height"],
        fps=display_config.get("fps", 0),
        to_ide=display_config.get("to_ide", True),
        quality=display_config.get("quality", 90),
    )


def step_guidance(
    detector,
    guidance,
    image,
    dt,
    image_timestamp_us,
    attitude_worker,
    lower_interface,
    fps,
    diagnostics=None,
):
    if attitude_worker.last_error is not None:
        raise RuntimeError(
            "attitude worker error ({}): {}".format(
                attitude_worker.last_error_type or "Exception",
                attitude_worker.last_error,
            )
        )
    detector_start_us = ticks_us() if diagnostics is not None else None
    detection = detector.detect(image)
    if diagnostics is not None:
        diagnostics.record_detector(ticks_diff(ticks_us(), detector_start_us))
    attitude_state = attitude_worker.state_at(image_timestamp_us)
    roll_rad = None
    gyro_b = None
    max_source_age_us = int(
        COMM_CONFIG.get("attitude", {}).get("max_source_age_us", 150000)
    )
    attitude_valid = bool(
        attitude_state is not None
        and attitude_state.get("timestamp_match", False)
        and attitude_state.get("initialized", False)
        and attitude_state.get("source_age_us", max_source_age_us + 1)
        <= max_source_age_us
    )
    if attitude_valid:
        roll_rad = attitude_state.get("roll_rad")
        gyro_b = attitude_state.get("gyro_b")

    guidance_start_us = ticks_us() if diagnostics is not None else None
    if not attitude_valid:
        # 缺少可靠姿态时无法完成滚转分配和惯性 LOS rate 补偿，禁止非零输出。
        guidance.reset()
        guidance_result = guidance.lost_result()
    elif detection.get("detected", False):
        guidance_result = guidance.update(
            detection["x"],
            detection["y"],
            dt=dt,
            roll_rad=roll_rad,
            gyro_b=gyro_b,
        )
    else:
        guidance_result = guidance.predict(
            dt,
            roll_rad=roll_rad,
            gyro_b=gyro_b,
        )
    if diagnostics is not None:
        diagnostics.record_guidance(ticks_diff(ticks_us(), guidance_start_us))

    # 将时间对齐诊断信息附加到引导输出中。
    # 在调优 UART 波特率、传感器速率和匹配窗口时很有用。
    if attitude_state is not None:
        guidance_result["sensor_timestamp_us"] = attitude_state.get("timestamp_us")
        guidance_result["sensor_timestamp_delta_us"] = attitude_state.get(
            "timestamp_delta_us"
        )
        guidance_result["sensor_timestamp_error_us"] = attitude_state.get(
            "timestamp_error_us"
        )
        guidance_result["sensor_timestamp_match"] = attitude_state.get(
            "timestamp_match", False
        )
        guidance_result["sensor_source_timestamp_us"] = attitude_state.get(
            "source_timestamp_us"
        )
        guidance_result["sensor_source_age_us"] = attitude_state.get(
            "source_age_us"
        )
        guidance_result["sensor_initialized"] = attitude_state.get(
            "initialized", False
        )
        guidance_result["sensor_roll_rad"] = attitude_state.get("roll_rad")
        guidance_result["sensor_gyro_b"] = attitude_state.get("gyro_b")
        guidance_result["sensor_gyro_held"] = attitude_state.get("gyro_held", False)
    guidance_result["sensor_valid"] = attitude_valid

    command = build_overload_command(
        detection,
        guidance_result,
        fps=fps,
        dt=dt,
        config=GUIDANCE_CONFIG,
    )
    lower_interface.send_overload(command)
    return detection, guidance_result, command


def run():
    sensor = None
    sensor_running = False
    display_inited = False
    media_inited = False
    attitude_worker = None
    imu_interface = None
    lower_interface = None
    flight_logger = None
    keyframe_saver = None
    diagnostics_config = COMM_CONFIG.get("diagnostics", {}).copy()
    diagnostics_config["enabled"] = bool(COMM_CONFIG.get("debug_print", False))
    diagnostics = RuntimeDiagnostics(diagnostics_config)
    try:
        flight_logger = FlightLogger(COMM_CONFIG.get("flight_log", {}))
        keyframe_saver = KeyframeSaver(COMM_CONFIG.get("keyframe", {}))
        detector = Detector(**DETECTOR_CONFIG)
        guidance = make_guidance_from_config(CAMERA_CONFIG, GUIDANCE_CONFIG)
        imu_interface = make_imu_interface(COMM_CONFIG)
        attitude_worker = AttitudeWorker(
            imu_interface,
            COMM_CONFIG.get("attitude", {}),
        )
        attitude_worker.start()
        # IMU 接收和制导指令下发复用同一个 UART1 实例，避免重复初始化串口。
        lower_interface = make_lower_computer_interface(
            COMM_CONFIG,
            uart=getattr(imu_interface, "uart", None),
        )

        sensor = create_sensor(CAMERA_CONFIG)
        init_display(DISPLAY_CONFIG)
        display_inited = DISPLAY_CONFIG.get("enabled", True)
        MediaManager.init()
        media_inited = True
        sensor.run()
        sensor_running = True
        apply_runtime_camera_settings(sensor, CAMERA_CONFIG)

        clock = time.clock()
        # 制导滤波器使用微秒时钟，避免毫秒量化在高帧率下放大角速度噪声。
        last_frame_us = ticks_us()
        channel_id = CAMERA_CONFIG["channel_id"]
        max_dt_sec = GUIDANCE_CONFIG.get("max_dt_sec", 0.2)
        fps = 0.0
        frame_index = 0

        while True:
            frame_index += 1
            frame_start_us = diagnostics.start_frame()
            os.exitpoint()
            clock.tick()

            now_us = ticks_us()
            dt = ticks_diff(now_us, last_frame_us) / 1000000.0
            last_frame_us = now_us
            if dt <= 0.0 or dt > max_dt_sec:
                guidance.reset()
                dt = 0.0

            # 基于 dt 的 EMA 平滑 FPS，每帧更新，反映当前真实帧率。
            if dt > 0.0:
                instant_fps = 1.0 / dt
                if fps == 0.0:
                    fps = instant_fps
                else:
                    fps += 0.1 * (instant_fps - fps)

            stage_start_us = diagnostics.start_stage()
            image = sensor.snapshot(chn=channel_id)
            diagnostics.record_stage("capture", stage_start_us)
            # 此时间戳在图像帧返回时立即记录。
            # 如果相机驱动后续暴露了曝光时间戳，则改用该值
            # （使用相同的 ticks_us() 时钟基准）。
            image_timestamp_us = ticks_us()
            stage_start_us = diagnostics.start_stage()
            detection, guidance_result, command = step_guidance(
                detector,
                guidance,
                image,
                dt,
                image_timestamp_us,
                attitude_worker,
                lower_interface,
                fps,
                diagnostics,
            )
            diagnostics.record_stage("guidance_loop", stage_start_us)

            stage_start_us = diagnostics.start_stage()
            draw_visualization(
                image,
                detection,
                command,
                guidance_result=guidance_result,
                config=DISPLAY_CONFIG,
            )
            diagnostics.record_stage("visualization", stage_start_us)

            stage_start_us = diagnostics.start_stage()
            keyframe_saver.save_if_needed(
                image,
                frame_index,
                image_timestamp_us,
            )
            diagnostics.record_stage("keyframe", stage_start_us)

            stage_start_us = diagnostics.start_stage()
            if DISPLAY_CONFIG.get("enabled", True):
                Display.show_image(image)
            diagnostics.record_stage("display", stage_start_us)

            gc_elapsed_us = 0
            gc_stage_start_us = diagnostics.start_stage()
            if frame_index % GC_COLLECT_INTERVAL_FRAMES == 0:
                gc_start_us = ticks_us()
                gc.collect()
                gc_elapsed_us = ticks_diff(ticks_us(), gc_start_us)
            diagnostics.record_stage("gc", gc_stage_start_us)
            stage_start_us = diagnostics.start_stage()
            flight_logger.record(
                frame_index,
                image_timestamp_us,
                fps,
                detection,
                guidance_result,
                command,
            )
            diagnostics.record_state(
                frame_index,
                image_timestamp_us,
                fps,
                detection,
                guidance_result,
                command,
            )
            diagnostics.record_stage("logging", stage_start_us)
            diagnostics.record_frame(
                frame_start_us,
                gc_elapsed_us,
            )
            diagnostics.report_if_due(
                get_millis(),
                attitude_worker,
                imu_interface,
                fps,
                camera_fps=CAMERA_CONFIG.get("fps"),
                display_fps=DISPLAY_CONFIG.get("fps"),
                display_enabled=DISPLAY_CONFIG.get("enabled", True),
            )
    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except Exception as exc:
        print("guidance loop error:", exc)
        # 排错阶段保留异常栈并中断程序，禁止静默继续控制。
        raise
    finally:
        if keyframe_saver is not None:
            keyframe_saver.close()
        if flight_logger is not None:
            flight_logger.close()
        if attitude_worker is not None:
            attitude_worker.stop()
            attitude_worker.join()
        if imu_interface is not None and hasattr(imu_interface, "deinit"):
            imu_interface.deinit()
        if lower_interface is not None and hasattr(lower_interface, "deinit"):
            lower_interface.deinit()
        if sensor is not None and sensor_running:
            sensor.stop()
        if display_inited:
            Display.deinit()
        if hasattr(os, "exitpoint"):
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        sleep_ms(100)
        if media_inited:
            MediaManager.deinit()


if __name__ == "__main__":
    run()
