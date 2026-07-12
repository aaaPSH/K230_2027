"""K230 CanMV entrypoint for dart guidance."""
import gc
import os
import time
import sys

# PROJECT_PATH = "/sdcard/dart_py"

# if PROJECT_PATH not in sys.path:
#     sys.path.append(PROJECT_PATH)

from media.sensor import *
from media.display import *
from media.media import *

from attitude import AttitudeWorker, ticks_us
from comm import make_gpio_interface, make_imu_interface, make_lower_computer_interface
from config.camera import CAMERA_CONFIG
from config.comm import COMM_CONFIG
from config.detector import DETECTOR_CONFIG
from config.display import DISPLAY_CONFIG
from config.guidance import GUIDANCE_CONFIG
from detector import Detector
from guidance import build_overload_command, make_guidance_from_config
from visualization import draw_visualization


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
    clock,
):
    detection = detector.detect(image)
    attitude_state = attitude_worker.state_at(image_timestamp_us)
    roll_rad = None
    gyro_b = None
    if attitude_state is not None and attitude_state.get("timestamp_match", False):
        roll_rad = attitude_state.get("roll_rad")
        gyro_b = attitude_state.get("gyro_b")

    if detection.get("detected", False):
        guidance_result = guidance.update(
            detection["x"],
            detection["y"],
            dt=dt,
            roll_rad=roll_rad,
            gyro_b=gyro_b,
        )
    else:
        guidance.predict_kalman(dt)
        guidance_result = guidance.lost_result()

    # Keep alignment diagnostics with the guidance output.  They are useful
    # when tuning UART baud rate, sensor rate, and the matching window.
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

    command = build_overload_command(
        detection,
        guidance_result,
        fps=clock.fps(),
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
    try:
        detector = Detector(**DETECTOR_CONFIG)
        guidance = make_guidance_from_config(CAMERA_CONFIG, GUIDANCE_CONFIG)
        imu_interface = make_imu_interface(COMM_CONFIG)
        gpio_interface = make_gpio_interface(COMM_CONFIG)
        attitude_worker = AttitudeWorker(
            imu_interface,
            gpio_interface,
            COMM_CONFIG.get("attitude", {}),
        )
        attitude_worker.start()
        lower_interface = make_lower_computer_interface(COMM_CONFIG)

        sensor = create_sensor(CAMERA_CONFIG)
        init_display(DISPLAY_CONFIG)
        display_inited = DISPLAY_CONFIG.get("enabled", True)
        MediaManager.init()
        media_inited = True
        sensor.run()
        sensor_running = True
        apply_runtime_camera_settings(sensor, CAMERA_CONFIG)

        clock = time.clock()
        last_frame_ms = get_millis()
        channel_id = CAMERA_CONFIG["channel_id"]
        max_dt_sec = GUIDANCE_CONFIG.get("max_dt_sec", 0.2)

        while True:
            os.exitpoint()
            clock.tick()

            now_ms = get_millis()
            dt = millis_diff(now_ms, last_frame_ms) / 1000.0
            last_frame_ms = now_ms
            if dt <= 0.0 or dt > max_dt_sec:
                guidance.reset()
                dt = 0.0

            image = sensor.snapshot(chn=channel_id)
            # This timestamp is recorded as soon as the frame is returned.
            # If the camera driver later exposes an exposure timestamp, pass
            # that value here instead (using the same ticks_us() clock base).
            image_timestamp_us = ticks_us()
            detection, guidance_result, command = step_guidance(
                detector,
                guidance,
                image,
                dt,
                image_timestamp_us,
                attitude_worker,
                lower_interface,
                clock,
            )

            draw_visualization(
                image,
                detection,
                command,
                guidance_result=guidance_result,
                config=DISPLAY_CONFIG,
            )
            if DISPLAY_CONFIG.get("enabled", True):
                Display.show_image(image)

            gc.collect()
    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except BaseException as exc:
        print("guidance loop error:", exc)
    finally:
        if attitude_worker is not None:
            attitude_worker.stop()
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
