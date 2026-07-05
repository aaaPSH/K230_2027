import time, os, sys

from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *
from guidance import ProportionalGuidance

# ---------------------- 新增：导入字体相关模块（关键补充） ----------------------
# 若IDE有专门的字体模块，需先导入（多数情况会包含在display或media中）
try:
    from media.font import Font, TEXT_ALIGN_LEFT  # 假设字体类和对齐常量在此路径
except ImportError:
    # 若导入失败，用通用默认值替代（避免报错）
    class Font:
        DEFAULT = None  # 默认字体对象（IDE会自动识别系统默认字体）
    TEXT_ALIGN_LEFT = 0  # 左对齐（多数库用0表示左对齐）

sensor_id = 2
sensor = None

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
NO_TARGET = -1

# 颜色阈值设置（绿色灯泡LAB阈值）
GREEN_THRESHOLD = ((60, 255, -60, -20, -20, 40))

image_center_x = IMAGE_WIDTH // 2
image_center_y = IMAGE_HEIGHT // 2

# 制导律参数，需结合实际飞行速度、舵效和相机标定继续整定。
GUIDANCE_NAVIGATION_RATIO = 3.0
GUIDANCE_CLOSING_VELOCITY = 15.0
GUIDANCE_POSITION_TO_RATE_GAIN = 2.0
GUIDANCE_MAX_OVERLOAD_G = 6.0

cx = 0
cy = 0


def get_millis():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def millis_diff(now_ms, last_ms):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now_ms, last_ms)
    diff = now_ms - last_ms
    return diff if diff >= 0 else 0


# 陀螺仪数据传输接口：后续在这里接 UART/SPI/飞控数据。
def read_gyro_data():
    """
    返回 (roll_rad, gyro_b)。

    roll_rad：当前滚转角，单位 rad；无数据时返回 None。
    gyro_b：镖体系角速度 [roll_rate, pitch_rate, yaw_rate]，单位 rad/s；
            无数据时返回 None。
    """
    return None, None


def safe_guidance_update(guidance, target_x, target_y, dt, roll_rad, gyro_b):
    try:
        return guidance.update(
            target_x,
            target_y,
            dt=dt,
            roll_rad=roll_rad,
            gyro_b=gyro_b,
        )
    except Exception as e:
        print(f"guidance update error: {e}")
        guidance.reset()
        return {
            "detected": False,
            "yaw_overload_g": 0.0,
            "pitch_overload_g": 0.0,
        }


# 色块有效性验证函数
def is_valid_blob(blob, min_area=300):
    if blob.pixels() < min_area:
        return False
    rect_area = blob.w() * blob.h()
    if rect_area == 0:
        return False
    density = blob.pixels() / rect_area
    if density < 0.2:
        return False
    return True

try:
    guidance = ProportionalGuidance(
        image_width=IMAGE_WIDTH,
        image_height=IMAGE_HEIGHT,
        navigation_ratio=GUIDANCE_NAVIGATION_RATIO,
        closing_velocity=GUIDANCE_CLOSING_VELOCITY,
        position_to_rate_gain=GUIDANCE_POSITION_TO_RATE_GAIN,
        max_overload_g=GUIDANCE_MAX_OVERLOAD_G,
        roll_compensation=True,
    )

    # 摄像头与显示初始化
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_vflip(False)
    sensor.set_framesize(Sensor.VGA, chn=CAM_CHN_ID_1)  # 640*480
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    sensor.auto_exposure(False)

    Display.init(Display.VIRT, width=IMAGE_WIDTH, height=IMAGE_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()

    sensor.exposure(2000)
    sensor.again(0.0)

    clock = time.clock()
    last_frame_ms = get_millis()

    while True:
        os.exitpoint()
        clock.tick()
        now_ms = get_millis()
        dt_seconds = millis_diff(now_ms, last_frame_ms) / 1000.0
        dt = dt_seconds if dt_seconds > 0.0 else None
        dt_log = dt if dt is not None else 0.0
        last_frame_ms = now_ms
        img = sensor.snapshot(chn=CAM_CHN_ID_1)

        # 绿色色块检测
        green_blobs = img.find_blobs(
            [GREEN_THRESHOLD],
            pixels_threshold=150,
            area_threshold=150,
            merge=True
        )
        largest_green = None
        for blob in green_blobs:
            if is_valid_blob(blob):
                if largest_green is None or blob.pixels() > largest_green.pixels():
                    largest_green = blob

        if largest_green:
            cx = largest_green.cx()
            cy = largest_green.cy()
            tw = largest_green.w()
            th = largest_green.h()
            x = cx - tw // 2
            y = cy - th // 2
            img.draw_rectangle((x, y, tw, th), color=(0, 255, 0), thickness=2)
            img.draw_cross(cx, cy, color=(0, 255, 0), size=10)
        else:
            cx = 0
            cy = 0

        roll_rad, gyro_b = read_gyro_data()
        target_x, target_y = (cx, cy) if largest_green else (NO_TARGET, NO_TARGET)
        guidance_result = safe_guidance_update(
            guidance,
            target_x,
            target_y,
            dt,
            roll_rad,
            gyro_b,
        )

        yaw_overload_g = guidance_result["yaw_overload_g"]
        pitch_overload_g = guidance_result["pitch_overload_g"]

        img.draw_string_advanced(10, 10, 30, f"FPS: {clock.fps():.1f}")
        img.draw_string_advanced(
            10,
            45,
            24,
            f"YawG:{yaw_overload_g:.2f} PitchG:{pitch_overload_g:.2f}",
        )
        Display.show_image(img)
        print(
            f"x={cx:.1f}, y={cy:.1f}, "
            f"yaw_g={yaw_overload_g:.3f}, pitch_g={pitch_overload_g:.3f}, "
            f"dt={dt_log:.3f}, FPS={clock.fps():.1f}"
        )


except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"用户手动异常: {e}")
finally:
    # 资源释放
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
