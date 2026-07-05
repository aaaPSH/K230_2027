import time, os, sys
import math

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

# 颜色阈值设置（绿色灯泡LAB阈值）
WHITE_THRESHOLD = ((60, 255, -60, -20, -20, 40))

image_center_x = 320
image_center_y = 240

cx = 0
cy = 0

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
    # 摄像头与显示初始化
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_vflip(False)
    sensor.set_framesize(Sensor.VGA, chn=CAM_CHN_ID_1)  # 640*480
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    sensor.auto_exposure(False)

    Display.init(Display.VIRT, width = 640, height = 480, to_ide=True)
    MediaManager.init()
    sensor.run()

    sensor.exposure(2000)
    sensor.again(0.0)

    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()
        img = sensor.snapshot(chn=CAM_CHN_ID_1)

        # 绿色色块检测
        white_blobs = img.find_blobs(
            [WHITE_THRESHOLD],
            pixels_threshold=150,
            area_threshold=150,
            merge=True
        )
        largest_white = None
        for blob in white_blobs:
            if is_valid_blob(blob):
                if largest_white is None or blob.pixels() > largest_white.pixels():
                    largest_white = blob

        if largest_white:
            cx = largest_white.cx()
            cy = largest_white.cy()
            tw = largest_white.w()
            th = largest_white.h()
            x = cx - tw // 2
            y = cy - th // 2
            img.draw_rectangle((x, y, tw, th), color=(0, 255, 0), thickness=2)
            img.draw_cross(cx, cy, color=(0, 255, 0), size=10)
        else:
            cx = 0
            cy = 0

        img.draw_string_advanced(10, 10, 30, f"FPS: {clock.fps():.1f}")
        Display.show_image(img)
        print(f"x={cx:.1f}, y={cy:.1f}, FPS={clock.fps():.1f}")


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
