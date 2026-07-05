import time, gc
from media.sensor import *
from media.display import *
from media.media import *
import cv2
from ulab import numpy as np

sensor = Sensor(width=640, height=480, fps=90)
sensor.reset()
sensor.set_framesize(width=640, height=480)
sensor.set_pixformat(Sensor.RGB888)    # OpenCV 需 RGB888
sensor.run()

# Display.init(Display.ST7701, width=640, height=480, to_ide=True)



def _is_circular_green_target(contour, min_area=20, min_circularity=0.65, max_aspect_ratio_diff=0.35,):
    """筛选面积足够、外接框接近正方形、圆度足够的绿色轮廓"""
    area = cv2.contourArea(contour)
    if area < min_area:
        return False

    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return False

    circularity = 4 * np.pi * area / (perimeter * perimeter)
    if circularity < min_circularity:
        return False

    x, y, w, h = cv2.boundingRect(contour)
    if w == 0 or h == 0:
        return False

    aspect_ratio_diff = abs(w - h) / max(w, h)
    if aspect_ratio_diff > max_aspect_ratio_diff:
        return False

    return True

try:
    clock = time.clock()
    while True:
        clock.tick()
        img = sensor.snapshot()            # image.Image
        img_np = img.to_numpy_ref()        # 共享内存转 ndarray

        hsv_image = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
        # 创建掩膜，提取绿色区域
        lo = np.array([45,80,120], dtype=np.uint8)
        hi = np.array([75,255,255], dtype=np.uint8)
        mask = cv2.inRange(
            hsv_image,
            lo,hi
        )
        mask = cv2.medianBlur(mask, 5)
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        circular_contours = [
            contour for contour in contours
            if _is_circular_green_target(contour)
        ]

        if circular_contours:
            # 获取最大圆形绿色轮廓
            max_contour = max(circular_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            # 使用图像矩计算质心（比边界框中心更精确）
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print("x,y",cx,cy)
            else:
                # 面积为零时回退到边界框中心
                cx, cy = x + w // 2, y + h // 2

            # ---- 绘制检测结果到图像 ----
            cv2.rectangle(img_np, (x, y), (x + w, y + h), (0, 255, 0), 2)          # 绿色边界框
            cv2.line(img_np, (cx - 10, cy), (cx + 10, cy), (255, 0, 0), 2)         # 红色十字横线
            cv2.line(img_np, (cx, cy - 10), (cx, cy + 10), (255, 0, 0), 2)         # 红色十字竖线
            cv2.circle(img_np, (cx, cy), 4, (0, 0, 255), -1)                        # 红色实心质心点
            cv2.putText(img_np, f"({cx},{cy})", (cx + 12, cy - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)          # 白色坐标
        
        print("FPS:", clock.fps())
        gc.collect()
        # Display.show_image(img)  # 直接显示 (OpenCV 已修改共享内存)

finally:
    sensor.stop()
    # Display.deinit()