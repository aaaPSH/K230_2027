"""进行基地目标靶绿灯的识别和检测"""
import math
import cv2

class Detector:

    def __init__(
        self,
        thresholds_low=(45, 80, 120),
        thresholds_high=(75, 255, 255),
        min_area=20,
        min_circularity=0.65,
        max_aspect_ratio_diff=0.35,
        fallback_green_min=120,
    ):
        self.thresholds_low = thresholds_low
        self.thresholds_high = thresholds_high
        self.min_area = min_area
        self.min_circularity = min_circularity
        self.max_aspect_ratio_diff = max_aspect_ratio_diff
        self.fallback_green_min = fallback_green_min

    def _is_circular_green_target(self, contour):
        """筛选面积足够、外接框接近正方形、圆度足够的绿色轮廓"""
        if cv2 is None:
            return False

        area = cv2.contourArea(contour)
        if area < self.min_area:
            return False

        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False

        circularity = 4 * math.pi * area / (perimeter * perimeter)
        if circularity < self.min_circularity:
            return False

        x, y, w, h = cv2.boundingRect(contour)
        if w == 0 or h == 0:
            return False

        aspect_ratio_diff = abs(w - h) / max(w, h)
        if aspect_ratio_diff > self.max_aspect_ratio_diff:
            return False

        return True

    def detect(self, image):
        """检测图像中的基地目标靶绿灯"""
        if cv2 is None:
            return self._detect_without_cv2(image)

        # 将图像转换为HSV颜色空间
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 创建掩膜，提取绿色区域
        mask = cv2.inRange(
            hsv_image,
            cv2.cvtColor(self.thresholds_low, cv2.COLOR_BGR2HSV)[0],
            cv2.cvtColor(self.thresholds_high, cv2.COLOR_BGR2HSV)[0],
        )
        mask = cv2.medianBlur(mask, 5)
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        circular_contours = [
            contour for contour in contours
            if self._is_circular_green_target(contour)
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
            else:
                # 面积为零时回退到边界框中心
                cx, cy = x + w // 2, y + h // 2
            return {
                "detected": True,
                "position": (cx, cy),
                "bbox": (x, y, w, h),
                "area": cv2.contourArea(max_contour),
            }
        else:
            return {"detected": False, "position": (-1, -1), "bbox": None, "area": 0}