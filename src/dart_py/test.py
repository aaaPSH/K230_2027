"""OpenCV green-light recognition demo for CanMV K230."""
import gc
import time

from media.sensor import *
from media.display import *
from media.media import *

from config.detector import DETECTOR_CONFIG
from detector import Detector


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def draw_detection(img, blob, cx, cy):
    x, y, w, h = blob
    try:
        img.draw_rectangle(x, y, w, h, color=(0, 255, 0), thickness=2)
        img.draw_string_advanced(
            x,
            max(0, y - 24),
            24,
            "({},{})".format(cx, cy),
            color=(255, 255, 255),
        )
    except Exception:
        pass


sensor = Sensor(width=IMAGE_WIDTH, height=IMAGE_HEIGHT, fps=90)
sensor.reset()
sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
sensor.set_pixformat(Sensor.RGB888)
sensor.run()

# Display.init(Display.ST7701, width=IMAGE_WIDTH, height=IMAGE_HEIGHT, to_ide=True)

try:
    detector = Detector(**DETECTOR_CONFIG)
    clock = time.clock()
    while True:
        clock.tick()
        img = sensor.snapshot()

        detection = detector.detect(img)
        if detection.get("detected", False):
            cx = int(detection.get("x", -1))
            cy = int(detection.get("y", -1))
            bbox = detection.get("bbox")
            print(
                "green x,y,area,pass,g/r,g/b:",
                cx,
                cy,
                detection.get("area", 0.0),
                detection.get("green_ratio", 0.0),
                detection.get("gr_ratio", 0.0),
                detection.get("gb_ratio", 0.0),
            )
            if bbox is not None:
                draw_detection(img, bbox, cx, cy)
        else:
            print("green lost")

        print("FPS:", clock.fps())
        gc.collect()
        # Display.show_image(img)

finally:
    sensor.stop()
    # Display.deinit()
