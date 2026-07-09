"""OpenCV overlay drawing for CanMV IDE visualization."""
import cv2


GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)


def draw_visualization(
    image,
    detection,
    command,
    guidance_result=None,
    config=None,
):
    image_np = _as_ndarray(image)
    if image_np is None:
        return image

    height, width = _image_shape(image_np)
    aim_x = width // 2
    aim_y = height // 2

    cv2.circle(image_np, (aim_x, aim_y), 4, BLUE, -1)
    cv2.line(image_np, (aim_x - 8, aim_y), (aim_x + 8, aim_y), BLUE, 1)
    cv2.line(image_np, (aim_x, aim_y - 8), (aim_x, aim_y + 8), BLUE, 1)

    if detection and detection.get("detected", False):
        target_x = int(detection.get("x", -1))
        target_y = int(detection.get("y", -1))
        bbox = detection.get("bbox")
        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(image_np, (int(x), int(y)), (int(x + w), int(y + h)), RED, 2)
        cv2.line(image_np, (target_x - 8, target_y), (target_x + 8, target_y), GREEN, 2)
        cv2.line(image_np, (target_x, target_y - 8), (target_x, target_y + 8), GREEN, 2)
        cv2.circle(image_np, (target_x, target_y), 3, GREEN, -1)
        cv2.line(image_np, (aim_x, aim_y), (target_x, target_y), YELLOW, 1)

    if config is None or config.get("show_fps", True):
        _draw_text(image_np, 10, 22, "FPS:{:.1f}".format(command.get("fps", 0.0)))

    if config is None or config.get("show_guidance", True):
        _draw_text(
            image_np,
            10,
            46,
            "P:{:.2f}g Y:{:.2f}g".format(
                command.get("pitch_overload_g", 0.0),
                command.get("yaw_overload_g", 0.0),
            ),
        )
        _draw_text(
            image_np,
            10,
            70,
            "T:{:.0f},{:.0f} {}".format(
                command.get("target_x", -1.0),
                command.get("target_y", -1.0),
                "LOCK" if command.get("detected", False) else "LOST",
            ),
        )
        if guidance_result is not None:
            _draw_text(
                image_np,
                10,
                94,
                "LOS:{:.3f},{:.3f}".format(
                    guidance_result.get("pitch_los_rate_rad_s", 0.0),
                    guidance_result.get("yaw_los_rate_rad_s", 0.0),
                ),
            )
    return image


def _draw_text(image_np, x, y, text):
    cv2.putText(
        image_np,
        text,
        (int(x), int(y)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        WHITE,
        1,
    )


def _as_ndarray(image):
    if hasattr(image, "to_numpy_ref"):
        return image.to_numpy_ref()
    return image


def _image_shape(image_np):
    shape = image_np.shape
    return int(shape[0]), int(shape[1])

