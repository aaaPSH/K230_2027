"""Proportional guidance and overload command generation."""
import math

from kalman import KalmanFilter


G = 9.80665
EPS = 1e-9


class ProportionalGuidance:
    """
    Pixel target -> camera LOS -> body LOS -> roll-stabilized LOS -> overload.

    Coordinate convention:
    - camera: x right, y down, z forward
    - body: x forward, y right, z down
    - gyro_b: [roll_rate, pitch_rate, yaw_rate] in body frame, rad/s
    """

    def __init__(
        self,
        cx=None,
        cy=None,
        fx=None,
        fy=None,
        image_width=640,
        image_height=480,
        fov_x_deg=65.0,
        fov_y_deg=40.0,
        R_bc=None,
        navigation_ratio=3.0,
        yaw_navigation_ratio=None,
        pitch_navigation_ratio=None,
        closing_velocity=15.0,
        yaw_closing_velocity=None,
        pitch_closing_velocity=None,
        position_to_rate_gain=0.0,
        yaw_angle_control_gain=0.0,
        pitch_angle_control_gain=0.0,
        rate_filter_alpha=None,
        use_kalman_filter=True,
        kalman=None,
        yaw_kalman=None,
        pitch_kalman=None,
        kalman_angle_variance=0.05,
        kalman_rate_variance=1.0,
        kalman_process_angle_variance=0.0001,
        kalman_process_rate_variance=0.02,
        kalman_measurement_angle_variance=0.0025,
        kalman_measurement_rate_variance=0.1,
        max_overload_g=0.5,
        yaw_max_overload_g=None,
        pitch_max_overload_g=None,
        roll_compensation=True,
        roll_sign=1.0,
    ):
        self.cx = image_width * 0.5 if cx is None else cx
        self.cy = image_height * 0.5 if cy is None else cy
        self.fx = self._focal_length(image_width, fov_x_deg) if fx is None else fx
        self.fy = self._focal_length(image_height, fov_y_deg) if fy is None else fy

        self.R_bc = _copy_matrix(R_bc) if R_bc is not None else [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]

        self.yaw_navigation_ratio = _axis_or_default(
            yaw_navigation_ratio,
            navigation_ratio,
        )
        self.pitch_navigation_ratio = _axis_or_default(
            pitch_navigation_ratio,
            navigation_ratio,
        )
        self.yaw_closing_velocity = _axis_or_default(
            yaw_closing_velocity,
            closing_velocity,
        )
        self.pitch_closing_velocity = _axis_or_default(
            pitch_closing_velocity,
            closing_velocity,
        )
        self.position_to_rate_gain = position_to_rate_gain
        self.yaw_angle_control_gain = yaw_angle_control_gain
        self.pitch_angle_control_gain = pitch_angle_control_gain
        self.rate_filter_alpha = _clean_rate_filter_alpha(rate_filter_alpha)
        self.use_kalman_filter = use_kalman_filter
        self.yaw_max_overload_g = _axis_or_default(
            yaw_max_overload_g,
            max_overload_g,
        )
        self.pitch_max_overload_g = _axis_or_default(
            pitch_max_overload_g,
            max_overload_g,
        )
        self.roll_compensation = roll_compensation
        self.roll_sign = roll_sign

        kalman_defaults = {
            "angle_variance": kalman_angle_variance,
            "rate_variance": kalman_rate_variance,
            "process_angle_variance": kalman_process_angle_variance,
            "process_rate_variance": kalman_process_rate_variance,
            "measurement_angle_variance": kalman_measurement_angle_variance,
            "measurement_rate_variance": kalman_measurement_rate_variance,
        }
        if kalman is not None:
            kalman_defaults.update(kalman)
        self.yaw_kalman = _merge_axis_config(kalman_defaults, yaw_kalman)
        self.pitch_kalman = _merge_axis_config(kalman_defaults, pitch_kalman)

        self.last_los_b = None
        self.filtered_yaw_dot = 0.0
        self.filtered_pitch_dot = 0.0
        self.has_filtered_rate = False
        self.yaw_filter = None
        self.pitch_filter = None

    def reset(self):
        self.last_los_b = None
        self.filtered_yaw_dot = 0.0
        self.filtered_pitch_dot = 0.0
        self.has_filtered_rate = False
        self.yaw_filter = None
        self.pitch_filter = None

    def predict_kalman(self, dt):
        """Advance filters during a frame without a target measurement."""
        self.last_los_b = None
        self.has_filtered_rate = False
        if not self.use_kalman_filter or dt is None or dt <= 0.0:
            return
        transition_matrix = [
            [1.0, dt],
            [0.0, 1.0],
        ]
        if self.yaw_filter is not None:
            self.yaw_filter.predict(transition_matrix=transition_matrix)
        if self.pitch_filter is not None:
            self.pitch_filter.predict(transition_matrix=transition_matrix)

    def lost_result(self):
        return _lost_guidance_result()

    def update(
        self,
        target_x,
        target_y,
        dt=None,
        roll_rad=None,
        roll_deg=0.0,
        gyro_b=None,
    ):
        if target_x < 0 or target_y < 0:
            self.reset()
            return self.lost_result()

        if dt is None or dt <= 0.0:
            dt = 0.0

        if not self.roll_compensation:
            roll_rad = 0.0
        elif roll_rad is None:
            roll_rad = math.radians(roll_deg)
        roll_rad *= self.roll_sign
        if gyro_b is None:
            gyro_b = [0.0, 0.0, 0.0]

        los_c = self.pixel_to_camera_los(target_x, target_y)
        los_b = _normalize(_mat_vec_mul(self.R_bc, los_c))

        R_roll_comp = self._roll_compensation_matrix(roll_rad)
        los_s = _normalize(_mat_vec_mul(R_roll_comp, los_b))

        yaw_angle, pitch_angle = self._los_angles(los_s)
        los_dot_b = self._los_dot_body(los_b, dt)
        los_dot_true_b = _vec_add(los_dot_b, _cross(gyro_b, los_b))
        los_dot_s = _mat_vec_mul(R_roll_comp, los_dot_true_b)
        yaw_dot, pitch_dot = self._los_angle_rates(los_s, los_dot_s)
        raw_yaw_angle = yaw_angle
        raw_pitch_angle = pitch_angle
        raw_yaw_dot = yaw_dot
        raw_pitch_dot = pitch_dot

        yaw_angle, yaw_dot, pitch_angle, pitch_dot = self._filter_los_states(
            yaw_angle,
            yaw_dot,
            pitch_angle,
            pitch_dot,
            dt,
        )

        yaw_angle_gain = self.yaw_angle_control_gain
        pitch_angle_gain = self.pitch_angle_control_gain
        yaw_rate_control = yaw_dot
        pitch_rate_control = pitch_dot
        yaw_angle_control = yaw_angle_gain * yaw_angle
        pitch_angle_control = pitch_angle_gain * pitch_angle
        yaw_command_rate = yaw_rate_control + yaw_angle_control

        # Match the current C++ guidance.hpp behavior: pitch angle control exists
        # as a parameter but is not added to the pitch command rate.
        pitch_command_rate = pitch_rate_control

        yaw_overload_g = (
            self.yaw_navigation_ratio
            * self.yaw_closing_velocity
            * yaw_command_rate
            / G
        )
        pitch_overload_g = (
            self.pitch_navigation_ratio
            * self.pitch_closing_velocity
            * pitch_command_rate
            / G
        )

        return {
            "detected": True,
            "pixel_error_x": target_x - self.cx,
            "pixel_error_y": target_y - self.cy,
            "los_c": los_c,
            "los_b": los_b,
            "los_s": los_s,
            "yaw_los_angle_rad": yaw_angle,
            "pitch_los_angle_rad": pitch_angle,
            "yaw_los_rate_rad_s": yaw_dot,
            "pitch_los_rate_rad_s": pitch_dot,
            "raw_yaw_los_angle_rad": raw_yaw_angle,
            "raw_pitch_los_angle_rad": raw_pitch_angle,
            "raw_yaw_los_rate_rad_s": raw_yaw_dot,
            "raw_pitch_los_rate_rad_s": raw_pitch_dot,
            "yaw_rate_control_rad_s": yaw_rate_control,
            "pitch_rate_control_rad_s": pitch_rate_control,
            "yaw_angle_control_rad_s": yaw_angle_control,
            "pitch_angle_control_rad_s": pitch_angle_control,
            "yaw_command_rate_rad_s": yaw_command_rate,
            "pitch_command_rate_rad_s": pitch_command_rate,
            "yaw_overload_g": _limit_overload(
                yaw_overload_g,
                self.yaw_max_overload_g,
            ),
            "pitch_overload_g": _limit_overload(
                pitch_overload_g,
                self.pitch_max_overload_g,
            ),
        }

    def pixel_to_camera_los(self, target_x, target_y):
        x_n = (target_x - self.cx) / self.fx
        y_n = (target_y - self.cy) / self.fy
        return _normalize([x_n, y_n, 1.0])

    def _los_dot_body(self, los_b, dt):
        if self.last_los_b is None or dt <= 0.0:
            self.last_los_b = los_b[:]
            return [0.0, 0.0, 0.0]

        los_dot_b = [
            (los_b[index] - self.last_los_b[index]) / dt
            for index in range(3)
        ]
        self.last_los_b = los_b[:]
        return los_dot_b

    def _roll_compensation_matrix(self, roll_rad):
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        return [
            [1.0, 0.0, 0.0],
            [0.0, cos_roll, sin_roll],
            [0.0, -sin_roll, cos_roll],
        ]

    def _los_angles(self, los_s):
        x, y, z = los_s
        rho = math.sqrt(x * x + y * y)
        yaw_angle = math.atan2(y, x)
        pitch_angle = math.atan2(-z, rho)
        return yaw_angle, pitch_angle

    def _los_angle_rates(self, los_s, los_dot_s):
        x, y, z = los_s
        xd, yd, zd = los_dot_s
        rho2 = x * x + y * y
        if rho2 < EPS:
            return 0.0, 0.0

        rho = math.sqrt(rho2)
        yaw_dot = (x * yd - y * xd) / rho2
        pitch_dot = -rho * zd + z * (x * xd + y * yd) / rho
        return yaw_dot, pitch_dot

    def _filter_los_states(self, yaw_angle, yaw_dot, pitch_angle, pitch_dot, dt):
        if self.use_kalman_filter:
            yaw_angle, yaw_dot = self._filter_axis_state(
                self.yaw_filter,
                yaw_angle,
                yaw_dot,
                dt,
                "yaw_filter",
                self.yaw_kalman,
            )
            pitch_angle, pitch_dot = self._filter_axis_state(
                self.pitch_filter,
                pitch_angle,
                pitch_dot,
                dt,
                "pitch_filter",
                self.pitch_kalman,
            )
            return yaw_angle, yaw_dot, pitch_angle, pitch_dot

        yaw_dot, pitch_dot = self._filter_los_rates(yaw_dot, pitch_dot)
        return yaw_angle, yaw_dot, pitch_angle, pitch_dot

    def _filter_axis_state(self, axis_filter, angle, rate, dt, attr_name, params):
        if axis_filter is None:
            axis_filter = self._create_axis_filter(angle, rate, params)
            setattr(self, attr_name, axis_filter)
            return angle, rate

        transition_matrix = [
            [1.0, dt],
            [0.0, 1.0],
        ]
        state = axis_filter.step(
            [angle, rate],
            transition_matrix=transition_matrix,
        )
        return state[0], state[1]

    def _create_axis_filter(self, angle, rate, params):
        return KalmanFilter(
            state=[angle, rate],
            covariance=[
                [params["angle_variance"], 0.0],
                [0.0, params["rate_variance"]],
            ],
            transition_matrix=[
                [1.0, 0.0],
                [0.0, 1.0],
            ],
            measurement_matrix=[
                [1.0, 0.0],
                [0.0, 1.0],
            ],
            process_noise=[
                [params["process_angle_variance"], 0.0],
                [0.0, params["process_rate_variance"]],
            ],
            measurement_noise=[
                [params["measurement_angle_variance"], 0.0],
                [0.0, params["measurement_rate_variance"]],
            ],
        )

    def _filter_los_rates(self, yaw_dot, pitch_dot):
        if self.rate_filter_alpha is None:
            return yaw_dot, pitch_dot
        if not self.has_filtered_rate:
            self.filtered_yaw_dot = yaw_dot
            self.filtered_pitch_dot = pitch_dot
            self.has_filtered_rate = True
            return yaw_dot, pitch_dot

        alpha = self.rate_filter_alpha
        self.filtered_yaw_dot = (
            alpha * yaw_dot + (1.0 - alpha) * self.filtered_yaw_dot
        )
        self.filtered_pitch_dot = (
            alpha * pitch_dot + (1.0 - alpha) * self.filtered_pitch_dot
        )
        return self.filtered_yaw_dot, self.filtered_pitch_dot

    def _focal_length(self, pixels, fov_deg):
        fov_rad = math.radians(fov_deg)
        return (pixels * 0.5) / math.tan(fov_rad * 0.5)


def make_guidance_from_config(camera_config, guidance_config):
    common_kalman = guidance_config.get("kalman")
    return ProportionalGuidance(
        image_width=camera_config["width"],
        image_height=camera_config["height"],
        fov_x_deg=camera_config.get("fov_x_deg", 65.0),
        fov_y_deg=camera_config.get("fov_y_deg", 40.0),
        fx=camera_config.get("fx"),
        fy=camera_config.get("fy"),
        cx=camera_config.get("cx"),
        cy=camera_config.get("cy"),
        navigation_ratio=guidance_config["navigation_ratio"],
        yaw_navigation_ratio=guidance_config.get("yaw_navigation_ratio"),
        pitch_navigation_ratio=guidance_config.get("pitch_navigation_ratio"),
        closing_velocity=guidance_config["closing_velocity"],
        yaw_closing_velocity=guidance_config.get("yaw_closing_velocity"),
        pitch_closing_velocity=guidance_config.get("pitch_closing_velocity"),
        yaw_angle_control_gain=guidance_config["yaw_angle_control_gain"],
        pitch_angle_control_gain=guidance_config["pitch_angle_control_gain"],
        rate_filter_alpha=guidance_config.get("rate_filter_alpha"),
        use_kalman_filter=guidance_config["use_kalman_filter"],
        kalman=common_kalman,
        yaw_kalman=guidance_config.get("yaw_kalman"),
        pitch_kalman=guidance_config.get("pitch_kalman"),
        max_overload_g=guidance_config["max_overload_g"],
        yaw_max_overload_g=guidance_config.get("yaw_max_overload_g"),
        pitch_max_overload_g=guidance_config.get("pitch_max_overload_g"),
        roll_compensation=guidance_config["roll_compensation"],
        roll_sign=guidance_config["roll_sign"],
    )


def build_overload_command(
    detection,
    guidance_result,
    fps=0.0,
    dt=0.0,
    config=None,
):
    """Build the final command payload for the lower computer interface."""
    if config is None:
        config = {}

    detected = bool(detection and detection.get("detected", False))
    guided = bool(guidance_result and guidance_result.get("detected", False))
    if not detected or not guided:
        return {
            "detected": False,
            "pitch_overload_g": 0.0,
            "yaw_overload_g": 0.0,
            "pitch_los_rate_rad_s": 0.0,
            "yaw_los_rate_rad_s": 0.0,
            "target_x": -1.0,
            "target_y": -1.0,
            "fps": float(fps),
            "dt": float(dt),
        }

    pitch_cmd_g = (
        config.get("pitch_output_sign", 1.0)
        * guidance_result.get("pitch_overload_g", 0.0)
    )
    yaw_cmd_g = (
        config.get("yaw_output_sign", -1.0)
        * guidance_result.get("yaw_overload_g", 0.0)
    )

    return {
        "detected": True,
        "pitch_overload_g": pitch_cmd_g,
        "yaw_overload_g": yaw_cmd_g,
        "pitch_los_rate_rad_s": guidance_result.get("pitch_los_rate_rad_s", 0.0),
        "yaw_los_rate_rad_s": guidance_result.get("yaw_los_rate_rad_s", 0.0),
        "target_x": float(detection.get("x", -1.0)),
        "target_y": float(detection.get("y", -1.0)),
        "fps": float(fps),
        "dt": float(dt),
    }


def _lost_guidance_result():
    return {
        "detected": False,
        "pixel_error_x": 0.0,
        "pixel_error_y": 0.0,
        "los_c": [0.0, 0.0, 1.0],
        "los_b": [1.0, 0.0, 0.0],
        "los_s": [1.0, 0.0, 0.0],
        "yaw_los_angle_rad": 0.0,
        "pitch_los_angle_rad": 0.0,
        "yaw_los_rate_rad_s": 0.0,
        "pitch_los_rate_rad_s": 0.0,
        "raw_yaw_los_angle_rad": 0.0,
        "raw_pitch_los_angle_rad": 0.0,
        "raw_yaw_los_rate_rad_s": 0.0,
        "raw_pitch_los_rate_rad_s": 0.0,
        "yaw_rate_control_rad_s": 0.0,
        "pitch_rate_control_rad_s": 0.0,
        "yaw_angle_control_rad_s": 0.0,
        "pitch_angle_control_rad_s": 0.0,
        "yaw_command_rate_rad_s": 0.0,
        "pitch_command_rate_rad_s": 0.0,
        "yaw_overload_g": 0.0,
        "pitch_overload_g": 0.0,
    }


def _copy_matrix(matrix):
    return [[float(value) for value in row] for row in matrix]


def _mat_vec_mul(matrix, vector):
    return [
        matrix[row][0] * vector[0]
        + matrix[row][1] * vector[1]
        + matrix[row][2] * vector[2]
        for row in range(3)
    ]


def _vec_add(left, right):
    return [
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2],
    ]


def _cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def _normalize(vector):
    norm = math.sqrt(
        vector[0] * vector[0]
        + vector[1] * vector[1]
        + vector[2] * vector[2]
    )
    if norm < EPS:
        return [0.0, 0.0, 0.0]
    return [vector[0] / norm, vector[1] / norm, vector[2] / norm]


def _clamp(value, low, high):
    return max(low, min(high, value))


def _limit_overload(overload_g, max_overload_g):
    if max_overload_g is None or max_overload_g <= 0.0:
        return overload_g
    return _clamp(overload_g, -max_overload_g, max_overload_g)


def _axis_or_default(axis_value, default_value):
    return default_value if axis_value is None else axis_value


def _merge_axis_config(defaults, override):
    result = defaults.copy()
    if override is not None:
        result.update(override)
    return result


def _clean_rate_filter_alpha(alpha):
    if alpha is None or alpha < 0.0:
        return None
    return _clamp(alpha, 0.0, 1.0)
