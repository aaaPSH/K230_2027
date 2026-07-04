"""根据像素视线向量计算比例导引过载量"""
import math

from kalman import KalmanFilter


G = 9.80665
EPS = 1e-9


class ProportionalGuidance:
    """
    像素坐标 -> 相机视线向量 -> 镖体系视线向量 -> roll稳定系视线角速度 -> 过载量。

    默认坐标约定：
    - 相机系：x 向右，y 向下，z 沿光轴向前
    - 镖体系：x 向前，y 向右，z 向下
    - gyro_b：镖体系角速度 [roll_rate, pitch_rate, yaw_rate]，单位 rad/s
    - yaw_overload_g > 0：目标在右侧，需要向右修正
    - pitch_overload_g > 0：目标在上方，需要抬头修正
    """

    def __init__(
        self,
        cx=None,
        cy=None,
        fx=None,
        fy=None,
        image_width=640,
        image_height=480,
        fov_x_deg=60.0,
        fov_y_deg=45.0,
        R_bc=None,
        navigation_ratio=3.0,
        closing_velocity=15.0,
        position_to_rate_gain=0.0,
        yaw_angle_control_gain=0.0,
        pitch_angle_control_gain=0.0,
        rate_filter_alpha=None,
        use_kalman_filter=True,
        kalman_angle_variance=0.05,
        kalman_rate_variance=1.0,
        kalman_process_angle_variance=0.0001,
        kalman_process_rate_variance=0.02,
        kalman_measurement_angle_variance=0.0025,
        kalman_measurement_rate_variance=10.0,
        max_overload_g=6.0,
        roll_compensation=True,
        roll_sign=1.0,
    ):
        self.cx = image_width * 0.5 if cx is None else cx
        self.cy = image_height * 0.5 if cy is None else cy
        self.fx = self._focal_length(image_width, fov_x_deg) if fx is None else fx
        self.fy = self._focal_length(image_height, fov_y_deg) if fy is None else fy

        # Camera x-right/y-down/z-forward -> body x-forward/y-right/z-down.
        self.R_bc = _copy_matrix(R_bc) if R_bc is not None else [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]

        self.navigation_ratio = navigation_ratio
        self.closing_velocity = closing_velocity
        self.position_to_rate_gain = position_to_rate_gain
        self.yaw_angle_control_gain = yaw_angle_control_gain
        self.pitch_angle_control_gain = pitch_angle_control_gain
        if rate_filter_alpha is None:
            self.rate_filter_alpha = None
        else:
            self.rate_filter_alpha = _clamp(rate_filter_alpha, 0.0, 1.0)
        self.use_kalman_filter = use_kalman_filter
        self.kalman_angle_variance = kalman_angle_variance
        self.kalman_rate_variance = kalman_rate_variance
        self.kalman_process_angle_variance = kalman_process_angle_variance
        self.kalman_process_rate_variance = kalman_process_rate_variance
        self.kalman_measurement_angle_variance = kalman_measurement_angle_variance
        self.kalman_measurement_rate_variance = kalman_measurement_rate_variance
        self.max_overload_g = max_overload_g
        self.roll_compensation = roll_compensation
        self.roll_sign = roll_sign

        self.last_los_b = None
        self.filtered_yaw_dot = 0.0
        self.filtered_pitch_dot = 0.0
        self.has_filtered_rate = False
        self.yaw_filter = None
        self.pitch_filter = None

    def reset(self):
        """目标丢失或重新发射时调用，清除上一帧视线向量"""
        self.last_los_b = None
        self.filtered_yaw_dot = 0.0
        self.filtered_pitch_dot = 0.0
        self.has_filtered_rate = False
        self.yaw_filter = None
        self.pitch_filter = None

    def update(
        self,
        target_x,
        target_y,
        dt=None,
        roll_rad=None,
        roll_deg=0.0,
        gyro_b=None,
    ):
        """
        输入目标像素坐标，输出 pitch/yaw 方向过载量。

        dt：两帧间隔，单位秒。
        roll_rad/roll_deg：当前 roll 角，优先使用 roll_rad。
        gyro_b：镖体系角速度 [p, q, r]，单位 rad/s；没有陀螺仪角速度时传 None。
        """
        if target_x < 0 or target_y < 0:
            self.reset()
            return self._lost_result()

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

        yaw_angle_gain = self.position_to_rate_gain + self.yaw_angle_control_gain
        pitch_angle_gain = self.position_to_rate_gain + self.pitch_angle_control_gain
        yaw_rate_control = yaw_dot
        pitch_rate_control = pitch_dot
        yaw_angle_control = yaw_angle_gain * yaw_angle
        pitch_angle_control = pitch_angle_gain * pitch_angle
        yaw_command_rate = yaw_rate_control + yaw_angle_control
        pitch_command_rate = pitch_rate_control + pitch_angle_control
        yaw_overload_g = (
            self.navigation_ratio * self.closing_velocity * yaw_command_rate / G
        )
        pitch_overload_g = (
            self.navigation_ratio * self.closing_velocity * pitch_command_rate / G
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
            "yaw_overload_g": self._limit_overload(yaw_overload_g),
            "pitch_overload_g": self._limit_overload(pitch_overload_g),
        }

    def pixel_to_camera_los(self, target_x, target_y):
        """像素坐标转归一化相机系视线向量"""
        x_n = (target_x - self.cx) / self.fx
        y_n = (target_y - self.cy) / self.fy
        return _normalize([x_n, y_n, 1.0])

    def _los_dot_body(self, los_b, dt):
        if self.last_los_b is None or dt is None or dt <= 0.0:
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
            )
            pitch_angle, pitch_dot = self._filter_axis_state(
                self.pitch_filter,
                pitch_angle,
                pitch_dot,
                dt,
                "pitch_filter",
            )
            return yaw_angle, yaw_dot, pitch_angle, pitch_dot

        yaw_dot, pitch_dot = self._filter_los_rates(yaw_dot, pitch_dot)
        return yaw_angle, yaw_dot, pitch_angle, pitch_dot

    def _filter_axis_state(self, axis_filter, angle, rate, dt, attr_name):
        if axis_filter is None:
            axis_filter = self._create_axis_filter(angle, rate)
            setattr(self, attr_name, axis_filter)
            return angle, rate

        if dt is None or dt <= 0.0:
            dt = 0.0

        transition_matrix = [
            [1.0, dt],
            [0.0, 1.0],
        ]
        state = axis_filter.step(
            [angle, rate],
            transition_matrix=transition_matrix,
        )
        return state[0], state[1]

    def _create_axis_filter(self, angle, rate):
        return KalmanFilter(
            state=[angle, rate],
            covariance=[
                [self.kalman_angle_variance, 0.0],
                [0.0, self.kalman_rate_variance],
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
                [self.kalman_process_angle_variance, 0.0],
                [0.0, self.kalman_process_rate_variance],
            ],
            measurement_noise=[
                [self.kalman_measurement_angle_variance, 0.0],
                [0.0, self.kalman_measurement_rate_variance],
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

    def _limit_overload(self, overload_g):
        if self.max_overload_g is None or self.max_overload_g <= 0:
            return overload_g
        return _clamp(overload_g, -self.max_overload_g, self.max_overload_g)

    def _lost_result(self):
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
            "yaw_rate_control_rad_s": 0.0,
            "pitch_rate_control_rad_s": 0.0,
            "yaw_angle_control_rad_s": 0.0,
            "pitch_angle_control_rad_s": 0.0,
            "yaw_command_rate_rad_s": 0.0,
            "pitch_command_rate_rad_s": 0.0,
            "yaw_overload_g": 0.0,
            "pitch_overload_g": 0.0,
        }

    def _focal_length(self, pixels, fov_deg):
        fov_rad = math.radians(fov_deg)
        return (pixels * 0.5) / math.tan(fov_rad * 0.5)


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
