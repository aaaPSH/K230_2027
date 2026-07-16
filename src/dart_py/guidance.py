"""比例导航制导与过载指令生成。"""
import math

from kalman import KalmanFilter


G = 9.80665
EPS = 1e-9


class ProportionalGuidance:
    """将像素目标转换为弹体系横向/法向过载指令。

    坐标约定：camera 为右、下、前；body 为前、右、下。PN 律在滚转稳定
    坐标系计算，随后通过滚转逆变换分配回弹体系 y/z 两个执行通道。
    """

    def __init__(
        self,
        camera_matrix=None,
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
        yaw_max_slew_g_s=0.0,
        pitch_max_slew_g_s=0.0,
        max_prediction_time_s=0.1,
    ):
        if camera_matrix is not None:
            self.camera_matrix = _validate_camera_matrix(camera_matrix)
            self.fx = self.camera_matrix[0][0]
            self.fy = self.camera_matrix[1][1]
            self.cx = self.camera_matrix[0][2]
            self.cy = self.camera_matrix[1][2]
            self.camera_skew = self.camera_matrix[0][1]
        else:
            self.cx = image_width * 0.5 if cx is None else float(cx)
            self.cy = image_height * 0.5 if cy is None else float(cy)
            self.fx = self._focal_length(image_width, fov_x_deg) if fx is None else float(fx)
            self.fy = self._focal_length(image_height, fov_y_deg) if fy is None else float(fy)
            self.camera_skew = 0.0
            self.camera_matrix = [
                [self.fx, self.camera_skew, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ]
        if self.fx <= EPS or self.fy <= EPS:
            raise ValueError("camera focal length must be positive")

        self.R_bc = _copy_matrix(R_bc) if R_bc is not None else [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
        self.yaw_navigation_ratio = _axis_or_default(yaw_navigation_ratio, navigation_ratio)
        self.pitch_navigation_ratio = _axis_or_default(pitch_navigation_ratio, navigation_ratio)
        self.yaw_closing_velocity = _axis_or_default(yaw_closing_velocity, closing_velocity)
        self.pitch_closing_velocity = _axis_or_default(pitch_closing_velocity, closing_velocity)
        self.position_to_rate_gain = float(position_to_rate_gain)
        self.yaw_angle_control_gain = float(yaw_angle_control_gain)
        self.pitch_angle_control_gain = float(pitch_angle_control_gain)
        self.rate_filter_alpha = _clean_rate_filter_alpha(rate_filter_alpha)
        self.use_kalman_filter = bool(use_kalman_filter)
        self.yaw_max_overload_g = _axis_or_default(yaw_max_overload_g, max_overload_g)
        self.pitch_max_overload_g = _axis_or_default(pitch_max_overload_g, max_overload_g)
        self.roll_compensation = bool(roll_compensation)
        self.roll_sign = _unit_sign(roll_sign, "roll_sign")
        self.yaw_max_slew_g_s = _positive_or_zero(yaw_max_slew_g_s)
        self.pitch_max_slew_g_s = _positive_or_zero(pitch_max_slew_g_s)
        self.max_prediction_time_s = _positive_or_zero(max_prediction_time_s)

        kalman_defaults = {
            "angle_variance": kalman_angle_variance,
            "rate_variance": kalman_rate_variance,
            # 保留旧参数名兼容；新的过程噪声是白加速度谱密度。
            "process_accel_variance": kalman_process_rate_variance,
            "measurement_angle_variance": kalman_measurement_angle_variance,
            "innovation_gate_sigma": 3.0,
            "measurement_rate_variance": kalman_measurement_rate_variance,
            "process_angle_variance": kalman_process_angle_variance,
            "process_rate_variance": kalman_process_rate_variance,
        }
        if kalman is not None:
            kalman_defaults.update(kalman)
        self.yaw_kalman = _merge_axis_config(kalman_defaults, yaw_kalman)
        self.pitch_kalman = _merge_axis_config(kalman_defaults, pitch_kalman)
        _resolve_pixel_measurement_noise(self.yaw_kalman, self.fx)
        _resolve_pixel_measurement_noise(self.pitch_kalman, self.fy)

        self.last_los_b = None
        self.last_los_s = None
        self.filtered_yaw_dot = 0.0
        self.filtered_pitch_dot = 0.0
        self.has_filtered_rate = False
        self.yaw_filter = None
        self.pitch_filter = None
        self._last_roll_rad = 0.0
        self._last_yaw_overload_g = None
        self._last_pitch_overload_g = None
        self._prediction_age_s = 0.0

    def reset(self):
        """清除滤波器与输出限速器状态，仅用于明确的时间基准异常。"""
        self.last_los_b = None
        self.last_los_s = None
        self.filtered_yaw_dot = 0.0
        self.filtered_pitch_dot = 0.0
        self.has_filtered_rate = False
        self.yaw_filter = None
        self.pitch_filter = None
        self._last_yaw_overload_g = None
        self._last_pitch_overload_g = None
        self._prediction_age_s = 0.0

    def predict_kalman(self, dt, roll_rad=None, gyro_b=None):
        """兼容旧接口：无量测帧在有限时间内继续预测。"""
        return self.predict(dt, roll_rad=roll_rad, gyro_b=gyro_b)

    def predict(self, dt, roll_rad=None, gyro_b=None):
        """短时丢失视觉量测时预测，超过上限后输出无效结果。"""
        self.last_los_b = None
        self.last_los_s = None
        self.has_filtered_rate = False
        dt = _clean_dt(dt)
        roll = self._effective_roll(roll_rad)
        gyro_b = _vector_or_zero(gyro_b)
        if self.yaw_filter is None or self.pitch_filter is None:
            return self.lost_result()
        self._prediction_age_s += dt
        if (
            self.max_prediction_time_s <= 0.0
            or self._prediction_age_s > self.max_prediction_time_s + EPS
        ):
            self.reset()
            return self.lost_result()

        current_yaw_angle = self.yaw_filter.state()[0]
        current_pitch_angle = self.pitch_filter.state()[0]
        los_s = self._los_from_angles(current_yaw_angle, current_pitch_angle)
        R_roll_comp = self._roll_compensation_matrix(roll)
        gyro_yaw_dot, gyro_pitch_dot = self._gyro_rate_correction(
            los_s,
            gyro_b,
            R_roll_comp,
        )
        yaw_angle, yaw_dot = self._predict_axis_filter(
            self.yaw_filter,
            dt,
            self.yaw_kalman,
            gyro_yaw_dot,
        )
        pitch_angle, pitch_dot = self._predict_axis_filter(
            self.pitch_filter,
            dt,
            self.pitch_kalman,
            gyro_pitch_dot,
        )
        los_s = self._los_from_angles(yaw_angle, pitch_angle)
        relative_yaw_dot = yaw_dot - gyro_yaw_dot
        relative_pitch_dot = pitch_dot - gyro_pitch_dot
        return self._build_result(
            detected=False,
            predicted=True,
            pixel_error_x=0.0,
            pixel_error_y=0.0,
            los_c=[0.0, 0.0, 1.0],
            los_b=[1.0, 0.0, 0.0],
            los_s=los_s,
            yaw_angle=yaw_angle,
            pitch_angle=pitch_angle,
            yaw_dot=yaw_dot,
            pitch_dot=pitch_dot,
            raw_yaw_angle=None,
            raw_pitch_angle=None,
            raw_yaw_dot=None,
            raw_pitch_dot=None,
            relative_yaw_dot=relative_yaw_dot,
            relative_pitch_dot=relative_pitch_dot,
            gyro_yaw_dot=gyro_yaw_dot,
            gyro_pitch_dot=gyro_pitch_dot,
            roll_rad=roll,
            dt=dt,
            filter_reinitialized=False,
        )

    def lost_result(self):
        return _lost_guidance_result()

    def update(self, target_x, target_y, dt=None, roll_rad=None, roll_deg=0.0, gyro_b=None):
        if not _is_finite(target_x) or not _is_finite(target_y) or target_x < 0 or target_y < 0:
            raise ValueError("target coordinates must be finite non-negative values")

        dt = _clean_dt(dt)
        if roll_rad is None and roll_deg is not None:
            roll_rad = math.radians(roll_deg)
        roll = self._effective_roll(roll_rad)
        gyro_b = _vector_or_zero(gyro_b)
        self._prediction_age_s = 0.0
        filter_was_initialized = self.yaw_filter is not None and self.pitch_filter is not None

        los_c = self.pixel_to_camera_los(target_x, target_y)
        los_b = _normalize(_mat_vec_mul(self.R_bc, los_c))
        R_roll_comp = self._roll_compensation_matrix(roll)
        los_s = _normalize(_mat_vec_mul(R_roll_comp, los_b))
        raw_yaw_angle, raw_pitch_angle = self._los_angles(los_s)
        los_dot_b = self._los_dot_body(los_b, dt)
        los_dot_s, relative_rate_valid = self._los_dot_stable(los_s, dt)
        raw_relative_yaw_dot, raw_relative_pitch_dot = self._los_angle_rates(los_s, los_dot_s)
        gyro_yaw_dot, gyro_pitch_dot = self._gyro_rate_correction(
            los_s,
            gyro_b,
            R_roll_comp,
        )
        if not relative_rate_valid and not filter_was_initialized:
            gyro_yaw_dot = 0.0
            gyro_pitch_dot = 0.0
        raw_yaw_dot = raw_relative_yaw_dot + gyro_yaw_dot
        raw_pitch_dot = raw_relative_pitch_dot + gyro_pitch_dot

        (
            yaw_angle,
            yaw_dot,
            pitch_angle,
            pitch_dot,
            filter_reinitialized,
        ) = self._filter_los_states(
            raw_yaw_angle,
            raw_relative_yaw_dot,
            raw_pitch_angle,
            raw_relative_pitch_dot,
            dt,
            gyro_yaw_dot,
            gyro_pitch_dot,
        )
        if filter_reinitialized:
            gyro_yaw_dot = 0.0
            gyro_pitch_dot = 0.0
        relative_yaw_dot = yaw_dot - gyro_yaw_dot
        relative_pitch_dot = pitch_dot - gyro_pitch_dot
        return self._build_result(
            detected=True,
            predicted=False,
            pixel_error_x=target_x - self.cx,
            pixel_error_y=target_y - self.cy,
            los_c=los_c,
            los_b=los_b,
            los_s=los_s,
            yaw_angle=yaw_angle,
            pitch_angle=pitch_angle,
            yaw_dot=yaw_dot,
            pitch_dot=pitch_dot,
            raw_yaw_angle=raw_yaw_angle,
            raw_pitch_angle=raw_pitch_angle,
            raw_yaw_dot=raw_yaw_dot,
            raw_pitch_dot=raw_pitch_dot,
            relative_yaw_dot=relative_yaw_dot,
            relative_pitch_dot=relative_pitch_dot,
            gyro_yaw_dot=gyro_yaw_dot,
            gyro_pitch_dot=gyro_pitch_dot,
            roll_rad=roll,
            dt=dt,
            filter_reinitialized=filter_reinitialized,
        )

    def pixel_to_camera_los(self, target_x, target_y):
        y_n = (target_y - self.cy) / self.fy
        x_n = (target_x - self.cx - self.camera_skew * y_n) / self.fx
        return _normalize([x_n, y_n, 1.0])

    def _build_result(self, detected, predicted, pixel_error_x, pixel_error_y, los_c, los_b, los_s,
                      yaw_angle, pitch_angle, yaw_dot, pitch_dot, raw_yaw_angle,
                      raw_pitch_angle, raw_yaw_dot, raw_pitch_dot, relative_yaw_dot,
                      relative_pitch_dot, gyro_yaw_dot, gyro_pitch_dot, roll_rad, dt,
                      filter_reinitialized):
        yaw_angle_gain = self.position_to_rate_gain + self.yaw_angle_control_gain
        pitch_angle_gain = self.position_to_rate_gain + self.pitch_angle_control_gain
        yaw_angle_control = yaw_angle_gain * yaw_angle
        pitch_angle_control = pitch_angle_gain * pitch_angle
        yaw_command_rate = yaw_dot + yaw_angle_control
        pitch_command_rate = pitch_dot + pitch_angle_control

        # 稳定坐标系 PN 指令先转换为 g，再逆滚转分配到弹体系 y/z。
        # 过载统一定义为实际镖体坐标系分量：+y 向右，+z 向下。
        # yaw LOS 角为向右为正，因此 yaw PN 项保持正号；pitch LOS 角为抬头为正，
        # 而镖体 +z 为向下，所以 pitch PN 项需要取负号。
        stable_yaw_g = self.yaw_navigation_ratio * self.yaw_closing_velocity * yaw_command_rate / G
        stable_pitch_g = -self.pitch_navigation_ratio * self.pitch_closing_velocity * pitch_command_rate / G
        yaw_overload_g, pitch_overload_g = self._allocate_to_body(stable_yaw_g, stable_pitch_g, roll_rad)
        yaw_overload_g = _limit_overload(yaw_overload_g, self.yaw_max_overload_g)
        pitch_overload_g = _limit_overload(pitch_overload_g, self.pitch_max_overload_g)
        yaw_overload_g, pitch_overload_g = self._apply_slew_limit(yaw_overload_g, pitch_overload_g, dt)

        return {
            "detected": bool(detected),
            "predicted": bool(predicted),
            "guidance_valid": True,
            "filter_reinitialized": bool(filter_reinitialized),
            "pixel_error_x": pixel_error_x,
            "pixel_error_y": pixel_error_y,
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
            "relative_yaw_los_rate_rad_s": relative_yaw_dot,
            "relative_pitch_los_rate_rad_s": relative_pitch_dot,
            "gyro_yaw_los_rate_correction_rad_s": gyro_yaw_dot,
            "gyro_pitch_los_rate_correction_rad_s": gyro_pitch_dot,
            "prediction_age_s": self._prediction_age_s,
            "yaw_rate_control_rad_s": yaw_dot,
            "pitch_rate_control_rad_s": pitch_dot,
            "yaw_angle_control_rad_s": yaw_angle_control,
            "pitch_angle_control_rad_s": pitch_angle_control,
            "yaw_command_rate_rad_s": yaw_command_rate,
            "pitch_command_rate_rad_s": pitch_command_rate,
            "stable_yaw_overload_g": stable_yaw_g,
            "stable_pitch_overload_g": stable_pitch_g,
            "yaw_overload_g": yaw_overload_g,
            "pitch_overload_g": pitch_overload_g,
            "body_y_overload_g": yaw_overload_g,
            "body_z_overload_g": pitch_overload_g,
        }

    def _los_dot_body(self, los_b, dt):
        if self.last_los_b is None or dt <= 0.0:
            self.last_los_b = los_b[:]
            return [0.0, 0.0, 0.0]
        result = [(los_b[index] - self.last_los_b[index]) / dt for index in range(3)]
        self.last_los_b = los_b[:]
        return result

    def _los_dot_stable(self, los_s, dt):
        """计算滚转稳定系中的相对 LOS 导数。"""
        if self.last_los_s is None or dt <= 0.0:
            self.last_los_s = los_s[:]
            return [0.0, 0.0, 0.0], False
        result = [(los_s[index] - self.last_los_s[index]) / dt for index in range(3)]
        self.last_los_s = los_s[:]
        return result, True

    def _effective_roll(self, roll_rad):
        if not self.roll_compensation:
            self._last_roll_rad = 0.0
            return 0.0
        if roll_rad is not None:
            if not _is_finite(roll_rad):
                raise ValueError("roll_rad must be finite when supplied")
            self._last_roll_rad = float(roll_rad) * self.roll_sign
        return self._last_roll_rad

    def _roll_compensation_matrix(self, roll_rad):
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        return [[1.0, 0.0, 0.0], [0.0, cos_roll, sin_roll], [0.0, -sin_roll, cos_roll]]

    def _gyro_rate_correction(self, los_s, gyro_b, R_roll_comp):
        """返回稳定系自身俯仰/偏航转动对 LOS 角速度的补偿项。"""
        gyro_s = _mat_vec_mul(R_roll_comp, gyro_b)
        # 稳定系已移除绕前向轴的滚转，不应再次把滚转角速度叠加到 LOS rate。
        gyro_s[0] = 0.0
        los_dot_correction_s = _cross(gyro_s, los_s)
        return self._los_angle_rates(los_s, los_dot_correction_s)

    def _allocate_to_body(self, stable_yaw_g, stable_pitch_g, roll_rad):
        # R_roll_comp 将 body 转到稳定系；其转置即稳定系回 body。
        R = self._roll_compensation_matrix(roll_rad)
        body = [
            R[0][0] * 0.0 + R[1][0] * stable_yaw_g + R[2][0] * stable_pitch_g,
            R[0][1] * 0.0 + R[1][1] * stable_yaw_g + R[2][1] * stable_pitch_g,
            R[0][2] * 0.0 + R[1][2] * stable_yaw_g + R[2][2] * stable_pitch_g,
        ]
        return body[1], body[2]

    def _los_angles(self, los_s):
        x, y, z = los_s
        rho = math.sqrt(x * x + y * y)
        return math.atan2(y, x), math.atan2(-z, rho)

    def _los_from_angles(self, yaw_angle, pitch_angle):
        cos_pitch = math.cos(pitch_angle)
        return [
            cos_pitch * math.cos(yaw_angle),
            cos_pitch * math.sin(yaw_angle),
            -math.sin(pitch_angle),
        ]

    def _los_angle_rates(self, los_s, los_dot_s):
        x, y, z = los_s
        xd, yd, zd = los_dot_s
        rho2 = x * x + y * y
        if rho2 < EPS:
            return 0.0, 0.0
        rho = math.sqrt(rho2)
        return (x * yd - y * xd) / rho2, -rho * zd + z * (x * xd + y * yd) / rho

    def _filter_los_states(
        self,
        yaw_angle,
        yaw_dot,
        pitch_angle,
        pitch_dot,
        dt,
        gyro_yaw_dot=0.0,
        gyro_pitch_dot=0.0,
    ):
        if not self.use_kalman_filter:
            yaw_dot, pitch_dot = self._filter_los_rates(yaw_dot, pitch_dot)
            return (
                yaw_angle,
                yaw_dot + gyro_yaw_dot,
                pitch_angle,
                pitch_dot + gyro_pitch_dot,
                False,
            )
        yaw_angle, yaw_dot, yaw_reset = self._update_axis_filter(
            "yaw_filter",
            yaw_angle,
            dt,
            self.yaw_kalman,
            gyro_yaw_dot,
        )
        pitch_angle, pitch_dot, pitch_reset = self._update_axis_filter(
            "pitch_filter",
            pitch_angle,
            dt,
            self.pitch_kalman,
            gyro_pitch_dot,
        )
        return yaw_angle, yaw_dot, pitch_angle, pitch_dot, yaw_reset or pitch_reset

    def _update_axis_filter(self, attr_name, angle, dt, params, gyro_rate_correction):
        axis_filter = getattr(self, attr_name)
        if axis_filter is None:
            axis_filter = self._create_axis_filter(angle, params)
            setattr(self, attr_name, axis_filter)
            return angle, 0.0, False
        self._predict_axis_filter(axis_filter, dt, params, gyro_rate_correction)
        state = axis_filter.state()
        covariance = axis_filter.covariance()
        measurement_variance = _positive_value(params.get("measurement_angle_variance"), 0.0025)
        innovation_variance = covariance[0][0] + measurement_variance
        residual = angle - state[0]
        gate_sigma = _positive_value(params.get("innovation_gate_sigma"), 3.0)
        if innovation_variance <= EPS or residual * residual > gate_sigma * gate_sigma * innovation_variance:
            axis_filter = self._create_axis_filter(angle, params)
            setattr(self, attr_name, axis_filter)
            return angle, 0.0, True
        state = axis_filter.update([angle])
        return state[0], state[1], False

    def _predict_axis_filter(self, axis_filter, dt, params, gyro_rate_correction=0.0):
        transition = [[1.0, dt], [0.0, 1.0]]
        process_noise = self._process_noise(dt, params)
        return axis_filter.predict(
            control=[gyro_rate_correction * dt],
            transition_matrix=transition,
            process_noise=process_noise,
        )

    def _create_axis_filter(self, angle, params):
        # 状态速度是惯性 LOS rate；状态角是相对滚转稳定系的 LOS 角。
        return KalmanFilter(
            state=[angle, 0.0],
            covariance=[[_positive_value(params.get("angle_variance"), 0.05), 0.0], [0.0, _positive_value(params.get("rate_variance"), 1.0)]],
            transition_matrix=[[1.0, 0.0], [0.0, 1.0]],
            measurement_matrix=[[1.0, 0.0]],
            process_noise=[[0.0, 0.0], [0.0, 0.0]],
            measurement_noise=[[_positive_value(params.get("measurement_angle_variance"), 0.0025)]],
            # 相对 LOS 角的状态转移需减去稳定坐标系自身角位移。
            control_matrix=[[-1.0], [0.0]],
        )

    def _process_noise(self, dt, params):
        q = _positive_or_zero(params.get("process_accel_variance", params.get("process_rate_variance", 0.02)))
        if dt <= 0.0 or q <= 0.0:
            return [[0.0, 0.0], [0.0, 0.0]]
        dt2 = dt * dt
        dt3 = dt2 * dt
        return [[q * dt3 / 3.0, q * dt2 / 2.0], [q * dt2 / 2.0, q * dt]]

    def _filter_los_rates(self, yaw_dot, pitch_dot):
        if self.rate_filter_alpha is None:
            return yaw_dot, pitch_dot
        if not self.has_filtered_rate:
            self.filtered_yaw_dot, self.filtered_pitch_dot = yaw_dot, pitch_dot
            self.has_filtered_rate = True
            return yaw_dot, pitch_dot
        alpha = self.rate_filter_alpha
        self.filtered_yaw_dot = alpha * yaw_dot + (1.0 - alpha) * self.filtered_yaw_dot
        self.filtered_pitch_dot = alpha * pitch_dot + (1.0 - alpha) * self.filtered_pitch_dot
        return self.filtered_yaw_dot, self.filtered_pitch_dot

    def _apply_slew_limit(self, yaw_g, pitch_g, dt):
        yaw_g = self._slew(yaw_g, self._last_yaw_overload_g, self.yaw_max_slew_g_s, dt)
        pitch_g = self._slew(pitch_g, self._last_pitch_overload_g, self.pitch_max_slew_g_s, dt)
        self._last_yaw_overload_g = yaw_g
        self._last_pitch_overload_g = pitch_g
        return yaw_g, pitch_g

    def _slew(self, value, previous, max_slew, dt):
        if previous is None or max_slew <= 0.0 or dt <= 0.0:
            return value
        return _clamp(value, previous - max_slew * dt, previous + max_slew * dt)

    def _focal_length(self, pixels, fov_deg):
        return (pixels * 0.5) / math.tan(math.radians(fov_deg) * 0.5)


def make_guidance_from_config(camera_config, guidance_config):
    return ProportionalGuidance(
        camera_matrix=camera_config.get("camera_matrix"),
        image_width=camera_config["width"], image_height=camera_config["height"],
        fov_x_deg=camera_config.get("fov_x_deg", 65.0), fov_y_deg=camera_config.get("fov_y_deg", 40.0),
        fx=camera_config.get("fx"), fy=camera_config.get("fy"), cx=camera_config.get("cx"), cy=camera_config.get("cy"),
        navigation_ratio=guidance_config["navigation_ratio"],
        yaw_navigation_ratio=guidance_config.get("yaw_navigation_ratio"),
        pitch_navigation_ratio=guidance_config.get("pitch_navigation_ratio"),
        closing_velocity=guidance_config["closing_velocity"],
        yaw_closing_velocity=guidance_config.get("yaw_closing_velocity"),
        pitch_closing_velocity=guidance_config.get("pitch_closing_velocity"),
        position_to_rate_gain=guidance_config.get("position_to_rate_gain", 0.0),
        yaw_angle_control_gain=guidance_config.get("yaw_angle_control_gain", 0.0),
        pitch_angle_control_gain=guidance_config.get("pitch_angle_control_gain", 0.0),
        rate_filter_alpha=guidance_config.get("rate_filter_alpha"),
        use_kalman_filter=guidance_config.get("use_kalman_filter", True),
        kalman=guidance_config.get("kalman"), yaw_kalman=guidance_config.get("yaw_kalman"), pitch_kalman=guidance_config.get("pitch_kalman"),
        max_overload_g=guidance_config.get("max_overload_g", 0.5),
        yaw_max_overload_g=guidance_config.get("yaw_max_overload_g"), pitch_max_overload_g=guidance_config.get("pitch_max_overload_g"),
        roll_compensation=guidance_config.get("roll_compensation", True), roll_sign=guidance_config.get("roll_sign", 1.0),
        yaw_max_slew_g_s=guidance_config.get("yaw_max_slew_g_s", 0.0), pitch_max_slew_g_s=guidance_config.get("pitch_max_slew_g_s", 0.0),
        max_prediction_time_s=guidance_config.get("max_prediction_time_s", 0.1),
    )


def build_overload_command(detection, guidance_result, fps=0.0, dt=0.0, config=None):
    """构造镖体坐标系过载；仅允许有效量测或限时预测结果产生非零指令。"""
    config = config or {}
    guidance_valid = bool(guidance_result and guidance_result.get("guidance_valid", False))
    raw_detected = bool(detection and detection.get("detected", False))
    predicted = bool(guidance_result and guidance_result.get("predicted", False))
    if not guidance_valid:
        return {
            "detected": False, "predicted": False, "guidance_valid": False,
            "pitch_overload_g": 0.0, "yaw_overload_g": 0.0,
            "body_y_overload_g": 0.0, "body_z_overload_g": 0.0,
            "pitch_los_rate_rad_s": 0.0, "yaw_los_rate_rad_s": 0.0,
            "target_x": -1.0, "target_y": -1.0, "fps": float(fps), "dt": float(dt),
        }
    body_y_overload_g = guidance_result.get(
        "body_y_overload_g", guidance_result.get("yaw_overload_g", 0.0)
    )
    body_z_overload_g = guidance_result.get(
        "body_z_overload_g", guidance_result.get("pitch_overload_g", 0.0)
    )
    return {
        "detected": raw_detected, "predicted": predicted, "guidance_valid": True,
        "pitch_overload_g": body_z_overload_g,
        "yaw_overload_g": body_y_overload_g,
        "body_y_overload_g": body_y_overload_g,
        "body_z_overload_g": body_z_overload_g,
        "pitch_los_rate_rad_s": guidance_result.get("pitch_los_rate_rad_s", 0.0),
        "yaw_los_rate_rad_s": guidance_result.get("yaw_los_rate_rad_s", 0.0),
        "target_x": float(detection.get("x", -1.0)) if raw_detected else -1.0,
        "target_y": float(detection.get("y", -1.0)) if raw_detected else -1.0,
        "fps": float(fps), "dt": float(dt),
    }


def _lost_guidance_result():
    return {
        "detected": False, "predicted": False, "guidance_valid": False,
        "filter_reinitialized": False, "pixel_error_x": 0.0, "pixel_error_y": 0.0,
        "los_c": [0.0, 0.0, 1.0], "los_b": [1.0, 0.0, 0.0], "los_s": [1.0, 0.0, 0.0],
        "yaw_los_angle_rad": 0.0, "pitch_los_angle_rad": 0.0,
        "yaw_los_rate_rad_s": 0.0, "pitch_los_rate_rad_s": 0.0,
        "raw_yaw_los_angle_rad": None, "raw_pitch_los_angle_rad": None,
        "raw_yaw_los_rate_rad_s": None, "raw_pitch_los_rate_rad_s": None,
        "relative_yaw_los_rate_rad_s": 0.0, "relative_pitch_los_rate_rad_s": 0.0,
        "gyro_yaw_los_rate_correction_rad_s": 0.0,
        "gyro_pitch_los_rate_correction_rad_s": 0.0,
        "prediction_age_s": 0.0,
        "yaw_rate_control_rad_s": 0.0, "pitch_rate_control_rad_s": 0.0,
        "yaw_angle_control_rad_s": 0.0, "pitch_angle_control_rad_s": 0.0,
        "yaw_command_rate_rad_s": 0.0, "pitch_command_rate_rad_s": 0.0,
        "stable_yaw_overload_g": 0.0, "stable_pitch_overload_g": 0.0,
        "yaw_overload_g": 0.0, "pitch_overload_g": 0.0,
    }


def _copy_matrix(matrix):
    return [[float(value) for value in row] for row in matrix]


def _validate_camera_matrix(camera_matrix):
    matrix = _copy_matrix(camera_matrix)
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("camera_matrix must be a 3x3 matrix")
    if not all(_is_finite(value) for row in matrix for value in row):
        raise ValueError("camera_matrix must contain finite values")
    if matrix[0][0] <= EPS or matrix[1][1] <= EPS:
        raise ValueError("camera_matrix focal lengths must be positive")
    if (
        abs(matrix[1][0]) > EPS
        or abs(matrix[2][0]) > EPS
        or abs(matrix[2][1]) > EPS
        or abs(matrix[2][2] - 1.0) > EPS
    ):
        raise ValueError("camera_matrix must use the standard pinhole form")
    return matrix


def _resolve_pixel_measurement_noise(params, focal_length_px):
    if "measurement_noise_px" not in params:
        return
    noise_px = _positive_value(params.get("measurement_noise_px"), 1.0)
    sigma_rad = math.atan(noise_px / focal_length_px)
    params["measurement_angle_variance"] = sigma_rad * sigma_rad


def _unit_sign(value, name):
    value = float(value)
    if not _is_finite(value) or abs(abs(value) - 1.0) > EPS:
        raise ValueError(name + " must be +1.0 or -1.0")
    return value


def _mat_vec_mul(matrix, vector):
    return [matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2] for row in range(3)]


def _vec_add(left, right):
    return [left[index] + right[index] for index in range(3)]


def _cross(left, right):
    return [left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2], left[0] * right[1] - left[1] * right[0]]


def _normalize(vector):
    norm = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if norm < EPS:
        raise ValueError("LOS vector norm is zero")
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


def _clean_dt(dt):
    return float(dt) if _is_finite(dt) and dt > 0.0 else 0.0


def _positive_or_zero(value):
    return float(value) if _is_finite(value) and value > 0.0 else 0.0


def _positive_value(value, fallback):
    return float(value) if _is_finite(value) and value > EPS else fallback


def _is_finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return value == value and value != float("inf") and value != -float("inf")


def _vector_or_zero(vector):
    if vector is None:
        return [0.0, 0.0, 0.0]
    if len(vector) != 3 or not all(_is_finite(value) for value in vector):
        raise ValueError("gyro_b must contain three finite values when supplied")
    return [float(value) for value in vector]
