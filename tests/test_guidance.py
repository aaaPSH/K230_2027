"""制导律关键坐标、滤波与丢失目标行为回归测试。"""
import math
import os
import struct
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "src", "dart_py"))

from attitude import AttitudeWorker
from flight_log import FlightLogger
from guidance import G, ProportionalGuidance, build_overload_command
from imu_uart import IMU_FRAME_TAIL, SerialImuReader


def make_guidance(**overrides):
    config = {
        "image_width": 320,
        "image_height": 240,
        "fov_x_deg": 65.0,
        "fov_y_deg": 40.0,
        "navigation_ratio": 1.0,
        "closing_velocity": G,
        "max_overload_g": 10.0,
        "roll_compensation": True,
        "roll_sign": 1.0,
        "use_kalman_filter": True,
        "kalman": {
            "angle_variance": 0.001,
            "rate_variance": 1.0,
            "process_accel_variance": 0.02,
            "measurement_angle_variance": 0.0001,
            "innovation_gate_sigma": 3.0,
        },
    }
    config.update(overrides)
    return ProportionalGuidance(**config)


class GuidanceTest(unittest.TestCase):
    def test_static_center_target_has_zero_command(self):
        guidance = make_guidance()
        guidance.update(160.0, 120.0, dt=0.02)
        result = guidance.update(160.0, 120.0, dt=0.02)
        self.assertAlmostEqual(result["yaw_overload_g"], 0.0, places=7)
        self.assertAlmostEqual(result["pitch_overload_g"], 0.0, places=7)

    def test_kalman_uses_angle_measurement_only(self):
        guidance = make_guidance()
        guidance.update(160.0, 120.0, dt=0.02)
        self.assertEqual(guidance.yaw_filter.measurement_size, 1)
        self.assertEqual(guidance.pitch_filter.measurement_size, 1)

    def test_body_yaw_is_removed_from_inertial_los_rate(self):
        guidance = make_guidance(roll_compensation=False)
        dt = 1.0 / 90.0
        yaw_rate = 0.1
        result = None
        for index in range(46):
            angle = -yaw_rate * index * dt
            target_x = guidance.cx + guidance.fx * math.tan(angle)
            result = guidance.update(
                target_x,
                guidance.cy,
                dt=dt,
                gyro_b=[0.0, 0.0, yaw_rate],
            )
        self.assertAlmostEqual(result["raw_yaw_los_rate_rad_s"], 0.0, places=4)
        self.assertAlmostEqual(result["yaw_los_rate_rad_s"], 0.0, places=3)

    def test_body_pitch_is_removed_from_inertial_los_rate(self):
        guidance = make_guidance(roll_compensation=False)
        dt = 1.0 / 90.0
        pitch_rate = 0.1
        result = None
        for index in range(46):
            angle = pitch_rate * index * dt
            target_y = guidance.cy + guidance.fy * math.tan(angle)
            result = guidance.update(
                guidance.cx,
                target_y,
                dt=dt,
                gyro_b=[0.0, pitch_rate, 0.0],
            )
        self.assertAlmostEqual(result["raw_pitch_los_rate_rad_s"], 0.0, places=4)
        self.assertAlmostEqual(result["pitch_los_rate_rad_s"], 0.0, places=3)

    def test_roll_stabilization_keeps_fixed_stable_los_rate_zero(self):
        guidance = make_guidance()
        stable_los = [1.0, 0.1, -0.05]
        norm = math.sqrt(sum(value * value for value in stable_los))
        stable_los = [value / norm for value in stable_los]
        result = None
        for index in range(46):
            roll = 0.4 * index / 45.0
            R = guidance._roll_compensation_matrix(roll)
            body_los = [
                sum(R[row][col] * stable_los[row] for row in range(3))
                for col in range(3)
            ]
            target_x = guidance.cx + guidance.fx * body_los[1] / body_los[0]
            target_y = guidance.cy + guidance.fy * body_los[2] / body_los[0]
            result = guidance.update(
                target_x,
                target_y,
                dt=1.0 / 90.0,
                roll_rad=roll,
                gyro_b=[0.4 * 90.0 / 45.0, 0.0, 0.0],
            )
        self.assertAlmostEqual(result["yaw_los_rate_rad_s"], 0.0, places=5)
        self.assertAlmostEqual(result["pitch_los_rate_rad_s"], 0.0, places=5)

    def test_roll_inverse_maps_stable_yaw_to_body_pitch_at_90_degrees(self):
        guidance = make_guidance()
        yaw_g, pitch_g = guidance._allocate_to_body(0.1, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(yaw_g, 0.0, places=6)
        self.assertAlmostEqual(pitch_g, 0.1, places=6)

    def test_lost_target_predicts_briefly_then_invalidates_command(self):
        guidance = make_guidance(position_to_rate_gain=1.0)
        target_x = guidance.cx + guidance.fx * math.tan(0.1)
        guidance.update(target_x, guidance.cy, dt=0.02)
        predicted = guidance.predict(0.02)
        command = build_overload_command(
            {"detected": False},
            predicted,
            config={"yaw_output_sign": 1.0, "pitch_output_sign": 1.0},
        )
        self.assertTrue(predicted["guidance_valid"])
        self.assertTrue(predicted["predicted"])
        self.assertTrue(command["guidance_valid"])
        self.assertFalse(command["detected"])
        self.assertGreater(abs(command["yaw_overload_g"]), 0.05)

        for _ in range(4):
            predicted = guidance.predict(0.02)
        self.assertTrue(predicted["guidance_valid"])
        expired = guidance.predict(0.02)
        expired_command = build_overload_command(
            {"detected": False},
            expired,
            config={"yaw_output_sign": 1.0, "pitch_output_sign": 1.0},
        )
        self.assertFalse(expired["guidance_valid"])
        self.assertEqual(expired_command["yaw_overload_g"], 0.0)
        self.assertEqual(expired_command["pitch_overload_g"], 0.0)

    def test_camera_matrix_is_used_for_pixel_to_los(self):
        camera_matrix = [
            [200.0, 5.0, 100.0],
            [0.0, 220.0, 80.0],
            [0.0, 0.0, 1.0],
        ]
        guidance = make_guidance(camera_matrix=camera_matrix)
        los = guidance.pixel_to_camera_los(121.0, 124.0)
        scale = math.sqrt(1.0 + 0.1 * 0.1 + 0.2 * 0.2)
        self.assertAlmostEqual(los[0], 0.1 / scale, places=7)
        self.assertAlmostEqual(los[1], 0.2 / scale, places=7)
        self.assertAlmostEqual(los[2], 1.0 / scale, places=7)

    def test_pixel_noise_is_converted_by_axis_focal_length(self):
        guidance = make_guidance(
            camera_matrix=[
                [200.0, 0.0, 100.0],
                [0.0, 400.0, 80.0],
                [0.0, 0.0, 1.0],
            ],
            kalman={
                "angle_variance": 0.001,
                "rate_variance": 1.0,
                "process_accel_variance": 0.02,
                "measurement_noise_px": 2.0,
                "innovation_gate_sigma": 3.0,
            },
        )
        self.assertAlmostEqual(
            guidance.yaw_kalman["measurement_angle_variance"],
            math.atan(2.0 / 200.0) ** 2,
            places=12,
        )
        self.assertAlmostEqual(
            guidance.pitch_kalman["measurement_angle_variance"],
            math.atan(2.0 / 400.0) ** 2,
            places=12,
        )

    def test_control_direction_signs_only_accept_unit_values(self):
        with self.assertRaises(ValueError):
            make_guidance(roll_sign=0.0)
        guidance = make_guidance()
        result = guidance.update(guidance.cx, guidance.cy, dt=0.02)
        with self.assertRaises(ValueError):
            build_overload_command(
                {"detected": True, "x": guidance.cx, "y": guidance.cy},
                result,
                config={"yaw_output_sign": 0.5, "pitch_output_sign": 1.0},
            )

    def test_large_reacquisition_innovation_reinitializes_filter(self):
        guidance = make_guidance()
        guidance.update(guidance.cx, guidance.cy, dt=0.02)
        result = guidance.update(guidance.cx + guidance.fx * 0.5, guidance.cy, dt=0.02)
        self.assertTrue(result["filter_reinitialized"])
        self.assertAlmostEqual(guidance.yaw_filter.state()[1], 0.0, places=7)


class ImuInputTest(unittest.TestCase):
    def test_non_finite_uart_frame_is_rejected(self):
        reader = SerialImuReader.__new__(SerialImuReader)
        frame = struct.pack("<8f", float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) + IMU_FRAME_TAIL
        with self.assertRaises(ValueError):
            reader._parse_binary_frame(frame)

    def test_invalid_uart_packet_is_not_silently_dropped_in_debug_mode(self):
        reader = SerialImuReader.__new__(SerialImuReader)
        reader.invalid_packet_count = 0
        frame = struct.pack("<8f", float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) + IMU_FRAME_TAIL
        with self.assertRaises(ValueError):
            reader._append_packet(frame)
        self.assertEqual(reader.invalid_packet_count, 1)

    def test_uart_frame_tail_is_checked_as_raw_bytes(self):
        reader = SerialImuReader.__new__(SerialImuReader)
        frame = struct.pack("<8f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0) + IMU_FRAME_TAIL
        record = reader._parse_binary_frame(frame)
        self.assertEqual(record["frame_tail"], IMU_FRAME_TAIL)
        invalid = frame[:-4] + b"\x00\x00\x00\x00"
        with self.assertRaises(ValueError):
            reader._parse_binary_frame(invalid)

    def test_non_finite_attitude_sample_raises(self):
        class BadImu:
            def read(self):
                return {"gyro_b": [float("nan"), 0.0, 0.0], "accel_b": [0.0, 0.0, 9.8]}

        worker = AttitudeWorker(BadImu(), {"initial_roll_samples": 1})
        with self.assertRaises(ValueError):
            worker.sample_once(timestamp_us=1)


class FlightLogTest(unittest.TestCase):
    def test_flight_log_row_matches_extended_header(self):
        row = FlightLogger.build_row(
            1,
            2,
            90.0,
            {"detected": False},
            {"guidance_valid": False, "sensor_valid": False},
            {"detected": False},
        )
        self.assertEqual(len(row), len(FlightLogger.FIELDS))


if __name__ == "__main__":
    unittest.main()
