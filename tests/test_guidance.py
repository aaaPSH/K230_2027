"""制导律关键坐标、滤波与丢失目标行为回归测试。"""
import math
import os
import struct
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "src", "dart_py"))

from attitude import AttitudeWorker
from command_output import SerialCommandOutput, pack_command_frame
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

    def test_body_overload_signs_match_dart_axes(self):
        guidance = make_guidance(use_kalman_filter=False)

        guidance.update(160.0, 150.0, dt=0.1)
        upward = guidance.update(160.0, 140.0, dt=0.1)
        self.assertLess(upward["pitch_overload_g"], 0.0)
        self.assertLess(upward["body_z_overload_g"], 0.0)

        guidance.reset()
        guidance.update(160.0, 90.0, dt=0.1)
        downward = guidance.update(160.0, 100.0, dt=0.1)
        self.assertGreater(downward["pitch_overload_g"], 0.0)
        self.assertGreater(downward["body_z_overload_g"], 0.0)

        guidance.reset()
        guidance.update(180.0, 120.0, dt=0.1)
        rightward = guidance.update(190.0, 120.0, dt=0.1)
        self.assertGreater(rightward["yaw_overload_g"], 0.0)
        self.assertGreater(rightward["body_y_overload_g"], 0.0)

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

    def test_body_overload_command_does_not_apply_output_signs(self):
        with self.assertRaises(ValueError):
            make_guidance(roll_sign=0.0)
        guidance = make_guidance()
        result = guidance.update(guidance.cx, guidance.cy, dt=0.02)
        command = build_overload_command(
            {"detected": True, "x": guidance.cx, "y": guidance.cy},
            result,
            config={"yaw_output_sign": -1.0, "pitch_output_sign": -1.0},
        )
        self.assertEqual(command["yaw_overload_g"], result["body_y_overload_g"])
        self.assertEqual(command["pitch_overload_g"], result["body_z_overload_g"])

    def test_large_reacquisition_innovation_reinitializes_filter(self):
        guidance = make_guidance()
        guidance.update(guidance.cx, guidance.cy, dt=0.02)
        result = guidance.update(guidance.cx + guidance.fx * 0.5, guidance.cy, dt=0.02)
        self.assertTrue(result["filter_reinitialized"])
        self.assertAlmostEqual(guidance.yaw_filter.state()[1], 0.0, places=7)


class ImuInputTest(unittest.TestCase):
    @staticmethod
    def _make_reader(chunks):
        class FakeUart:
            def __init__(self, data_chunks):
                self.data_chunks = list(data_chunks)

            def read(self):
                if not self.data_chunks:
                    return b""
                return self.data_chunks.pop(0)

        reader = SerialImuReader.__new__(SerialImuReader)
        reader.uart = FakeUart(chunks)
        reader.accel_to_body = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        reader.gyro_to_body = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        reader.max_pending_packets = 32
        reader.max_consecutive_invalid_frames = 3
        reader._rx_buffer = bytearray()
        reader._pending_packets = []
        reader.invalid_packet_count = 0
        reader.consecutive_invalid_count = 0
        reader.imu_fault = False
        reader.last_invalid_error = None
        return reader

    @staticmethod
    def _frame(first_value):
        values = [first_value, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        return struct.pack("<8f", *values) + IMU_FRAME_TAIL

    def test_non_finite_uart_frame_is_rejected(self):
        reader = SerialImuReader.__new__(SerialImuReader)
        frame = struct.pack("<8f", float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) + IMU_FRAME_TAIL
        with self.assertRaises(ValueError):
            reader._parse_binary_frame(frame)

    def test_invalid_uart_packet_does_not_escape_reader(self):
        reader = self._make_reader([])
        frame = struct.pack("<8f", float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) + IMU_FRAME_TAIL
        reader._append_packet(frame)
        self.assertEqual(reader.invalid_packet_count, 1)
        self.assertEqual(reader.consecutive_invalid_count, 1)
        self.assertFalse(reader.imu_fault)

    def test_stream_parser_accepts_split_frame_and_resynchronizes_after_bad_frame(self):
        bad = struct.pack("<8f", float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) + IMU_FRAME_TAIL
        good = self._frame(11.0)
        reader = self._make_reader([b"\x55", bad, good[:13], good[13:]])

        self.assertIsNone(reader.read())
        self.assertEqual(reader.invalid_packet_count, 0)
        self.assertIsNone(reader.read())
        self.assertEqual(reader.invalid_packet_count, 1)
        self.assertIsNone(reader.read())
        sample = reader.read()
        self.assertIsNotNone(sample)
        self.assertEqual(sample["uart_fields"]["ax"], 11.0)
        self.assertEqual(reader.consecutive_invalid_count, 0)
        self.assertFalse(reader.imu_fault)

    def test_many_consecutive_bad_frames_mark_imu_fault_without_raising(self):
        bad = struct.pack("<8f", float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) + IMU_FRAME_TAIL
        reader = self._make_reader([bad * 3])
        self.assertIsNone(reader.read())
        self.assertEqual(reader.invalid_packet_count, 3)
        self.assertTrue(reader.imu_fault)

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


class CommandOutputTest(unittest.TestCase):
    def test_command_frame_matches_lower_computer_protocol(self):
        frame = pack_command_frame(1.25, -2.5)
        expected_without_checksum = b"\x5a\xa5" + struct.pack("<2f", 1.25, -2.5)
        self.assertEqual(frame[:10], expected_without_checksum)
        self.assertEqual(len(frame), 11)
        self.assertEqual(frame[10], sum(frame[:10]) & 0xFF)

    def test_serial_command_output_maps_identity_axes(self):
        class FakeUart:
            def __init__(self):
                self.frames = []

            def write(self, frame):
                self.frames.append(frame)

        uart = FakeUart()
        output = SerialCommandOutput(
            {"lateral_imu_axis": 1, "normal_imu_axis": 2}, uart=uart
        )
        output.send_overload(
            {
                "guidance_valid": True,
                "yaw_overload_g": 0.25,
                "pitch_overload_g": -0.5,
            }
        )
        self.assertEqual(len(uart.frames), 1)
        self.assertEqual(uart.frames[0], pack_command_frame(0.25, -0.5))

    def test_serial_command_output_converts_body_command_to_imu_axes(self):
        class FakeUart:
            def __init__(self):
                self.frames = []

            def write(self, frame):
                self.frames.append(frame)

        uart = FakeUart()
        output = SerialCommandOutput(
            {
                # body = [imu_y, imu_x, -imu_z]
                "imu_to_body": [
                    [0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0],
                ]
            },
            uart=uart,
        )
        output.send_overload(
            {
                "guidance_valid": True,
                "yaw_overload_g": 0.25,
                "pitch_overload_g": -0.5,
            }
        )
        self.assertEqual(uart.frames[0], pack_command_frame(0.25, 0.5))

    def test_invalid_command_value_is_rejected(self):
        with self.assertRaises(ValueError):
            pack_command_frame(float("nan"), 0.0)

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
