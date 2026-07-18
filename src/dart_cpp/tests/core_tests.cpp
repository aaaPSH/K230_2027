#include "guidance.hpp"
#include "kalman.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void expectTrue(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void expectNear(double actual, double expected, double tolerance, const std::string& message)
{
    if (std::abs(actual - expected) > tolerance)
    {
        throw std::runtime_error(message);
    }
}

GuidanceConfig makeGuidanceConfig(bool use_kalman = true)
{
    GuidanceConfig config;
    config.image_width = 320;
    config.image_height = 240;
    config.navigation_ratio = 1.0;
    config.closing_velocity = kGuidanceGravity;
    config.max_overload_g = 10.0;
    config.roll_sign = 1.0;
    config.use_kalman_filter = use_kalman;
    config.yaw_kalman.angle_variance = 0.001;
    config.yaw_kalman.rate_variance = 1.0;
    config.yaw_kalman.process_accel_variance = 0.02;
    config.yaw_kalman.measurement_noise_px.reset();
    config.yaw_kalman.measurement_angle_variance = 0.0001;
    config.yaw_kalman.innovation_gate_sigma = 3.0;
    config.pitch_kalman = config.yaw_kalman;
    return config;
}

void testKalmanFilter()
{
    KalmanFilter filter = makeConstantVelocityFilter(
        1.0, 2.0, 0.5, 1.0, 1.0, 0.01, 0.1);
    expectTrue(filter.stateSize() == 2, "卡尔曼状态维度错误");
    expectTrue(filter.measurementSize() == 1, "卡尔曼量测维度错误");
    expectNear(filter.predict().at<double>(0), 2.0, 1e-12, "匀速预测错误");
    const cv::Mat corrected = filter.update(
        (cv::Mat_<double>(1, 1) << 2.1));
    expectTrue(std::isfinite(corrected.at<double>(0)), "卡尔曼更新无效");
}

void testStaticCenterTarget()
{
    ProportionalGuidance guidance(makeGuidanceConfig());
    guidance.update(160.0, 120.0, 0.02);
    const GuidanceResult result = guidance.update(160.0, 120.0, 0.02);
    expectNear(result.yaw_overload_g, 0.0, 1e-7, "中心偏航指令不为零");
    expectNear(result.pitch_overload_g, 0.0, 1e-7, "中心俯仰指令不为零");
}

void testBodyOverloadSigns()
{
    ProportionalGuidance guidance(makeGuidanceConfig(false));
    guidance.update(160.0, 150.0, 0.1);
    const GuidanceResult upward = guidance.update(160.0, 140.0, 0.1);
    expectTrue(upward.body_z_overload_g < 0.0, "向上目标 z 指令符号错误");

    guidance.reset();
    guidance.update(180.0, 120.0, 0.1);
    const GuidanceResult rightward = guidance.update(190.0, 120.0, 0.1);
    expectTrue(rightward.body_y_overload_g > 0.0, "向右目标 y 指令符号错误");
}

void testCameraMatrixAndNoiseConversion()
{
    GuidanceConfig config = makeGuidanceConfig();
    config.camera_matrix = cv::Matx33d(
        200.0, 5.0, 100.0,
        0.0, 220.0, 80.0,
        0.0, 0.0, 1.0);
    config.yaw_kalman.measurement_noise_px = 2.0;
    config.pitch_kalman.measurement_noise_px = 2.0;
    ProportionalGuidance guidance(config);

    const cv::Vec3d los = guidance.pixelToCameraLos(121.0, 124.0);
    const double scale = std::sqrt(1.0 + 0.1 * 0.1 + 0.2 * 0.2);
    expectNear(los[0], 0.1 / scale, 1e-7, "相机 LOS x 计算错误");
    expectNear(los[1], 0.2 / scale, 1e-7, "相机 LOS y 计算错误");
    expectNear(
        guidance.yawKalmanConfig().measurement_angle_variance,
        std::pow(std::atan(2.0 / 200.0), 2),
        1e-12,
        "偏航像素噪声换算错误");
    expectNear(
        guidance.pitchKalmanConfig().measurement_angle_variance,
        std::pow(std::atan(2.0 / 220.0), 2),
        1e-12,
        "俯仰像素噪声换算错误");
}

void testGyroCompensation()
{
    GuidanceConfig config = makeGuidanceConfig();
    config.roll_compensation = false;
    ProportionalGuidance guidance(config);
    const double dt = 1.0 / 90.0;
    const double yaw_rate = 0.1;
    GuidanceResult result;
    for (int index = 0; index < 46; ++index)
    {
        const double angle = -yaw_rate * index * dt;
        result = guidance.update(
            guidance.cx() + guidance.fx() * std::tan(angle),
            guidance.cy(),
            dt,
            std::nullopt,
            0.0,
            cv::Vec3d(0.0, 0.0, yaw_rate));
    }
    expectNear(*result.raw_yaw_los_rate_rad_s, 0.0, 1e-4, "陀螺补偿错误");
    expectNear(result.yaw_los_rate_rad_s, 0.0, 1e-3, "偏航滤波错误");
}

void testPredictionAndInnovationGate()
{
    GuidanceConfig config = makeGuidanceConfig();
    config.position_to_rate_gain = 1.0;
    ProportionalGuidance guidance(config);
    guidance.update(
        guidance.cx() + guidance.fx() * std::tan(0.1),
        guidance.cy(),
        0.02);

    GuidanceResult predicted = guidance.predict(0.02);
    expectTrue(predicted.guidance_valid && predicted.predicted, "短时预测无效");
    for (int index = 0; index < 4; ++index)
    {
        predicted = guidance.predict(0.02);
    }
    expectTrue(predicted.guidance_valid, "预测提前失效");
    expectTrue(!guidance.predict(0.02).guidance_valid, "超时预测仍然有效");

    ProportionalGuidance gate_guidance(makeGuidanceConfig());
    gate_guidance.update(gate_guidance.cx(), gate_guidance.cy(), 0.02);
    const GuidanceResult reacquired = gate_guidance.update(
        gate_guidance.cx() + gate_guidance.fx() * 0.5,
        gate_guidance.cy(),
        0.02);
    expectTrue(reacquired.filter_reinitialized, "大创新量未重置滤波器");
}
}

int main()
{
    try
    {
        testKalmanFilter();
        testStaticCenterTarget();
        testBodyOverloadSigns();
        testCameraMatrixAndNoiseConversion();
        testGyroCompensation();
        testPredictionAndInnovationGate();
    }
    catch (const std::exception& error)
    {
        std::cerr << "测试失败：" << error.what() << '\n';
        return 1;
    }
    std::cout << "所有 C++ 核心回归测试通过。\n";
    return 0;
}
