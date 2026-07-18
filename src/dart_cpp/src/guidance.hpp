#pragma once

#include <opencv2/core.hpp>

#include <optional>
#include <utility>

inline constexpr double kGuidanceGravity = 9.80665;

// 单轴 LOS 卡尔曼滤波参数。
struct AxisKalmanConfig
{
    double angle_variance = 0.05;
    double rate_variance = 1.0;
    double process_accel_variance = 0.02;
    double measurement_angle_variance = 0.0025;
    std::optional<double> measurement_noise_px = 1.0;
    double innovation_gate_sigma = 3.0;
};

// 比例导航制导配置。
struct GuidanceConfig
{
    int image_width = 640;
    int image_height = 480;
    double fov_x_deg = 65.0;
    double fov_y_deg = 40.0;
    std::optional<double> cx;
    std::optional<double> cy;
    std::optional<double> fx;
    std::optional<double> fy;
    std::optional<cv::Matx33d> camera_matrix;

    // camera 坐标为右、下、前；body 坐标为前、右、下。
    cv::Matx33d R_bc = cv::Matx33d(
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0);

    double navigation_ratio = 3.0;
    std::optional<double> yaw_navigation_ratio;
    std::optional<double> pitch_navigation_ratio;
    double closing_velocity = 15.0;
    std::optional<double> yaw_closing_velocity;
    std::optional<double> pitch_closing_velocity;

    double position_to_rate_gain = 0.0;
    double yaw_angle_control_gain = 0.0;
    double pitch_angle_control_gain = 0.0;
    std::optional<double> rate_filter_alpha;
    bool use_kalman_filter = true;

    AxisKalmanConfig yaw_kalman;
    AxisKalmanConfig pitch_kalman;

    double max_overload_g = 0.5;
    std::optional<double> yaw_max_overload_g;
    std::optional<double> pitch_max_overload_g;
    bool roll_compensation = true;
    double roll_sign = 1.0;
    double yaw_max_slew_g_s = 0.0;
    double pitch_max_slew_g_s = 0.0;
    double max_prediction_time_s = 0.1;
};

// 制导计算结果。向量均为右手坐标系下的 [x, y, z] 分量。
struct GuidanceResult
{
    bool detected = false;
    bool predicted = false;
    bool guidance_valid = false;
    bool filter_reinitialized = false;

    double pixel_error_x = 0.0;
    double pixel_error_y = 0.0;
    cv::Vec3d los_c = cv::Vec3d(0.0, 0.0, 1.0);
    cv::Vec3d los_b = cv::Vec3d(1.0, 0.0, 0.0);
    cv::Vec3d los_s = cv::Vec3d(1.0, 0.0, 0.0);

    double yaw_los_angle_rad = 0.0;
    double pitch_los_angle_rad = 0.0;
    double yaw_los_rate_rad_s = 0.0;
    double pitch_los_rate_rad_s = 0.0;

    std::optional<double> raw_yaw_los_angle_rad;
    std::optional<double> raw_pitch_los_angle_rad;
    std::optional<double> raw_yaw_los_rate_rad_s;
    std::optional<double> raw_pitch_los_rate_rad_s;

    double relative_yaw_los_rate_rad_s = 0.0;
    double relative_pitch_los_rate_rad_s = 0.0;
    double gyro_yaw_los_rate_correction_rad_s = 0.0;
    double gyro_pitch_los_rate_correction_rad_s = 0.0;
    double prediction_age_s = 0.0;

    double yaw_rate_control_rad_s = 0.0;
    double pitch_rate_control_rad_s = 0.0;
    double yaw_angle_control_rad_s = 0.0;
    double pitch_angle_control_rad_s = 0.0;
    double yaw_command_rate_rad_s = 0.0;
    double pitch_command_rate_rad_s = 0.0;

    double stable_yaw_overload_g = 0.0;
    double stable_pitch_overload_g = 0.0;
    double yaw_overload_g = 0.0;
    double pitch_overload_g = 0.0;
    double body_y_overload_g = 0.0;
    double body_z_overload_g = 0.0;
};

class ProportionalGuidance
{
public:
    explicit ProportionalGuidance(const GuidanceConfig& config = GuidanceConfig());

    // 清除滤波器、视线差分和过载斜率限制器状态。
    void reset();

    // 输入检测到的像素坐标，返回当前制导结果。
    GuidanceResult update(
        double target_x,
        double target_y,
        double dt = 0.0,
        std::optional<double> roll_rad = std::nullopt,
        std::optional<double> roll_deg = 0.0,
        std::optional<cv::Vec3d> gyro_b = std::nullopt);

    // 视觉丢失时在限定时间内使用卡尔曼状态继续预测。
    GuidanceResult predict(
        double dt,
        std::optional<double> roll_rad = std::nullopt,
        std::optional<cv::Vec3d> gyro_b = std::nullopt);

    GuidanceResult predictKalman(
        double dt,
        std::optional<double> roll_rad = std::nullopt,
        std::optional<cv::Vec3d> gyro_b = std::nullopt);

    GuidanceResult lostResult() const;
    cv::Vec3d pixelToCameraLos(double target_x, double target_y) const;

    double cx() const;
    double cy() const;
    double fx() const;
    double fy() const;
    const AxisKalmanConfig& yawKalmanConfig() const;
    const AxisKalmanConfig& pitchKalmanConfig() const;

private:
    struct FilterState
    {
        double angle = 0.0;
        double rate = 0.0;
        bool reinitialized = false;
    };

    struct LosState
    {
        double yaw_angle = 0.0;
        double yaw_rate = 0.0;
        double pitch_angle = 0.0;
        double pitch_rate = 0.0;
        bool filter_reinitialized = false;
    };

    struct AxisFilter
    {
        cv::Vec2d state;
        cv::Matx22d covariance;
    };

    double cx_;
    double cy_;
    double fx_;
    double fy_;
    double camera_skew_ = 0.0;
    cv::Matx33d R_bc_;

    double yaw_navigation_ratio_;
    double pitch_navigation_ratio_;
    double yaw_closing_velocity_;
    double pitch_closing_velocity_;
    double position_to_rate_gain_;
    double yaw_angle_control_gain_;
    double pitch_angle_control_gain_;
    std::optional<double> rate_filter_alpha_;
    bool use_kalman_filter_;
    double yaw_max_overload_g_;
    double pitch_max_overload_g_;
    bool roll_compensation_;
    double roll_sign_;
    double yaw_max_slew_g_s_;
    double pitch_max_slew_g_s_;
    double max_prediction_time_s_;

    AxisKalmanConfig yaw_kalman_;
    AxisKalmanConfig pitch_kalman_;

    std::optional<cv::Vec3d> last_los_s_;
    std::optional<AxisFilter> yaw_filter_;
    std::optional<AxisFilter> pitch_filter_;
    double filtered_yaw_dot_ = 0.0;
    double filtered_pitch_dot_ = 0.0;
    bool has_filtered_rate_ = false;
    double last_roll_rad_ = 0.0;
    std::optional<double> last_yaw_overload_g_;
    std::optional<double> last_pitch_overload_g_;
    double prediction_age_s_ = 0.0;

    static bool isFinite(double value);
    static double positiveOrZero(double value);
    static double positiveValue(double value, double fallback);
    static double axisOrDefault(
        const std::optional<double>& axis_value,
        double default_value);
    static double focalLength(double pixels, double fov_deg);
    static cv::Vec3d normalize(const cv::Vec3d& vector);
    static cv::Vec3d gyroOrZero(const std::optional<cv::Vec3d>& gyro_b);
    static double cleanDt(double dt);
    static double clamp(double value, double low, double high);

    void initializeCamera(const GuidanceConfig& config);
    static void validateCameraMatrix(const cv::Matx33d& camera_matrix);
    static void validateFiniteMatrix(
        const cv::Matx33d& matrix,
        const char* name);
    void resolveKalmanConfig(AxisKalmanConfig& config, double focal_length);

    double effectiveRoll(const std::optional<double>& roll_rad);
    cv::Matx33d rollCompensationMatrix(double roll_rad) const;
    std::pair<double, double> gyroRateCorrection(
        const cv::Vec3d& los_s,
        const cv::Vec3d& gyro_b,
        const cv::Matx33d& roll_matrix) const;
    std::pair<double, double> allocateToBody(
        double stable_yaw_g,
        double stable_pitch_g,
        double roll_rad) const;
    std::pair<double, double> losAngles(const cv::Vec3d& los_s) const;
    cv::Vec3d losFromAngles(double yaw_angle, double pitch_angle) const;
    std::pair<double, double> losAngleRates(
        const cv::Vec3d& los_s,
        const cv::Vec3d& los_dot_s) const;
    std::pair<cv::Vec3d, bool> losDotStable(
        const cv::Vec3d& los_s,
        double dt);

    FilterState updateAxisFilter(
        std::optional<AxisFilter>& axis_filter,
        double angle,
        double dt,
        const AxisKalmanConfig& config,
        double gyro_rate_correction);
    cv::Vec2d predictAxisFilter(
        AxisFilter& axis_filter,
        double dt,
        const AxisKalmanConfig& config,
        double gyro_rate_correction);
    AxisFilter createAxisFilter(
        double angle,
        const AxisKalmanConfig& config) const;
    cv::Matx22d processNoise(
        double dt,
        const AxisKalmanConfig& config) const;
    LosState filterLosStates(
        double yaw_angle,
        double yaw_dot,
        double pitch_angle,
        double pitch_dot,
        double dt,
        double gyro_yaw_dot,
        double gyro_pitch_dot);
    std::pair<double, double> filterLosRates(
        double yaw_dot,
        double pitch_dot);

    std::pair<double, double> applySlewLimit(
        double yaw_g,
        double pitch_g,
        double dt);
    static double slew(
        double value,
        const std::optional<double>& previous,
        double max_slew,
        double dt);
    static double limitOverload(double overload_g, double max_overload_g);

    GuidanceResult finalizeResult(
        GuidanceResult result,
        double roll_rad,
        double dt);
};
