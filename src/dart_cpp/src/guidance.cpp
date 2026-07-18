#include "guidance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace
{
constexpr double kEps = 1e-9;
}

ProportionalGuidance::ProportionalGuidance(const GuidanceConfig& config)
    : R_bc_(config.R_bc),
      yaw_navigation_ratio_(axisOrDefault(
          config.yaw_navigation_ratio,
          config.navigation_ratio)),
      pitch_navigation_ratio_(axisOrDefault(
          config.pitch_navigation_ratio,
          config.navigation_ratio)),
      yaw_closing_velocity_(axisOrDefault(
          config.yaw_closing_velocity,
          config.closing_velocity)),
      pitch_closing_velocity_(axisOrDefault(
          config.pitch_closing_velocity,
          config.closing_velocity)),
      position_to_rate_gain_(config.position_to_rate_gain),
      yaw_angle_control_gain_(config.yaw_angle_control_gain),
      pitch_angle_control_gain_(config.pitch_angle_control_gain),
      use_kalman_filter_(config.use_kalman_filter),
      yaw_max_overload_g_(axisOrDefault(
          config.yaw_max_overload_g,
          config.max_overload_g)),
      pitch_max_overload_g_(axisOrDefault(
          config.pitch_max_overload_g,
          config.max_overload_g)),
      roll_compensation_(config.roll_compensation),
      roll_sign_(config.roll_sign),
      yaw_max_slew_g_s_(positiveOrZero(config.yaw_max_slew_g_s)),
      pitch_max_slew_g_s_(positiveOrZero(config.pitch_max_slew_g_s)),
      max_prediction_time_s_(positiveOrZero(config.max_prediction_time_s)),
      yaw_kalman_(config.yaw_kalman),
      pitch_kalman_(config.pitch_kalman)
{
    initializeCamera(config);

    if (!isFinite(roll_sign_) ||
        std::abs(std::abs(roll_sign_) - 1.0) > kEps)
    {
        throw std::invalid_argument("roll_sign must be +1.0 or -1.0.");
    }

    if (config.rate_filter_alpha.has_value() &&
        isFinite(*config.rate_filter_alpha) &&
        *config.rate_filter_alpha >= 0.0)
    {
        rate_filter_alpha_ = clamp(*config.rate_filter_alpha, 0.0, 1.0);
    }

    resolveKalmanConfig(yaw_kalman_, fx_);
    resolveKalmanConfig(pitch_kalman_, fy_);
}

void ProportionalGuidance::reset()
{
    last_los_s_.reset();
    filtered_yaw_dot_ = 0.0;
    filtered_pitch_dot_ = 0.0;
    has_filtered_rate_ = false;
    yaw_filter_.reset();
    pitch_filter_.reset();
    last_yaw_overload_g_.reset();
    last_pitch_overload_g_.reset();
    prediction_age_s_ = 0.0;
}

GuidanceResult ProportionalGuidance::update(
    double target_x,
    double target_y,
    double dt,
    std::optional<double> roll_rad,
    std::optional<double> roll_deg,
    std::optional<cv::Vec3d> gyro_b)
{
    if (!isFinite(target_x) || !isFinite(target_y) ||
        target_x < 0.0 || target_y < 0.0)
    {
        throw std::invalid_argument(
            "target coordinates must be finite non-negative values.");
    }

    const double clean_dt = cleanDt(dt);
    if (!roll_rad.has_value() && roll_deg.has_value())
    {
        if (!isFinite(*roll_deg))
        {
            throw std::invalid_argument("roll_deg must be finite when supplied.");
        }
        roll_rad = *roll_deg * CV_PI / 180.0;
    }

    const double roll = effectiveRoll(roll_rad);
    const cv::Vec3d gyro = gyroOrZero(gyro_b);

    prediction_age_s_ = 0.0;
    const bool filters_were_initialized =
        yaw_filter_.has_value() && pitch_filter_.has_value();

    const cv::Vec3d los_c = pixelToCameraLos(target_x, target_y);
    const cv::Vec3d los_b = normalize(R_bc_ * los_c);
    const cv::Matx33d roll_matrix = rollCompensationMatrix(roll);
    const cv::Vec3d los_s = normalize(roll_matrix * los_b);
    const auto raw_angles = losAngles(los_s);

    const auto stable_dot = losDotStable(los_s, clean_dt);
    const auto raw_relative_rates = losAngleRates(los_s, stable_dot.first);
    auto gyro_rates = gyroRateCorrection(los_s, gyro, roll_matrix);
    if (!stable_dot.second && !filters_were_initialized)
    {
        gyro_rates = {0.0, 0.0};
    }

    const double raw_yaw_dot = raw_relative_rates.first + gyro_rates.first;
    const double raw_pitch_dot = raw_relative_rates.second + gyro_rates.second;

    const LosState filtered = filterLosStates(
        raw_angles.first,
        raw_relative_rates.first,
        raw_angles.second,
        raw_relative_rates.second,
        clean_dt,
        gyro_rates.first,
        gyro_rates.second);

    if (filtered.filter_reinitialized)
    {
        gyro_rates = {0.0, 0.0};
    }

    GuidanceResult result;
    result.detected = true;
    result.filter_reinitialized = filtered.filter_reinitialized;
    result.pixel_error_x = target_x - cx_;
    result.pixel_error_y = target_y - cy_;
    result.los_c = los_c;
    result.los_b = los_b;
    result.los_s = los_s;
    result.yaw_los_angle_rad = filtered.yaw_angle;
    result.pitch_los_angle_rad = filtered.pitch_angle;
    result.yaw_los_rate_rad_s = filtered.yaw_rate;
    result.pitch_los_rate_rad_s = filtered.pitch_rate;
    result.raw_yaw_los_angle_rad = raw_angles.first;
    result.raw_pitch_los_angle_rad = raw_angles.second;
    result.raw_yaw_los_rate_rad_s = raw_yaw_dot;
    result.raw_pitch_los_rate_rad_s = raw_pitch_dot;
    result.relative_yaw_los_rate_rad_s =
        filtered.yaw_rate - gyro_rates.first;
    result.relative_pitch_los_rate_rad_s =
        filtered.pitch_rate - gyro_rates.second;
    result.gyro_yaw_los_rate_correction_rad_s = gyro_rates.first;
    result.gyro_pitch_los_rate_correction_rad_s = gyro_rates.second;
    return finalizeResult(std::move(result), roll, clean_dt);
}

GuidanceResult ProportionalGuidance::predict(
    double dt,
    std::optional<double> roll_rad,
    std::optional<cv::Vec3d> gyro_b)
{
    last_los_s_.reset();
    has_filtered_rate_ = false;

    const double clean_dt = cleanDt(dt);
    const double roll = effectiveRoll(roll_rad);
    const cv::Vec3d gyro = gyroOrZero(gyro_b);

    if (!yaw_filter_.has_value() || !pitch_filter_.has_value())
    {
        return lostResult();
    }

    prediction_age_s_ += clean_dt;
    if (max_prediction_time_s_ <= 0.0 ||
        prediction_age_s_ > max_prediction_time_s_ + kEps)
    {
        reset();
        return lostResult();
    }

    const double current_yaw_angle = yaw_filter_->state[0];
    const double current_pitch_angle = pitch_filter_->state[0];
    const cv::Vec3d los_s = losFromAngles(
        current_yaw_angle,
        current_pitch_angle);
    const cv::Matx33d roll_matrix = rollCompensationMatrix(roll);
    const auto gyro_rates = gyroRateCorrection(los_s, gyro, roll_matrix);

    const cv::Vec2d yaw_prediction = predictAxisFilter(
        *yaw_filter_,
        clean_dt,
        yaw_kalman_,
        gyro_rates.first);
    const cv::Vec2d pitch_prediction = predictAxisFilter(
        *pitch_filter_,
        clean_dt,
        pitch_kalman_,
        gyro_rates.second);

    const double yaw_angle = yaw_prediction[0];
    const double yaw_dot = yaw_prediction[1];
    const double pitch_angle = pitch_prediction[0];
    const double pitch_dot = pitch_prediction[1];
    const cv::Vec3d predicted_los_s = losFromAngles(yaw_angle, pitch_angle);

    GuidanceResult result;
    result.predicted = true;
    result.los_s = predicted_los_s;
    result.yaw_los_angle_rad = yaw_angle;
    result.pitch_los_angle_rad = pitch_angle;
    result.yaw_los_rate_rad_s = yaw_dot;
    result.pitch_los_rate_rad_s = pitch_dot;
    result.relative_yaw_los_rate_rad_s = yaw_dot - gyro_rates.first;
    result.relative_pitch_los_rate_rad_s = pitch_dot - gyro_rates.second;
    result.gyro_yaw_los_rate_correction_rad_s = gyro_rates.first;
    result.gyro_pitch_los_rate_correction_rad_s = gyro_rates.second;
    return finalizeResult(std::move(result), roll, clean_dt);
}

GuidanceResult ProportionalGuidance::predictKalman(
    double dt,
    std::optional<double> roll_rad,
    std::optional<cv::Vec3d> gyro_b)
{
    return predict(dt, roll_rad, gyro_b);
}

GuidanceResult ProportionalGuidance::lostResult() const
{
    return {};
}

cv::Vec3d ProportionalGuidance::pixelToCameraLos(
    double target_x,
    double target_y) const
{
    if (!isFinite(target_x) || !isFinite(target_y))
    {
        throw std::invalid_argument("pixel coordinates must be finite.");
    }

    const double y_normalized = (target_y - cy_) / fy_;
    const double x_normalized =
        (target_x - cx_ - camera_skew_ * y_normalized) / fx_;
    return normalize(cv::Vec3d(x_normalized, y_normalized, 1.0));
}

double ProportionalGuidance::cx() const
{
    return cx_;
}

double ProportionalGuidance::cy() const
{
    return cy_;
}

double ProportionalGuidance::fx() const
{
    return fx_;
}

double ProportionalGuidance::fy() const
{
    return fy_;
}

const AxisKalmanConfig& ProportionalGuidance::yawKalmanConfig() const
{
    return yaw_kalman_;
}

const AxisKalmanConfig& ProportionalGuidance::pitchKalmanConfig() const
{
    return pitch_kalman_;
}

bool ProportionalGuidance::isFinite(double value)
{
    return std::isfinite(value);
}

double ProportionalGuidance::positiveOrZero(double value)
{
    return isFinite(value) && value > 0.0 ? value : 0.0;
}

double ProportionalGuidance::positiveValue(double value, double fallback)
{
    return isFinite(value) && value > kEps ? value : fallback;
}

double ProportionalGuidance::axisOrDefault(
    const std::optional<double>& axis_value,
    double default_value)
{
    return axis_value.has_value() ? *axis_value : default_value;
}

double ProportionalGuidance::focalLength(double pixels, double fov_deg)
{
    if (!isFinite(pixels) || pixels <= 0.0 ||
        !isFinite(fov_deg) || fov_deg <= 0.0 || fov_deg >= 180.0)
    {
        throw std::invalid_argument(
            "image size and field of view must be valid positive values.");
    }
    return (pixels * 0.5) /
        std::tan(fov_deg * CV_PI / 360.0);
}

cv::Vec3d ProportionalGuidance::normalize(const cv::Vec3d& vector)
{
    const double norm = std::sqrt(vector.dot(vector));
    if (!isFinite(norm) || norm < kEps)
    {
        throw std::invalid_argument("LOS vector norm is zero.");
    }
    return vector * (1.0 / norm);
}

cv::Vec3d ProportionalGuidance::gyroOrZero(
    const std::optional<cv::Vec3d>& gyro_b)
{
    const cv::Vec3d gyro = gyro_b.value_or(cv::Vec3d(0.0, 0.0, 0.0));
    for (int index = 0; index < 3; ++index)
    {
        if (!isFinite(gyro[index]))
        {
            throw std::invalid_argument(
                "gyro_b must contain three finite values when supplied.");
        }
    }
    return gyro;
}

double ProportionalGuidance::cleanDt(double dt)
{
    return isFinite(dt) && dt > 0.0 ? dt : 0.0;
}

double ProportionalGuidance::clamp(
    double value,
    double low,
    double high)
{
    return std::max(low, std::min(high, value));
}

void ProportionalGuidance::initializeCamera(const GuidanceConfig& config)
{
    validateFiniteMatrix(R_bc_, "R_bc");

    if (config.camera_matrix.has_value())
    {
        const cv::Matx33d& camera_matrix = *config.camera_matrix;
        validateCameraMatrix(camera_matrix);
        fx_ = camera_matrix(0, 0);
        fy_ = camera_matrix(1, 1);
        cx_ = camera_matrix(0, 2);
        cy_ = camera_matrix(1, 2);
        camera_skew_ = camera_matrix(0, 1);
    }
    else
    {
        cx_ = config.cx.value_or(config.image_width * 0.5);
        cy_ = config.cy.value_or(config.image_height * 0.5);
        fx_ = config.fx.has_value()
            ? *config.fx
            : focalLength(config.image_width, config.fov_x_deg);
        fy_ = config.fy.has_value()
            ? *config.fy
            : focalLength(config.image_height, config.fov_y_deg);
        camera_skew_ = 0.0;
    }

    if (!isFinite(fx_) || !isFinite(fy_) || fx_ <= kEps || fy_ <= kEps)
    {
        throw std::invalid_argument("camera focal length must be positive.");
    }
}

void ProportionalGuidance::validateCameraMatrix(
    const cv::Matx33d& camera_matrix)
{
    validateFiniteMatrix(camera_matrix, "camera_matrix");

    if (camera_matrix(0, 0) <= kEps || camera_matrix(1, 1) <= kEps ||
        std::abs(camera_matrix(1, 0)) > kEps ||
        std::abs(camera_matrix(2, 0)) > kEps ||
        std::abs(camera_matrix(2, 1)) > kEps ||
        std::abs(camera_matrix(2, 2) - 1.0) > kEps)
    {
        throw std::invalid_argument(
            "camera_matrix must use the standard pinhole form.");
    }
}

void ProportionalGuidance::validateFiniteMatrix(
    const cv::Matx33d& matrix,
    const char* name)
{
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            if (!isFinite(matrix(row, col)))
            {
                throw std::invalid_argument(
                    std::string(name) + " must contain finite values.");
            }
        }
    }
}

void ProportionalGuidance::resolveKalmanConfig(
    AxisKalmanConfig& config,
    double focal_length)
{
    config.angle_variance = positiveValue(config.angle_variance, 0.05);
    config.rate_variance = positiveValue(config.rate_variance, 1.0);
    config.process_accel_variance = positiveOrZero(
        config.process_accel_variance);
    config.innovation_gate_sigma = positiveValue(
        config.innovation_gate_sigma,
        3.0);

    if (config.measurement_noise_px.has_value())
    {
        const double noise_px = positiveValue(
            *config.measurement_noise_px,
            1.0);
        const double sigma_rad = std::atan(noise_px / focal_length);
        config.measurement_angle_variance = sigma_rad * sigma_rad;
    }
    else
    {
        config.measurement_angle_variance = positiveValue(
            config.measurement_angle_variance,
            0.0025);
    }
}

double ProportionalGuidance::effectiveRoll(
    const std::optional<double>& roll_rad)
{
    if (!roll_compensation_)
    {
        last_roll_rad_ = 0.0;
        return 0.0;
    }

    if (roll_rad.has_value())
    {
        if (!isFinite(*roll_rad))
        {
            throw std::invalid_argument(
                "roll_rad must be finite when supplied.");
        }
        last_roll_rad_ = *roll_rad * roll_sign_;
    }
    return last_roll_rad_;
}

cv::Matx33d ProportionalGuidance::rollCompensationMatrix(double roll_rad) const
{
    const double cos_roll = std::cos(roll_rad);
    const double sin_roll = std::sin(roll_rad);
    return cv::Matx33d(
        1.0, 0.0, 0.0,
        0.0, cos_roll, sin_roll,
        0.0, -sin_roll, cos_roll);
}

std::pair<double, double> ProportionalGuidance::gyroRateCorrection(
    const cv::Vec3d& los_s,
    const cv::Vec3d& gyro_b,
    const cv::Matx33d& roll_matrix) const
{
    cv::Vec3d gyro_s = roll_matrix * gyro_b;
    gyro_s[0] = 0.0;
    return losAngleRates(los_s, gyro_s.cross(los_s));
}

std::pair<double, double> ProportionalGuidance::allocateToBody(
    double stable_yaw_g,
    double stable_pitch_g,
    double roll_rad) const
{
    const cv::Matx33d roll_matrix = rollCompensationMatrix(roll_rad);
    const cv::Vec3d stable_command(0.0, stable_yaw_g, stable_pitch_g);
    const cv::Vec3d body_command = roll_matrix.t() * stable_command;
    return {body_command[1], body_command[2]};
}

std::pair<double, double> ProportionalGuidance::losAngles(
    const cv::Vec3d& los_s) const
{
    const double rho = std::sqrt(los_s[0] * los_s[0] + los_s[1] * los_s[1]);
    return {
        std::atan2(los_s[1], los_s[0]),
        std::atan2(-los_s[2], rho)};
}

cv::Vec3d ProportionalGuidance::losFromAngles(
    double yaw_angle,
    double pitch_angle) const
{
    const double cos_pitch = std::cos(pitch_angle);
    return cv::Vec3d(
        cos_pitch * std::cos(yaw_angle),
        cos_pitch * std::sin(yaw_angle),
        -std::sin(pitch_angle));
}

std::pair<double, double> ProportionalGuidance::losAngleRates(
    const cv::Vec3d& los_s,
    const cv::Vec3d& los_dot_s) const
{
    const double x = los_s[0];
    const double y = los_s[1];
    const double z = los_s[2];
    const double xd = los_dot_s[0];
    const double yd = los_dot_s[1];
    const double zd = los_dot_s[2];
    const double rho2 = x * x + y * y;
    if (rho2 < kEps)
    {
        return {0.0, 0.0};
    }

    const double rho = std::sqrt(rho2);
    return {
        (x * yd - y * xd) / rho2,
        -rho * zd + z * (x * xd + y * yd) / rho};
}

std::pair<cv::Vec3d, bool> ProportionalGuidance::losDotStable(
    const cv::Vec3d& los_s,
    double dt)
{
    if (!last_los_s_.has_value() || dt <= 0.0)
    {
        last_los_s_ = los_s;
        return {cv::Vec3d(0.0, 0.0, 0.0), false};
    }

    const cv::Vec3d result = (los_s - *last_los_s_) * (1.0 / dt);
    last_los_s_ = los_s;
    return {result, true};
}

ProportionalGuidance::FilterState ProportionalGuidance::updateAxisFilter(
    std::optional<AxisFilter>& axis_filter,
    double angle,
    double dt,
    const AxisKalmanConfig& config,
    double gyro_rate_correction)
{
    if (!axis_filter.has_value())
    {
        axis_filter = createAxisFilter(angle, config);
        return {angle, 0.0, false};
    }

    predictAxisFilter(*axis_filter, dt, config, gyro_rate_correction);
    const double innovation_variance =
        axis_filter->covariance(0, 0) + config.measurement_angle_variance;
    const double residual = angle - axis_filter->state[0];
    const double gate = config.innovation_gate_sigma;

    if (innovation_variance <= kEps ||
        residual * residual > gate * gate * innovation_variance)
    {
        axis_filter = createAxisFilter(angle, config);
        return {angle, 0.0, true};
    }

    const double measurement_variance = config.measurement_angle_variance;
    const cv::Matx22d covariance = axis_filter->covariance;
    const double gain_angle = covariance(0, 0) / innovation_variance;
    const double gain_rate = covariance(1, 0) / innovation_variance;

    axis_filter->state[0] += gain_angle * residual;
    axis_filter->state[1] += gain_rate * residual;

    // 使用固定尺寸 Joseph 形式更新协方差，保持数值稳定性。
    const double angle_factor = 1.0 - gain_angle;
    const double p00 =
        angle_factor * angle_factor * covariance(0, 0) +
        gain_angle * gain_angle * measurement_variance;
    const double p01 =
        angle_factor *
            (covariance(0, 1) - gain_rate * covariance(0, 0)) +
        gain_angle * gain_rate * measurement_variance;
    const double p10 =
        angle_factor *
            (covariance(1, 0) - gain_rate * covariance(0, 0)) +
        gain_angle * gain_rate * measurement_variance;
    const double p11 =
        covariance(1, 1) -
        gain_rate * (covariance(0, 1) + covariance(1, 0)) +
        gain_rate * gain_rate *
            (covariance(0, 0) + measurement_variance);
    const double off_diagonal = 0.5 * (p01 + p10);
    axis_filter->covariance = cv::Matx22d(
        p00, off_diagonal,
        off_diagonal, p11);

    return {axis_filter->state[0], axis_filter->state[1], false};
}

cv::Vec2d ProportionalGuidance::predictAxisFilter(
    AxisFilter& axis_filter,
    double dt,
    const AxisKalmanConfig& config,
    double gyro_rate_correction)
{
    axis_filter.state[0] +=
        dt * (axis_filter.state[1] - gyro_rate_correction);

    const cv::Matx22d covariance = axis_filter.covariance;
    const cv::Matx22d process = processNoise(dt, config);
    const double dt2 = dt * dt;
    const double p01 = covariance(0, 1) + dt * covariance(1, 1);
    const double p10 = covariance(1, 0) + dt * covariance(1, 1);
    const double off_diagonal = 0.5 * (p01 + p10) + process(0, 1);
    axis_filter.covariance = cv::Matx22d(
        covariance(0, 0) +
            dt * (covariance(0, 1) + covariance(1, 0)) +
            dt2 * covariance(1, 1) + process(0, 0),
        off_diagonal,
        off_diagonal,
        covariance(1, 1) + process(1, 1));
    return axis_filter.state;
}

ProportionalGuidance::AxisFilter ProportionalGuidance::createAxisFilter(
    double angle,
    const AxisKalmanConfig& config) const
{
    return {
        cv::Vec2d(angle, 0.0),
        cv::Matx22d(
            config.angle_variance, 0.0,
            0.0, config.rate_variance)};
}

cv::Matx22d ProportionalGuidance::processNoise(
    double dt,
    const AxisKalmanConfig& config) const
{
    const double q = positiveOrZero(config.process_accel_variance);
    if (dt <= 0.0 || q <= 0.0)
    {
        return cv::Matx22d::zeros();
    }

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    return cv::Matx22d(
        q * dt3 / 3.0, q * dt2 / 2.0,
        q * dt2 / 2.0, q * dt);
}

ProportionalGuidance::LosState ProportionalGuidance::filterLosStates(
    double yaw_angle,
    double yaw_dot,
    double pitch_angle,
    double pitch_dot,
    double dt,
    double gyro_yaw_dot,
    double gyro_pitch_dot)
{
    if (!use_kalman_filter_)
    {
        const auto filtered_rates = filterLosRates(yaw_dot, pitch_dot);
        return {
            yaw_angle,
            filtered_rates.first + gyro_yaw_dot,
            pitch_angle,
            filtered_rates.second + gyro_pitch_dot,
            false};
    }

    const FilterState yaw_state = updateAxisFilter(
        yaw_filter_,
        yaw_angle,
        dt,
        yaw_kalman_,
        gyro_yaw_dot);
    const FilterState pitch_state = updateAxisFilter(
        pitch_filter_,
        pitch_angle,
        dt,
        pitch_kalman_,
        gyro_pitch_dot);
    return {
        yaw_state.angle,
        yaw_state.rate,
        pitch_state.angle,
        pitch_state.rate,
        yaw_state.reinitialized || pitch_state.reinitialized};
}

std::pair<double, double> ProportionalGuidance::filterLosRates(
    double yaw_dot,
    double pitch_dot)
{
    if (!rate_filter_alpha_.has_value())
    {
        return {yaw_dot, pitch_dot};
    }

    const double alpha = *rate_filter_alpha_;
    if (!has_filtered_rate_)
    {
        filtered_yaw_dot_ = yaw_dot;
        filtered_pitch_dot_ = pitch_dot;
        has_filtered_rate_ = true;
        return {yaw_dot, pitch_dot};
    }

    filtered_yaw_dot_ =
        alpha * yaw_dot + (1.0 - alpha) * filtered_yaw_dot_;
    filtered_pitch_dot_ =
        alpha * pitch_dot + (1.0 - alpha) * filtered_pitch_dot_;
    return {filtered_yaw_dot_, filtered_pitch_dot_};
}

std::pair<double, double> ProportionalGuidance::applySlewLimit(
    double yaw_g,
    double pitch_g,
    double dt)
{
    yaw_g = slew(yaw_g, last_yaw_overload_g_, yaw_max_slew_g_s_, dt);
    pitch_g = slew(
        pitch_g,
        last_pitch_overload_g_,
        pitch_max_slew_g_s_,
        dt);
    last_yaw_overload_g_ = yaw_g;
    last_pitch_overload_g_ = pitch_g;
    return {yaw_g, pitch_g};
}

double ProportionalGuidance::slew(
    double value,
    const std::optional<double>& previous,
    double max_slew,
    double dt)
{
    if (!previous.has_value() || max_slew <= 0.0 || dt <= 0.0)
    {
        return value;
    }
    return clamp(
        value,
        *previous - max_slew * dt,
        *previous + max_slew * dt);
}

double ProportionalGuidance::limitOverload(
    double overload_g,
    double max_overload_g)
{
    if (max_overload_g <= 0.0)
    {
        return overload_g;
    }
    return clamp(overload_g, -max_overload_g, max_overload_g);
}

GuidanceResult ProportionalGuidance::finalizeResult(
    GuidanceResult result,
    double roll_rad,
    double dt)
{
    const double yaw_angle_gain =
        position_to_rate_gain_ + yaw_angle_control_gain_;
    const double pitch_angle_gain =
        position_to_rate_gain_ + pitch_angle_control_gain_;
    const double yaw_angle_control =
        yaw_angle_gain * result.yaw_los_angle_rad;
    const double pitch_angle_control =
        pitch_angle_gain * result.pitch_los_angle_rad;
    const double yaw_command_rate =
        result.yaw_los_rate_rad_s + yaw_angle_control;
    const double pitch_command_rate =
        result.pitch_los_rate_rad_s + pitch_angle_control;

    const double stable_yaw_g =
        yaw_navigation_ratio_ * yaw_closing_velocity_ *
        yaw_command_rate / kGuidanceGravity;
    const double stable_pitch_g =
        -pitch_navigation_ratio_ * pitch_closing_velocity_ *
        pitch_command_rate / kGuidanceGravity;
    auto body_overload = allocateToBody(
        stable_yaw_g,
        stable_pitch_g,
        roll_rad);
    double yaw_overload_g = limitOverload(
        body_overload.first,
        yaw_max_overload_g_);
    double pitch_overload_g = limitOverload(
        body_overload.second,
        pitch_max_overload_g_);
    const auto slew_limited = applySlewLimit(
        yaw_overload_g,
        pitch_overload_g,
        dt);
    yaw_overload_g = slew_limited.first;
    pitch_overload_g = slew_limited.second;

    result.guidance_valid = true;
    result.prediction_age_s = prediction_age_s_;
    result.yaw_rate_control_rad_s = result.yaw_los_rate_rad_s;
    result.pitch_rate_control_rad_s = result.pitch_los_rate_rad_s;
    result.yaw_angle_control_rad_s = yaw_angle_control;
    result.pitch_angle_control_rad_s = pitch_angle_control;
    result.yaw_command_rate_rad_s = yaw_command_rate;
    result.pitch_command_rate_rad_s = pitch_command_rate;
    result.stable_yaw_overload_g = stable_yaw_g;
    result.stable_pitch_overload_g = stable_pitch_g;
    result.yaw_overload_g = yaw_overload_g;
    result.pitch_overload_g = pitch_overload_g;
    result.body_y_overload_g = yaw_overload_g;
    result.body_z_overload_g = pitch_overload_g;
    return result;
}
