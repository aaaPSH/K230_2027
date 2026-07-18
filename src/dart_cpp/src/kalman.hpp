#pragma once

#include <opencv2/core.hpp>

// 通用线性卡尔曼滤波器。
// 模型：x(k) = A * x(k-1) + B * u(k)；z(k) = H * x(k)。
// 所有向量都使用 n 行 1 列的 cv::Mat，矩阵内部统一保存为 CV_64F。
class KalmanFilter
{
public:
    KalmanFilter(
        const cv::Mat& state,
        const cv::Mat& covariance,
        const cv::Mat& transition_matrix,
        const cv::Mat& measurement_matrix,
        const cv::Mat& process_noise,
        const cv::Mat& measurement_noise,
        const cv::Mat& control_matrix = cv::Mat());

    // 重置状态；不传 covariance 时保留当前协方差。
    void reset(const cv::Mat& state, const cv::Mat& covariance = cv::Mat());

    // 执行一步预测，可临时覆盖状态转移矩阵、过程噪声和控制量。
    cv::Mat predict(
        const cv::Mat& control = cv::Mat(),
        const cv::Mat& transition_matrix = cv::Mat(),
        const cv::Mat& process_noise = cv::Mat());

    // 使用测量值修正当前状态。
    cv::Mat update(
        const cv::Mat& measurement,
        const cv::Mat& measurement_matrix = cv::Mat(),
        const cv::Mat& measurement_noise = cv::Mat());

    // 先预测，再更新。
    cv::Mat step(
        const cv::Mat& measurement,
        const cv::Mat& control = cv::Mat(),
        const cv::Mat& transition_matrix = cv::Mat(),
        const cv::Mat& process_noise = cv::Mat(),
        const cv::Mat& measurement_matrix = cv::Mat(),
        const cv::Mat& measurement_noise = cv::Mat());

    int stateSize() const;
    int measurementSize() const;
    cv::Mat state() const;
    cv::Mat covariance() const;

    void setTransitionMatrix(const cv::Mat& transition_matrix);
    void setProcessNoise(const cv::Mat& process_noise);
    void setMeasurementNoise(const cv::Mat& measurement_noise);

private:
    cv::Mat state_;
    cv::Mat covariance_;
    cv::Mat transition_matrix_;
    cv::Mat measurement_matrix_;
    cv::Mat process_noise_;
    cv::Mat measurement_noise_;
    cv::Mat control_matrix_;
    static cv::Mat toDouble(const cv::Mat& matrix);
    static void validateState(const cv::Mat& vector, const char* name);
    static void validateSquare(
        const cv::Mat& matrix,
        int size,
        const char* name);

    void validateModel() const;
    void symmetrizeCovariance();
};

// 创建一维匀速模型，状态为 [位置, 速度]。
KalmanFilter makeConstantVelocityFilter(
    double position,
    double velocity = 0.0,
    double dt = 1.0,
    double position_variance = 100.0,
    double velocity_variance = 100.0,
    double process_variance = 0.01,
    double measurement_variance = 1.0);
