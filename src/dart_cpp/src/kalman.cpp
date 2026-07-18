#include "kalman.hpp"

#include <stdexcept>
#include <string>

KalmanFilter::KalmanFilter(
    const cv::Mat& state,
    const cv::Mat& covariance,
    const cv::Mat& transition_matrix,
    const cv::Mat& measurement_matrix,
    const cv::Mat& process_noise,
    const cv::Mat& measurement_noise,
    const cv::Mat& control_matrix)
{
    state_ = toDouble(state);
    covariance_ = toDouble(covariance);
    transition_matrix_ = toDouble(transition_matrix);
    measurement_matrix_ = toDouble(measurement_matrix);
    process_noise_ = toDouble(process_noise);
    measurement_noise_ = toDouble(measurement_noise);
    control_matrix_ = control_matrix.empty()
        ? cv::Mat()
        : toDouble(control_matrix);

    validateModel();
}

void KalmanFilter::reset(const cv::Mat& state, const cv::Mat& covariance)
{
    state_ = toDouble(state);
    if (!covariance.empty())
    {
        covariance_ = toDouble(covariance);
    }

    validateModel();
}

cv::Mat KalmanFilter::predict(
    const cv::Mat& control,
    const cv::Mat& transition_matrix,
    const cv::Mat& process_noise)
{
    const cv::Mat transition = transition_matrix.empty()
        ? transition_matrix_
        : toDouble(transition_matrix);
    const cv::Mat process = process_noise.empty()
        ? process_noise_
        : toDouble(process_noise);

    validateSquare(transition, state_.rows, "transition_matrix");
    validateSquare(process, state_.rows, "process_noise");

    state_ = transition * state_;
    if (!control.empty())
    {
        if (control_matrix_.empty())
        {
            throw std::invalid_argument(
                "control_matrix is required when control is provided.");
        }

        const cv::Mat control_vector = toDouble(control);
        validateState(control_vector, "control");
        if (control_vector.rows != control_matrix_.cols)
        {
            throw std::invalid_argument(
                "control size does not match control_matrix.");
        }
        state_ += control_matrix_ * control_vector;
    }

    covariance_ = transition * covariance_ * transition.t() + process;
    symmetrizeCovariance();
    return state_.clone();
}

cv::Mat KalmanFilter::update(
    const cv::Mat& measurement,
    const cv::Mat& measurement_matrix,
    const cv::Mat& measurement_noise)
{
    const cv::Mat measurement_vector = toDouble(measurement);
    validateState(measurement_vector, "measurement");

    const cv::Mat measurement_model = measurement_matrix.empty()
        ? measurement_matrix_
        : toDouble(measurement_matrix);
    const cv::Mat measurement_covariance = measurement_noise.empty()
        ? measurement_noise_
        : toDouble(measurement_noise);

    const int measurement_size = measurement_vector.rows;
    if (measurement_model.rows != measurement_size ||
        measurement_model.cols != state_.rows)
    {
        throw std::invalid_argument(
            "measurement_matrix shape does not match measurement and state.");
    }
    validateSquare(
        measurement_covariance,
        measurement_size,
        "measurement_noise");

    const cv::Mat residual =
        measurement_vector - measurement_model * state_;
    const cv::Mat innovation_covariance =
        measurement_model * covariance_ * measurement_model.t() +
        measurement_covariance;
    const cv::Mat state_measurement_covariance =
        covariance_ * measurement_model.t();

    // 求解 S * X = (P * H^T)^T，避免显式计算 S 的逆矩阵。
    cv::Mat solved_gain;
    const cv::Mat gain_rhs = state_measurement_covariance.t();
    bool solved = cv::solve(
        innovation_covariance,
        gain_rhs,
        solved_gain,
        cv::DECOMP_CHOLESKY);
    if (!solved)
    {
        solved = cv::solve(
            innovation_covariance,
            gain_rhs,
            solved_gain,
            cv::DECOMP_SVD);
    }
    if (!solved)
    {
        throw std::runtime_error("Failed to solve Kalman innovation covariance.");
    }
    const cv::Mat kalman_gain = solved_gain.t();

    state_ += kalman_gain * residual;

    // Joseph 形式有助于保持协方差矩阵的对称性和半正定性。
    const cv::Mat identity = cv::Mat::eye(state_.rows, state_.rows, CV_64F);
    const cv::Mat identity_minus_gain_model =
        identity - kalman_gain * measurement_model;
    covariance_ =
        identity_minus_gain_model * covariance_ *
            identity_minus_gain_model.t() +
        kalman_gain * measurement_covariance * kalman_gain.t();
    symmetrizeCovariance();
    return state_.clone();
}

cv::Mat KalmanFilter::step(
    const cv::Mat& measurement,
    const cv::Mat& control,
    const cv::Mat& transition_matrix,
    const cv::Mat& process_noise,
    const cv::Mat& measurement_matrix,
    const cv::Mat& measurement_noise)
{
    predict(control, transition_matrix, process_noise);
    return update(measurement, measurement_matrix, measurement_noise);
}

int KalmanFilter::stateSize() const
{
    return state_.rows;
}

int KalmanFilter::measurementSize() const
{
    return measurement_matrix_.rows;
}

cv::Mat KalmanFilter::state() const
{
    return state_.clone();
}

cv::Mat KalmanFilter::covariance() const
{
    return covariance_.clone();
}

void KalmanFilter::setTransitionMatrix(const cv::Mat& transition_matrix)
{
    const cv::Mat matrix = toDouble(transition_matrix);
    validateSquare(matrix, state_.rows, "transition_matrix");
    transition_matrix_ = matrix;
}

void KalmanFilter::setProcessNoise(const cv::Mat& process_noise)
{
    const cv::Mat matrix = toDouble(process_noise);
    validateSquare(matrix, state_.rows, "process_noise");
    process_noise_ = matrix;
}

void KalmanFilter::setMeasurementNoise(const cv::Mat& measurement_noise)
{
    const cv::Mat matrix = toDouble(measurement_noise);
    validateSquare(
        matrix,
        measurement_matrix_.rows,
        "measurement_noise");
    measurement_noise_ = matrix;
}

cv::Mat KalmanFilter::toDouble(const cv::Mat& matrix)
{
    if (matrix.empty())
    {
        return cv::Mat();
    }

    cv::Mat result;
    matrix.convertTo(result, CV_64F);
    return result;
}

void KalmanFilter::validateState(const cv::Mat& vector, const char* name)
{
    if (vector.empty() || vector.cols != 1)
    {
        throw std::invalid_argument(
            std::string(name) + " must be a non-empty column vector.");
    }
}

void KalmanFilter::validateSquare(
    const cv::Mat& matrix,
    int size,
    const char* name)
{
    if (matrix.empty() || matrix.rows != size || matrix.cols != size)
    {
        throw std::invalid_argument(
            std::string(name) + " must be a " +
            std::to_string(size) + "x" + std::to_string(size) +
            " matrix.");
    }
}

void KalmanFilter::validateModel() const
{
    validateState(state_, "state");
    const int state_size = state_.rows;
    const int measurement_size = measurement_matrix_.rows;

    validateSquare(covariance_, state_size, "covariance");
    validateSquare(transition_matrix_, state_size, "transition_matrix");
    validateSquare(process_noise_, state_size, "process_noise");

    if (measurement_matrix_.empty() ||
        measurement_matrix_.cols != state_size)
    {
        throw std::invalid_argument(
            "measurement_matrix columns must match state size.");
    }
    validateSquare(
        measurement_noise_,
        measurement_size,
        "measurement_noise");

    if (!control_matrix_.empty() && control_matrix_.rows != state_size)
    {
        throw std::invalid_argument(
            "control_matrix rows must match state size.");
    }
}

void KalmanFilter::symmetrizeCovariance()
{
    covariance_ = (covariance_ + covariance_.t()) * 0.5;
}

KalmanFilter makeConstantVelocityFilter(
    double position,
    double velocity,
    double dt,
    double position_variance,
    double velocity_variance,
    double process_variance,
    double measurement_variance)
{
    return KalmanFilter(
        (cv::Mat_<double>(2, 1) << position, velocity),
        (cv::Mat_<double>(2, 2) <<
            position_variance, 0.0,
            0.0, velocity_variance),
        (cv::Mat_<double>(2, 2) << 1.0, dt, 0.0, 1.0),
        (cv::Mat_<double>(1, 2) << 1.0, 0.0),
        cv::Mat::eye(2, 2, CV_64F) * process_variance,
        (cv::Mat_<double>(1, 1) << measurement_variance));
}
