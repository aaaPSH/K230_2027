#pragma once
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

struct GreenTarget
{
    cv::Point2f center;
    float radius = 0.0f;

    cv::Rect bounding_box;

    double area = 0.0;
    double circularity = 0.0;

     GreenTarget(cv::Point2f c, float r, cv::Rect bb, double a, double circ) : center(c), radius(r), bounding_box(bb), area(a), circularity(circ) {}
};



class Detector
{
private:
    cv::Scalar hsv_threshold_low;
    cv::Scalar hsv_threshold_high;

    double min_circularity;
    double min_area;
    double max_area;
    double max_aspect_ratio_diff = 0.5;

    bool initialized = false;


public:
    Detector() = default;
    void initialize(
        const cv::Scalar& hsv_low,
        const cv::Scalar& hsv_high,
        double min_circ,
        double min_a,
        double max_a,
        double max_aspect_diff = 0.5);
    std::optional<GreenTarget> detect(const cv::Mat& image);
};  

