#include <detector.hpp>
#include <stdexcept>

void Detector::initialize(
    const cv::Scalar& hsv_low,
    const cv::Scalar& hsv_high,
    double min_circ,
    double min_a,
    double max_a,
    double max_aspect_diff)
{
    hsv_threshold_low = hsv_low;
    hsv_threshold_high = hsv_high;
    min_circularity = min_circ;
    min_area = min_a;
    max_area = max_a;
    max_aspect_ratio_diff = max_aspect_diff;

    initialized = true;
}

std::optional<GreenTarget> Detector::detect(const cv::Mat& image)
{
    if (!initialized)
    {
        throw std::runtime_error("Detector not initialized. Call initialize() before detect().");
    }

    if (image.empty())
    {
        return std::nullopt;
    }

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Threshold the HSV image to get only green colors
    cv::Mat mask;
    cv::inRange(hsv_image, hsv_threshold_low, hsv_threshold_high, mask);

    // 使用闭运算填补目标区域中的小孔洞和断裂：先膨胀，再腐蚀
    cv::Mat morphology_kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morphology_kernel);

    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::optional<GreenTarget> best_target;

    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < min_area || (max_area > 0.0 && area > max_area))
        {
            continue;
        }

        double perimeter = cv::arcLength(contour, true);
        if (perimeter <= 0.0)
        {
            continue;
        }

        // 过滤过于细长的轮廓，避免将非目标绿色区域识别为目标
        cv::Rect bounding_box = cv::boundingRect(contour);
        if (bounding_box.width <= 0 || bounding_box.height <= 0)
        {
            continue;
        }

        double aspect_ratio_diff =
            static_cast<double>(std::abs(bounding_box.width - bounding_box.height)) /
            static_cast<double>(std::max(bounding_box.width, bounding_box.height));
        if (aspect_ratio_diff > max_aspect_ratio_diff)
        {
            continue;
        }

        // 计算轮廓圆形度
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < min_circularity)
        {
            continue;
        }

        // 使用最小包围圆计算目标中心和半径
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        GreenTarget candidate(center, radius, bounding_box, area, circularity);
        if (!best_target || candidate.area > best_target->area)
        {
            best_target = candidate;
        }
    }

    return best_target;
}
