#include "vision/laser_detector.hpp"

#include <opencv2/imgproc.hpp>

namespace nxv {

std::optional<cv::Point2f> LaserDetector::detect_red_or_blue_spot(const cv::Mat &color_bgr) const
{
    if (color_bgr.empty()) {
        return std::nullopt;
    }

    cv::Mat hsv;
    cv::cvtColor(color_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red1;
    cv::Mat red2;
    cv::Mat blue;
    cv::inRange(hsv, cv::Scalar(0, 80, 120), cv::Scalar(12, 255, 255), red1);
    cv::inRange(hsv, cv::Scalar(168, 80, 120), cv::Scalar(180, 255, 255), red2);
    cv::inRange(hsv, cv::Scalar(95, 80, 120), cv::Scalar(135, 255, 255), blue);

    cv::Mat mask = red1 | red2 | blue;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    cv::Point2f best_center;
    for (const auto &contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < 2.0 || area > 2000.0 || area <= best_area) {
            continue;
        }
        cv::Moments m = cv::moments(contour);
        if (std::abs(m.m00) < 1e-6) {
            continue;
        }
        best_area = area;
        best_center = cv::Point2f(static_cast<float>(m.m10 / m.m00),
                                  static_cast<float>(m.m01 / m.m00));
    }

    if (best_area <= 0.0) {
        return std::nullopt;
    }
    return best_center;
}

}  // namespace nxv

