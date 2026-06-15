#pragma once

#include <opencv2/core.hpp>

#include <optional>

namespace nxv {

class LaserDetector {
public:
    std::optional<cv::Point2f> detect_red_or_blue_spot(const cv::Mat &color_bgr) const;
};

}  // namespace nxv

