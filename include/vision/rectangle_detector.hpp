#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

namespace nxv {

class RectangleDetector {
public:
    RectangleDetection detect(const PreprocessOutput &preprocess, const VisionParams &params) const;

private:
    static std::vector<cv::Point2f> order_corners(const std::vector<cv::Point2f> &points);
};

}  // namespace nxv

