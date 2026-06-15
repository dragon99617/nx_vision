#pragma once

#include "common/config.hpp"
#include "common/types.hpp"
#include "vision/rectangle_detector.hpp"

namespace nxv {

class TargetTracker {
public:
    RectangleDetection update(const cv::Mat &color_bgr, const VisionParams &params);
    void reset();

private:
    RectangleDetector detector_;
};

}  // namespace nxv

