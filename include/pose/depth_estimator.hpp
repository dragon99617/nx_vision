#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

namespace nxv {

class DepthEstimator {
public:
    DepthEstimate estimate(const cv::Mat &depth_mm,
                           const RectangleDetection &rect,
                           const CameraIntrinsics &intrinsics,
                           const DepthConfig &config,
                           const cv::Point2f &target_px) const;
};

}  // namespace nxv
