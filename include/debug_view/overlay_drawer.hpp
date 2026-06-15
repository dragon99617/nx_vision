#pragma once

#include "common/types.hpp"

namespace nxv {

class OverlayDrawer {
public:
    cv::Mat draw_main_overlay(const cv::Mat &color_bgr, const PipelineResult &result) const;
    cv::Mat draw_pose_overlay(const cv::Mat &color_bgr,
                              const PipelineResult &result,
                              const CameraIntrinsics &intrinsics) const;
};

}  // namespace nxv

