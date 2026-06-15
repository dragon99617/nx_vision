#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

namespace nxv {

PreprocessOutput run_preprocess(const cv::Mat &color_bgr, const VisionParams &params);

}  // namespace nxv

