#include "vision/target_tracker.hpp"

#include "vision/preprocess.hpp"

namespace nxv {

RectangleDetection TargetTracker::update(const cv::Mat &color_bgr, const VisionParams &params)
{
    PreprocessOutput preprocess = run_preprocess(color_bgr, params);
    return detector_.detect(preprocess, params);
}

void TargetTracker::reset()
{
}

}  // namespace nxv

