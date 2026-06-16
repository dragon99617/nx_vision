#pragma once

#include "common/config.hpp"
#include "common/types.hpp"
#include "pose/depth_estimator.hpp"
#include "pose/laser_compensator.hpp"
#include "pose/pnp_solver.hpp"
#include "vision/target_tracker.hpp"

namespace nxv {

class AimPipeline {
public:
    explicit AimPipeline(RuntimeConfig config);

    PipelineResult process(const FrameBundle &frame, bool make_debug_panels = true);
    PipelineResult process_board_point(const FrameBundle &frame,
                                       const cv::Point3d &board_point_mm,
                                       bool make_debug_panels = true);

private:
    PipelineResult process_impl(const FrameBundle &frame,
                                const cv::Point3d &board_point_mm,
                                bool make_debug_panels);

    RuntimeConfig config_;
    TargetTracker tracker_;
    PnpSolver pnp_;
    DepthEstimator depth_;
    LaserCompensator laser_;
};

}  // namespace nxv
