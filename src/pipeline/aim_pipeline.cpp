#include "pipeline/aim_pipeline.hpp"

#include "comm/gimbal_protocol.hpp"
#include "pose/angle_projector.hpp"
#include "vision/preprocess.hpp"

#include <utility>

namespace nxv {

AimPipeline::AimPipeline(RuntimeConfig config)
    : config_(std::move(config))
{
}

PipelineResult AimPipeline::process(const FrameBundle &frame)
{
    return process_impl(frame, config_.target.center_mm());
}

PipelineResult AimPipeline::process_board_point(const FrameBundle &frame, const cv::Point3d &board_point_mm)
{
    return process_impl(frame, board_point_mm);
}

PipelineResult AimPipeline::process_impl(const FrameBundle &frame, const cv::Point3d &board_point_mm)
{
    PipelineResult result;
    if (frame.color_bgr.empty()) {
        GimbalProtocol protocol(config_.serial);
        result.serial_packet = protocol.make_invalid_packet();
        return result;
    }

    result.preprocess = run_preprocess(frame.color_bgr, config_.vision);
    RectangleDetector detector;
    result.rectangle = detector.detect(result.preprocess, config_.vision);
    result.pose = pnp_.solve(result.rectangle, config_.intrinsics, config_.target);

    cv::Point3d target_cam = result.pose.target_cam_mm;
    if (result.pose.valid) {
        target_cam = transform_board_point_to_camera(board_point_mm, result.pose);
    }

    result.aim = laser_.compute(result.pose, config_.laser, target_cam);
    result.valid = result.aim.valid;

    GimbalProtocol protocol(config_.serial);
    result.serial_packet = protocol.make_packet(result.aim);

    result.panels["raw"] = frame.color_bgr;
    result.panels["binary"] = result.preprocess.binary;
    result.panels["edges"] = result.preprocess.edges;
    result.panels["combined"] = result.preprocess.combined;
    result.panels["contours"] = result.rectangle.contour_overlay;
    result.panels["perspective"] = result.rectangle.perspective_view;
    return result;
}

}  // namespace nxv
