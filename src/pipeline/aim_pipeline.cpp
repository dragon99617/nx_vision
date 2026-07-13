#include "pipeline/aim_pipeline.hpp"

#include "comm/gimbal_protocol.hpp"
#include "pose/angle_projector.hpp"
#include "vision/preprocess.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <utility>

namespace nxv {
namespace {

cv::Point2f project_board_point_to_image(const cv::Point3d &board_point_mm,
                                         const PoseResult &pose,
                                         const CameraIntrinsics &intrinsics,
                                         const cv::Point2f &fallback)
{
    if (!pose.valid || !intrinsics.valid) {
        return fallback;
    }

    std::vector<cv::Point3f> object_points = {
        cv::Point3f(static_cast<float>(board_point_mm.x),
                    static_cast<float>(board_point_mm.y),
                    static_cast<float>(board_point_mm.z)),
    };
    std::vector<cv::Point2f> image_points;
    try {
        cv::projectPoints(object_points,
                          pose.rvec,
                          pose.tvec,
                          intrinsics.camera_matrix,
                          intrinsics.dist_coeffs,
                          image_points);
    } catch (const cv::Exception &) {
        return fallback;
    }
    return image_points.empty() ? fallback : image_points.front();
}

cv::Mat make_depth_preview(const cv::Mat &depth_mm, double max_depth_mm)
{
    if (depth_mm.empty()) {
        return {};
    }

    cv::Mat depth_float;
    if (depth_mm.type() == CV_32FC1) {
        depth_float = depth_mm;
    } else if (depth_mm.type() == CV_16UC1) {
        depth_mm.convertTo(depth_float, CV_32F);
    } else {
        return {};
    }

    const double scale = max_depth_mm > 1.0 ? 255.0 / max_depth_mm : 255.0 / 8000.0;
    cv::Mat clipped;
    cv::threshold(depth_float, clipped, max_depth_mm, max_depth_mm, cv::THRESH_TRUNC);
    clipped.setTo(0.0f, clipped != clipped);

    cv::Mat gray;
    clipped.convertTo(gray, CV_8U, scale);
    cv::Mat color;
    cv::applyColorMap(gray, color, cv::COLORMAP_JET);
    color.setTo(cv::Scalar(15, 15, 15), gray == 0);
    return color;
}

}  // namespace

AimPipeline::AimPipeline(RuntimeConfig config)
    : config_(std::move(config))
{
}

PipelineResult AimPipeline::process(const FrameBundle &frame, bool make_debug_panels)
{
    return process_impl(frame, config_.target.center_mm(), make_debug_panels);
}

PipelineResult AimPipeline::process_board_point(const FrameBundle &frame,
                                                const cv::Point3d &board_point_mm,
                                                bool make_debug_panels)
{
    return process_impl(frame, board_point_mm, make_debug_panels);
}

PipelineResult AimPipeline::process_impl(const FrameBundle &frame,
                                         const cv::Point3d &board_point_mm,
                                         bool make_debug_panels)
{
    PipelineResult result;
    if (frame.color_bgr.empty()) {
        GimbalProtocol protocol(config_.serial);
        result.serial_packet = protocol.make_packet(result.aim, result.depth, frame);
        return result;
    }

    result.preprocess = run_preprocess(frame.color_bgr, config_.vision);
    RectangleDetector detector;
    result.rectangle = detector.detect(result.preprocess, config_.vision, make_debug_panels);
    result.pose = pnp_.solve(result.rectangle, config_.intrinsics, config_.target);

    const cv::Point2f target_px = project_board_point_to_image(board_point_mm,
                                                               result.pose,
                                                               config_.intrinsics,
                                                               result.rectangle.center);
    cv::Mat depth_for_estimate = frame.depth_mm;
    std::string rejected_reused_depth_reason;
    if (frame.depth_reused) {
        if (!config_.depth.reuse_last_depth) {
            depth_for_estimate.release();
            rejected_reused_depth_reason = "reused depth disabled";
        } else if (config_.depth.max_reused_depth_age_s > 0.0 &&
                   frame.depth_age_s > config_.depth.max_reused_depth_age_s) {
            depth_for_estimate.release();
            rejected_reused_depth_reason = "reused depth expired";
        }
    }

    result.depth = depth_.estimate(depth_for_estimate,
                                   result.rectangle,
                                   config_.intrinsics,
                                   config_.depth,
                                   target_px);
    result.depth.source_reused = frame.depth_reused && !depth_for_estimate.empty();
    result.depth.source_age_s = frame.depth_age_s;
    if (!rejected_reused_depth_reason.empty() && result.depth.failure_reason == "missing depth frame") {
        result.depth.failure_reason = rejected_reused_depth_reason;
    }

    cv::Point3d pnp_target_cam = result.pose.target_cam_mm;
    if (result.pose.valid) {
        pnp_target_cam = transform_board_point_to_camera(board_point_mm, result.pose);
    }

    cv::Point3d target_cam = pnp_target_cam;
    bool can_aim = result.pose.valid;
    if (result.depth.valid) {
        target_cam = result.depth.target_cam_mm;
        result.depth.used_for_aim = true;
        can_aim = true;
    } else if (config_.depth.enabled && !config_.depth.fallback_to_pnp) {
        can_aim = false;
    } else if (config_.depth.enabled && result.pose.valid) {
        result.depth.fallback_used = true;
    }

    PoseResult aim_pose = result.pose;
    aim_pose.valid = can_aim;
    result.aim = laser_.compute(aim_pose, config_.laser, target_cam);
    result.valid = result.aim.valid;

    GimbalProtocol protocol(config_.serial);
    result.serial_packet = protocol.make_packet(result.aim, result.depth, frame);

    if (make_debug_panels) {
        result.panels["raw"] = frame.color_bgr;
        result.panels["binary"] = result.preprocess.binary;
        result.panels["edges"] = result.preprocess.edges;
        result.panels["combined"] = result.preprocess.combined;
        result.panels["contours"] = result.rectangle.contour_overlay;
        result.panels["perspective"] = result.rectangle.perspective_view;
        result.panels["depth"] = make_depth_preview(depth_for_estimate, config_.depth.max_depth_mm);
    }
    return result;
}

}  // namespace nxv
