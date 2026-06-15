#include "debug_view/overlay_drawer.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iomanip>
#include <sstream>

namespace nxv {
namespace {

void put_label(cv::Mat &img, const std::string &text, int y, const cv::Scalar &color)
{
    cv::putText(img, text, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
}

std::string format_double(const char *name, double value, const char *unit)
{
    std::ostringstream oss;
    oss << name << ": " << std::fixed << std::setprecision(2) << value << unit;
    return oss.str();
}

std::vector<cv::Point> to_points(const std::vector<cv::Point2f> &pts)
{
    std::vector<cv::Point> out;
    out.reserve(pts.size());
    for (const cv::Point2f &pt : pts) {
        out.emplace_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
    }
    return out;
}

}  // namespace

cv::Mat OverlayDrawer::draw_main_overlay(const cv::Mat &color_bgr, const PipelineResult &result) const
{
    cv::Mat overlay = color_bgr.empty() ? cv::Mat(480, 640, CV_8UC3, cv::Scalar(20, 20, 20)) : color_bgr.clone();

    if (result.rectangle.valid && result.rectangle.corners.size() == 4) {
        cv::polylines(overlay, to_points(result.rectangle.corners), true, cv::Scalar(0, 255, 0), 2);
        cv::circle(overlay, cv::Point(cvRound(result.rectangle.center.x), cvRound(result.rectangle.center.y)), 5, cv::Scalar(0, 255, 255), -1);
    }

    if (result.aim.valid) {
        put_label(overlay, format_double("Yaw", result.aim.yaw_delta_deg, " deg"), 24, cv::Scalar(0, 255, 0));
        put_label(overlay, format_double("Pitch", result.aim.pitch_delta_deg, " deg"), 48, cv::Scalar(0, 255, 0));
        put_label(overlay, format_double("Distance", result.aim.distance_mm, " mm"), 72, cv::Scalar(0, 255, 0));
        if (result.depth.used_for_aim) {
            put_label(overlay, "Distance source: depth", 96, cv::Scalar(0, 255, 0));
        } else if (result.depth.fallback_used) {
            put_label(overlay, "Distance source: pnp fallback", 96, cv::Scalar(0, 255, 255));
        } else {
            put_label(overlay, "Distance source: pnp", 96, cv::Scalar(0, 255, 255));
        }
    } else {
        put_label(overlay, "NO VALID TARGET", 24, cv::Scalar(0, 0, 255));
        if (!result.depth.failure_reason.empty()) {
            put_label(overlay, "Depth: " + result.depth.failure_reason, 48, cv::Scalar(0, 0, 255));
        }
    }
    put_label(overlay, "Serial: " + result.serial_packet, overlay.rows - 18, cv::Scalar(255, 255, 0));
    return overlay;
}

cv::Mat OverlayDrawer::draw_pose_overlay(const cv::Mat &color_bgr,
                                         const PipelineResult &result,
                                         const CameraIntrinsics &intrinsics) const
{
    cv::Mat overlay = draw_main_overlay(color_bgr, result);
    if (!result.pose.valid || !intrinsics.valid) {
        return overlay;
    }

    std::vector<cv::Point3f> axes = {
        {0.0f, 0.0f, 0.0f},
        {80.0f, 0.0f, 0.0f},
        {0.0f, 80.0f, 0.0f},
        {0.0f, 0.0f, -80.0f},
    };
    std::vector<cv::Point2f> points;
    cv::projectPoints(axes,
                      result.pose.rvec,
                      result.pose.tvec,
                      intrinsics.camera_matrix,
                      intrinsics.dist_coeffs,
                      points);
    if (points.size() == 4) {
        const cv::Point origin(cvRound(points[0].x), cvRound(points[0].y));
        cv::line(overlay, origin, cv::Point(cvRound(points[1].x), cvRound(points[1].y)), cv::Scalar(0, 0, 255), 3);
        cv::line(overlay, origin, cv::Point(cvRound(points[2].x), cvRound(points[2].y)), cv::Scalar(0, 255, 0), 3);
        cv::line(overlay, origin, cv::Point(cvRound(points[3].x), cvRound(points[3].y)), cv::Scalar(255, 0, 0), 3);
    }
    return overlay;
}

}  // namespace nxv
