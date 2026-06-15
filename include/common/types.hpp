#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace nxv {

struct FrameBundle {
    cv::Mat color_bgr;
    cv::Mat depth_mm;
    double timestamp_s = 0.0;
};

struct CameraIntrinsics {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size image_size;
    bool valid = false;
};

struct TargetGeometry {
    double width_mm = 265.0;
    double height_mm = 203.0;
    double circle_radius_mm = 60.0;

    std::vector<cv::Point3f> outer_object_points() const;
    cv::Point3d center_mm() const;
};

struct LaserExtrinsic {
    cv::Point3d origin_cam_mm = cv::Point3d(0.0, 0.0, 0.0);
};

struct PreprocessOutput {
    cv::Mat resized_bgr;
    cv::Mat gray;
    cv::Mat blurred;
    cv::Mat binary;
    cv::Mat closed;
    cv::Mat edges;
    cv::Mat combined;
    double scale_x_to_src = 1.0;
    double scale_y_to_src = 1.0;
};

struct RectangleDetection {
    bool valid = false;
    std::vector<cv::Point2f> corners;
    cv::Point2f center = cv::Point2f(0.0f, 0.0f);
    double score = 0.0;
    cv::Mat contour_overlay;
    cv::Mat perspective_view;
};

struct PoseResult {
    bool valid = false;
    cv::Vec3d rvec = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d tvec = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Point3d target_cam_mm = cv::Point3d(0.0, 0.0, 0.0);
    double distance_mm = 0.0;
};

struct DepthEstimate {
    bool valid = false;
    bool used_for_aim = false;
    bool fallback_used = false;
    cv::Point3d target_cam_mm = cv::Point3d(0.0, 0.0, 0.0);
    double depth_mm = 0.0;
    int valid_sample_count = 0;
    std::string failure_reason;
};

struct AimResult {
    bool valid = false;
    double yaw_delta_deg = 0.0;
    double pitch_delta_deg = 0.0;
    double distance_mm = 0.0;
    cv::Point3d target_cam_mm = cv::Point3d(0.0, 0.0, 0.0);
    cv::Point3d laser_origin_cam_mm = cv::Point3d(0.0, 0.0, 0.0);
};

struct PipelineResult {
    bool valid = false;
    PreprocessOutput preprocess;
    RectangleDetection rectangle;
    PoseResult pose;
    DepthEstimate depth;
    AimResult aim;
    std::string serial_packet;
    std::map<std::string, cv::Mat> panels;
};

}  // namespace nxv
