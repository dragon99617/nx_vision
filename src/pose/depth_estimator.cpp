#include "pose/depth_estimator.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace nxv {
namespace {

double clamp_ratio(double value)
{
    if (value < 0.05) {
        return 0.05;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}

std::vector<cv::Point> make_inner_polygon(const std::vector<cv::Point2f> &corners, double shrink_ratio)
{
    cv::Point2f center(0.0f, 0.0f);
    for (const cv::Point2f &corner : corners) {
        center += corner;
    }
    center *= 1.0f / static_cast<float>(corners.size());

    const double ratio = clamp_ratio(shrink_ratio);
    std::vector<cv::Point> inner;
    inner.reserve(corners.size());
    for (const cv::Point2f &corner : corners) {
        const cv::Point2f shrunk = center + (corner - center) * static_cast<float>(ratio);
        inner.emplace_back(cvRound(shrunk.x), cvRound(shrunk.y));
    }
    return inner;
}

bool read_depth_value(const cv::Mat &depth_mm, int y, int x, float *value)
{
    if (value == nullptr) {
        return false;
    }
    if (depth_mm.type() == CV_32FC1) {
        *value = depth_mm.at<float>(y, x);
        return true;
    }
    if (depth_mm.type() == CV_16UC1) {
        *value = static_cast<float>(depth_mm.at<uint16_t>(y, x));
        return true;
    }
    return false;
}

}  // namespace

DepthEstimate DepthEstimator::estimate(const cv::Mat &depth_mm,
                                       const RectangleDetection &rect,
                                       const CameraIntrinsics &intrinsics,
                                       const DepthConfig &config,
                                       const cv::Point2f &target_px) const
{
    DepthEstimate estimate;
    if (!config.enabled) {
        estimate.failure_reason = "depth disabled";
        return estimate;
    }
    if (depth_mm.empty()) {
        estimate.failure_reason = "missing depth frame";
        return estimate;
    }
    if (depth_mm.channels() != 1) {
        estimate.failure_reason = "unsupported depth channels";
        return estimate;
    }
    if (!rect.valid || rect.corners.size() != 4) {
        estimate.failure_reason = "missing rectangle";
        return estimate;
    }
    if (!intrinsics.valid || intrinsics.camera_matrix.empty()) {
        estimate.failure_reason = "missing intrinsics";
        return estimate;
    }
    if (target_px.x < 0.0f || target_px.y < 0.0f ||
        target_px.x >= static_cast<float>(depth_mm.cols) ||
        target_px.y >= static_cast<float>(depth_mm.rows)) {
        estimate.failure_reason = "target pixel outside depth";
        return estimate;
    }

    const double fx = intrinsics.camera_matrix.at<double>(0, 0);
    const double fy = intrinsics.camera_matrix.at<double>(1, 1);
    const double cx = intrinsics.camera_matrix.at<double>(0, 2);
    const double cy = intrinsics.camera_matrix.at<double>(1, 2);
    if (fx <= 1e-6 || fy <= 1e-6) {
        estimate.failure_reason = "invalid intrinsics";
        return estimate;
    }

    const std::vector<cv::Point> inner = make_inner_polygon(rect.corners, config.roi_shrink_ratio);
    cv::Mat mask(depth_mm.rows, depth_mm.cols, CV_8UC1, cv::Scalar(0));
    cv::fillConvexPoly(mask, inner, cv::Scalar(255), cv::LINE_AA);

    cv::Rect roi = cv::boundingRect(inner) & cv::Rect(0, 0, depth_mm.cols, depth_mm.rows);
    if (roi.empty()) {
        estimate.failure_reason = "empty depth roi";
        return estimate;
    }

    std::vector<float> samples;
    samples.reserve(static_cast<size_t>(roi.area()));
    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        const uint8_t *mask_row = mask.ptr<uint8_t>(y);
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
            if (mask_row[x] == 0) {
                continue;
            }
            float value = 0.0f;
            if (!read_depth_value(depth_mm, y, x, &value) ||
                !std::isfinite(value) ||
                value < config.min_depth_mm ||
                value > config.max_depth_mm) {
                continue;
            }
            samples.push_back(value);
        }
    }

    estimate.valid_sample_count = static_cast<int>(samples.size());
    if (estimate.valid_sample_count < config.min_valid_samples) {
        estimate.failure_reason = "not enough valid depth samples";
        return estimate;
    }

    const size_t median_index = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + static_cast<long>(median_index), samples.end());
    const double z = samples[median_index];
    estimate.depth_mm = z;
    estimate.target_cam_mm = cv::Point3d((static_cast<double>(target_px.x) - cx) * z / fx,
                                         (static_cast<double>(target_px.y) - cy) * z / fy,
                                         z);
    estimate.valid = true;
    return estimate;
}

}  // namespace nxv
