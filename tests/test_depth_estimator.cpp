#include "pose/depth_estimator.hpp"

#include <cassert>
#include <cmath>

namespace {

nxv::CameraIntrinsics make_intrinsics()
{
    nxv::CameraIntrinsics intrinsics;
    intrinsics.camera_matrix = (cv::Mat_<double>(3, 3) << 100.0, 0.0, 5.0,
                                                         0.0, 100.0, 5.0,
                                                         0.0, 0.0, 1.0);
    intrinsics.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    intrinsics.image_size = cv::Size(10, 10);
    intrinsics.valid = true;
    return intrinsics;
}

nxv::RectangleDetection make_rect()
{
    nxv::RectangleDetection rect;
    rect.valid = true;
    rect.corners = {
        cv::Point2f(2.0f, 2.0f),
        cv::Point2f(8.0f, 2.0f),
        cv::Point2f(8.0f, 8.0f),
        cv::Point2f(2.0f, 8.0f),
    };
    rect.center = cv::Point2f(5.0f, 5.0f);
    return rect;
}

nxv::DepthConfig make_config()
{
    nxv::DepthConfig config;
    config.enabled = true;
    config.min_depth_mm = 150.0;
    config.max_depth_mm = 8000.0;
    config.roi_shrink_ratio = 1.0;
    config.min_valid_samples = 4;
    config.fallback_to_pnp = true;
    return config;
}

}  // namespace

int main()
{
    nxv::DepthEstimator estimator;
    const nxv::CameraIntrinsics intrinsics = make_intrinsics();
    const nxv::RectangleDetection rect = make_rect();
    nxv::DepthConfig config = make_config();

    cv::Mat depth(10, 10, CV_32FC1, cv::Scalar(1000.0f));
    depth.at<float>(5, 5) = 0.0f;
    depth.at<float>(6, 6) = 7000.0f;

    nxv::DepthEstimate estimate = estimator.estimate(depth, rect, intrinsics, config, cv::Point2f(7.0f, 5.0f));
    assert(estimate.valid);
    assert(estimate.valid_sample_count >= config.min_valid_samples);
    assert(std::abs(estimate.depth_mm - 1000.0) < 1e-6);
    assert(std::abs(estimate.target_cam_mm.x - 20.0) < 1e-6);
    assert(std::abs(estimate.target_cam_mm.y - 0.0) < 1e-6);
    assert(std::abs(estimate.target_cam_mm.z - 1000.0) < 1e-6);

    config.min_valid_samples = 1000;
    estimate = estimator.estimate(depth, rect, intrinsics, config, cv::Point2f(5.0f, 5.0f));
    assert(!estimate.valid);
    assert(estimate.failure_reason == "not enough valid depth samples");

    config.enabled = false;
    estimate = estimator.estimate(depth, rect, intrinsics, config, cv::Point2f(5.0f, 5.0f));
    assert(!estimate.valid);
    assert(estimate.failure_reason == "depth disabled");

    return 0;
}
