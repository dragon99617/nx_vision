#include "pose/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>

namespace nxv {

PoseResult PnpSolver::solve(const RectangleDetection &rect,
                            const CameraIntrinsics &intrinsics,
                            const TargetGeometry &target) const
{
    PoseResult result;
    if (!rect.valid || rect.corners.size() != 4 || !intrinsics.valid) {
        return result;
    }

    const std::vector<cv::Point3f> object_points = target.outer_object_points();
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    bool ok = false;

    try {
        ok = cv::solvePnP(object_points,
                          rect.corners,
                          intrinsics.camera_matrix,
                          intrinsics.dist_coeffs,
                          rvec,
                          tvec,
                          false,
                          cv::SOLVEPNP_IPPE);
    } catch (const cv::Exception &) {
        ok = cv::solvePnP(object_points,
                          rect.corners,
                          intrinsics.camera_matrix,
                          intrinsics.dist_coeffs,
                          rvec,
                          tvec,
                          false,
                          cv::SOLVEPNP_ITERATIVE);
    }

    if (!ok) {
        return result;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Matx33d rotation;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation(row, col) = R.at<double>(row, col);
        }
    }

    result.valid = true;
    result.rvec = rvec;
    result.tvec = tvec;
    result.rotation = rotation;
    result.target_cam_mm = cv::Point3d(tvec[0], tvec[1], tvec[2]);
    result.distance_mm = std::sqrt(tvec[0] * tvec[0] + tvec[1] * tvec[1] + tvec[2] * tvec[2]);
    return result;
}

}  // namespace nxv
