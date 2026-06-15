#include "pose/laser_compensator.hpp"

#include <cmath>

namespace nxv {
namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

}  // namespace

AimResult LaserCompensator::compute(const PoseResult &pose,
                                    const LaserExtrinsic &laser,
                                    const cv::Point3d &target_cam_mm) const
{
    AimResult aim;
    if (!pose.valid) {
        return aim;
    }

    const cv::Point3d direction = target_cam_mm - laser.origin_cam_mm;
    const double forward = std::sqrt(direction.x * direction.x + direction.z * direction.z);
    if (forward < 1e-6 || direction.z <= 1e-6) {
        return aim;
    }

    aim.valid = true;
    aim.yaw_delta_deg = std::atan2(direction.x, direction.z) * kRadToDeg;
    aim.pitch_delta_deg = std::atan2(-direction.y, forward) * kRadToDeg;
    aim.distance_mm = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    aim.target_cam_mm = target_cam_mm;
    aim.laser_origin_cam_mm = laser.origin_cam_mm;
    return aim;
}

}  // namespace nxv
