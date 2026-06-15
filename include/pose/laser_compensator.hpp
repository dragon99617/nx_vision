#pragma once

#include "common/types.hpp"

namespace nxv {

class LaserCompensator {
public:
    AimResult compute(const PoseResult &pose,
                      const LaserExtrinsic &laser,
                      const cv::Point3d &target_cam_mm) const;
};

}  // namespace nxv

