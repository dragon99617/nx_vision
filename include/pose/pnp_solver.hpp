#pragma once

#include "common/types.hpp"

namespace nxv {

class PnpSolver {
public:
    PoseResult solve(const RectangleDetection &rect,
                     const CameraIntrinsics &intrinsics,
                     const TargetGeometry &target) const;
};

}  // namespace nxv

