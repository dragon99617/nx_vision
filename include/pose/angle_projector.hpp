#pragma once

#include "common/types.hpp"

#include <opencv2/core.hpp>

namespace nxv {

cv::Point3d transform_board_point_to_camera(const cv::Point3d &board_point_mm,
                                            const PoseResult &pose);

}  // namespace nxv
