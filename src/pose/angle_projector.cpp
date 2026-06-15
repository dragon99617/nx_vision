#include "pose/angle_projector.hpp"

#include "common/types.hpp"

namespace nxv {

cv::Point3d transform_board_point_to_camera(const cv::Point3d &board_point_mm,
                                            const PoseResult &pose)
{
    const cv::Vec3d p(board_point_mm.x, board_point_mm.y, board_point_mm.z);
    const cv::Vec3d t(pose.tvec[0], pose.tvec[1], pose.tvec[2]);
    const cv::Vec3d out = pose.rotation * p + t;
    return cv::Point3d(out[0], out[1], out[2]);
}

}  // namespace nxv

