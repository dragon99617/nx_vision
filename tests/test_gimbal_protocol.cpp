#include "comm/gimbal_protocol.hpp"

#include <cassert>
#include <cstdint>
#include <limits>

namespace {

nxv::AimResult make_valid_aim()
{
    nxv::AimResult aim;
    aim.valid = true;
    aim.yaw_delta_deg = 1.25;
    aim.pitch_delta_deg = -0.5;
    aim.distance_mm = 1000.0;
    return aim;
}

nxv::FrameBundle make_frame(uint32_t sequence,
                            double timestamp_s,
                            bool timestamp_reliable,
                            double depth_age_s)
{
    nxv::FrameBundle frame;
    frame.sequence = sequence;
    frame.timestamp_s = timestamp_s;
    frame.timestamp_reliable = timestamp_reliable;
    frame.depth_age_s = depth_age_s;
    return frame;
}

}  // namespace

int main()
{
    nxv::SerialConfig v2_config;
    v2_config.angle_scale_cdeg = 100;
    v2_config.protocol = "v2";
    nxv::GimbalProtocol v2(v2_config);

    const nxv::AimResult valid_aim = make_valid_aim();
    nxv::DepthEstimate valid_depth;
    valid_depth.valid = true;

    nxv::FrameBundle new_depth_frame =
        make_frame(42, 10.0, true, 0.0046);
    new_depth_frame.depth_mm =
        cv::Mat(1, 1, CV_32FC1, cv::Scalar(1000.0f));
    assert(v2.make_packet(valid_aim,
                          valid_depth,
                          new_depth_frame,
                          10.012345) ==
           "V2,42,12345,125,-50,1000,5,15,4B91\n");

    nxv::AimResult invalid_aim;
    invalid_aim.yaw_delta_deg = 99.0;
    invalid_aim.pitch_delta_deg = 99.0;
    invalid_aim.distance_mm = 9999.0;
    const nxv::DepthEstimate invalid_depth;
    const nxv::FrameBundle invalid_frame =
        make_frame(7, 20.0, false, 0.0);
    assert(v2.make_packet(invalid_aim,
                          invalid_depth,
                          invalid_frame,
                          20.001) ==
           "V2,7,1000,0,0,0,0,0,DBC6\n");

    nxv::FrameBundle reused_depth_frame =
        make_frame(99, 30.0, true, 0.100);
    reused_depth_frame.depth_mm =
        cv::Mat(1, 1, CV_32FC1, cv::Scalar(1000.0f));
    reused_depth_frame.depth_reused = true;
    assert(v2.make_packet(valid_aim,
                          valid_depth,
                          reused_depth_frame,
                          30.050) ==
           "V2,99,50000,125,-50,1000,100,27,8142\n");

    nxv::FrameBundle unreliable_frame =
        make_frame(100, 40.0, false, 0.100);
    unreliable_frame.depth_mm =
        cv::Mat(1, 1, CV_32FC1, cv::Scalar(1000.0f));
    assert(v2.make_packet(valid_aim,
                          valid_depth,
                          unreliable_frame,
                          40.050) ==
           "V2,100,50000,125,-50,1000,100,13,8E2D\n");

    const nxv::FrameBundle lower_bound_frame =
        make_frame(std::numeric_limits<uint32_t>::max() - 1U,
                   50.0,
                   true,
                   -1.0);
    assert(v2.make_packet(valid_aim,
                          invalid_depth,
                          lower_bound_frame,
                          49.0) ==
           "V2,4294967294,0,125,-50,1000,0,3,33DC\n");

    const nxv::FrameBundle upper_bound_frame =
        make_frame(std::numeric_limits<uint32_t>::max(),
                   60.0,
                   true,
                   2.0);
    assert(v2.make_packet(valid_aim,
                          invalid_depth,
                          upper_bound_frame,
                          61.0) ==
           "V2,4294967295,500000,125,-50,1000,1000,3,7775\n");

    nxv::SerialConfig legacy_config;
    legacy_config.angle_scale_cdeg = 100;
    legacy_config.protocol = "legacy_a";
    nxv::GimbalProtocol legacy(legacy_config);
    assert(legacy.make_packet(valid_aim,
                              valid_depth,
                              new_depth_frame,
                              10.012345) ==
           "A,125,-50,1000,1\n");
    assert(legacy.make_packet(invalid_aim,
                              invalid_depth,
                              invalid_frame,
                              20.001) ==
           "A,0,0,0,0\n");
    return 0;
}
