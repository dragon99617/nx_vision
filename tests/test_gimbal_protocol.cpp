#include "comm/gimbal_protocol.hpp"

#include <cassert>

int main()
{
    nxv::SerialConfig config;
    config.angle_scale_cdeg = 100;
    nxv::GimbalProtocol protocol(config);
    nxv::AimResult aim;
    aim.valid = true;
    aim.yaw_delta_deg = 1.25;
    aim.pitch_delta_deg = -0.5;
    aim.distance_mm = 1000.0;
    assert(protocol.make_packet(aim) == "A,125,-50,1000,1\n");
    assert(protocol.make_invalid_packet() == "A,0,0,0,0\n");
    return 0;
}

