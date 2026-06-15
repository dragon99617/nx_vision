//协议实现
#include "comm/gimbal_protocol.hpp"

#include <cmath>
#include <sstream>
#include <utility>

namespace nxv {

GimbalProtocol::GimbalProtocol(SerialConfig config)
    : config_(std::move(config))
{
}

std::string GimbalProtocol::make_packet(const AimResult &aim) const
{
    if (!aim.valid) {
        return make_invalid_packet();
    }

    const int yaw = static_cast<int>(std::lround(aim.yaw_delta_deg * config_.angle_scale_cdeg));
    const int pitch = static_cast<int>(std::lround(aim.pitch_delta_deg * config_.angle_scale_cdeg));
    const int distance = static_cast<int>(std::lround(aim.distance_mm));

    std::ostringstream oss;
    oss << "A," << yaw << "," << pitch << "," << distance << ",1\n";
    return oss.str();
}

std::string GimbalProtocol::make_invalid_packet() const
{
    return "A,0,0,0,0\n";
}

}  // namespace nxv
