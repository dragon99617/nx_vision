//协议实现
#include "comm/gimbal_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <utility>

namespace nxv {
namespace {

double steady_timestamp_s()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

uint16_t crc16_ccitt_false(const std::string &text)
{
    uint16_t crc = 0xFFFF;
    for (const unsigned char byte : text) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

int clamp_rounded(double value, int minimum, int maximum)
{
    if (!std::isfinite(value)) {
        return minimum;
    }
    const double clamped = std::clamp(
        value,
        static_cast<double>(minimum),
        static_cast<double>(maximum));
    return static_cast<int>(std::lround(clamped));
}

}  // namespace

GimbalProtocol::GimbalProtocol(SerialConfig config)
    : config_(std::move(config))
{
}

std::string GimbalProtocol::make_packet(const AimResult &aim,
                                        const DepthEstimate &depth,
                                        const FrameBundle &frame,
                                        double generation_timestamp_s) const
{
    if (config_.protocol == "legacy_a") {
        return make_legacy_packet(aim);
    }
    if (generation_timestamp_s < 0.0) {
        generation_timestamp_s = steady_timestamp_s();
    }
    return make_v2_packet(aim, depth, frame, generation_timestamp_s);
}

std::string GimbalProtocol::make_legacy_packet(const AimResult &aim) const
{
    const int yaw = aim.valid
                        ? static_cast<int>(
                              std::lround(aim.yaw_delta_deg * config_.angle_scale_cdeg))
                        : 0;
    const int pitch = aim.valid
                          ? static_cast<int>(
                                std::lround(aim.pitch_delta_deg * config_.angle_scale_cdeg))
                          : 0;
    const int distance =
        aim.valid ? static_cast<int>(std::lround(aim.distance_mm)) : 0;

    std::ostringstream oss;
    oss << "A," << yaw << "," << pitch << "," << distance << ","
        << (aim.valid ? 1 : 0) << "\n";
    return oss.str();
}

std::string GimbalProtocol::make_v2_packet(const AimResult &aim,
                                           const DepthEstimate &depth,
                                           const FrameBundle &frame,
                                           double generation_timestamp_s) const
{
    const int yaw = aim.valid
                        ? static_cast<int>(
                              std::lround(aim.yaw_delta_deg * config_.angle_scale_cdeg))
                        : 0;
    const int pitch = aim.valid
                          ? static_cast<int>(
                                std::lround(aim.pitch_delta_deg * config_.angle_scale_cdeg))
                          : 0;
    const int distance =
        aim.valid ? static_cast<int>(std::lround(aim.distance_mm)) : 0;
    const int age_us = clamp_rounded(
        (generation_timestamp_s - frame.timestamp_s) * 1'000'000.0,
        0,
        500'000);
    const int depth_age_ms =
        clamp_rounded(frame.depth_age_s * 1000.0, 0, 1000);

    uint32_t flags = 0;
    if (aim.valid) {
        flags |= 1U << 0;
    }
    if (frame.timestamp_reliable) {
        flags |= 1U << 1;
    }
    if (!frame.depth_mm.empty() && !frame.depth_reused) {
        flags |= 1U << 2;
    }
    if (depth.valid) {
        flags |= 1U << 3;
    }
    if (frame.depth_reused) {
        flags |= 1U << 4;
    }

    std::ostringstream body;
    body << "V2," << frame.sequence << "," << age_us << ","
         << yaw << "," << pitch << "," << distance << ","
         << depth_age_ms << "," << flags;

    const std::string body_text = body.str();
    std::ostringstream packet;
    packet << body_text << "," << std::uppercase << std::hex
           << std::setw(4) << std::setfill('0')
           << crc16_ccitt_false(body_text) << "\n";
    return packet.str();
}

}  // namespace nxv
