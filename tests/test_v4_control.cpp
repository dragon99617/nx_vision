#include "comm/time_sync.hpp"
#include "comm/v4_protocol.hpp"
#include "control/world_controller.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::array<double, 4> yaw_quaternion(double yaw)
{
    return {std::cos(0.5 * yaw), 0.0, std::sin(0.5 * yaw), 0.0};
}

cv::Vec3d rotate_world_to_body(const std::array<double, 4> &q, const cv::Vec3d &world)
{
    const std::array<double, 4> conjugate {q[0], -q[1], -q[2], -q[3]};
    const cv::Vec3d qv(conjugate[1], conjugate[2], conjugate[3]);
    const cv::Vec3d t = 2.0 * qv.cross(world);
    return world + conjugate[0] * t + qv.cross(t);
}

}  // namespace

int main()
{
    const nxv::RuntimeConfig runtime_config = nxv::load_config("config");
    assert(runtime_config.serial.protocol == "v4");
    assert(runtime_config.control.velocity_feedforward_enabled);
    assert(!runtime_config.control.torque_feedforward_enabled);
    assert(std::abs(cv::determinant(cv::Mat(runtime_config.control.body_from_camera)) - 1.0) <
           1.0e-9);
    static constexpr char crc_text[] = "123456789";
    assert(nxv::v4::crc32c(reinterpret_cast<const uint8_t *>(crc_text), 9U) ==
           0xE3069283U);
    nxv::v4::ControlSetpoint control;
    control.execute_time_us = 123456;
    control.yaw_position_urad = 1234;
    control.yaw_velocity_urad_s = -5678;
    control.yaw_torque_1e4_nm = 100;
    control.pitch_position_urad = -2222;
    control.pitch_velocity_urad_s = 3333;
    control.pitch_torque_1e4_nm = -50;
    control.flags = nxv::v4::ControlValid | nxv::v4::VelocityFeedforward;
    const auto bytes = nxv::v4::encode_control(42, control);
    assert(bytes.size() == 42U);
    nxv::v4::StreamDecoder decoder;
    assert(decoder.push(bytes.data(), 7).empty());
    const auto frames = decoder.push(bytes.data() + 7, bytes.size() - 7);
    assert(frames.size() == 1U && frames.front().sequence == 42U);
    nxv::v4::ControlSetpoint decoded;
    assert(nxv::v4::decode_control(frames.front(), &decoded));
    assert(decoded.execute_time_us == control.execute_time_us);
    assert(decoded.pitch_torque_1e4_nm == control.pitch_torque_1e4_nm);
    auto corrupt = bytes;
    corrupt[20] ^= 0x80U;
    assert(decoder.push(corrupt.data(), corrupt.size()).empty());
    assert(decoder.crc_error_count() == 1U);
    std::vector<uint8_t> sticky = bytes;
    sticky.insert(sticky.end(), bytes.begin(), bytes.end());
    assert(decoder.push(sticky.data(), sticky.size()).size() == 2U);
    std::vector<uint8_t> fault_payload(12U, 0U);
    fault_payload[4] = 0x15U;
    const auto fault_bytes = nxv::v4::encode_frame(nxv::v4::MessageType::FaultStatus,
                                                   44U,
                                                   fault_payload);
    const auto fault_frames = decoder.push(fault_bytes.data(), fault_bytes.size());
    nxv::v4::FaultStatus fault;
    assert(fault_frames.size() == 1U &&
           nxv::v4::decode_fault_status(fault_frames.front(), &fault));
    assert(fault.fault_bits == 0x15U);

    nxv::ClockSynchronizer sync;
    const uint64_t host_base_ns = 1'000'000'000'000ULL;
    for (uint32_t i = 0; i < 12; ++i) {
        nxv::v4::SyncResponse response;
        response.host_transmit_time_ns = host_base_ns + i * 100'000'000ULL;
        response.mcu_receive_time_us = 2'000'000U + i * 100'000U + 500U;
        response.mcu_transmit_time_us = response.mcu_receive_time_us + 20U;
        sync.observe_sync(response, response.host_transmit_time_ns + 1'020'000ULL);
    }
    assert(sync.valid());
    uint64_t mapped_us = 0;
    assert(sync.host_to_mcu((host_base_ns + 500'000'000ULL) * 1.0e-9, &mapped_us));
    assert(std::llabs(static_cast<int64_t>(mapped_us) - 2'500'000LL) < 1000);
    assert(sync.uncertainty_s() <= 0.0005);

    nxv::ClockSynchronizer wrapping_clock;
    const uint64_t before_wrap = wrapping_clock.observe_mcu_time(0xFFFFFF00U);
    const uint64_t after_wrap = wrapping_clock.observe_mcu_time(0x00000100U);
    assert(after_wrap - before_wrap == 512U);

    nxv::AttitudeHistory history;
    nxv::AttitudeSample first;
    first.mcu_time_us = 1000;
    first.quaternion = yaw_quaternion(0.0);
    first.gyro_rad_s = {0.0, 1.0, 0.0};
    nxv::AttitudeSample second = first;
    second.mcu_time_us = 2000;
    second.quaternion = yaw_quaternion(0.1);
    history.push(first);
    history.push(second);
    nxv::AttitudeSample interpolated;
    assert(history.sample_at(1500, 2000, &interpolated));
    assert(std::abs(interpolated.quaternion[0] - std::cos(0.025)) < 1.0e-6);
    assert(history.sample_at(2500, 2000, &interpolated));
    assert(interpolated.mcu_time_us == 2500U);
    assert(!history.sample_at(5000, 2000, &interpolated));

    const double fixed_yaw = 20.0 * kPi / 180.0;
    const double fixed_pitch = -5.0 * kPi / 180.0;
    const cv::Vec3d world(std::sin(fixed_yaw) * std::cos(fixed_pitch),
                          -std::sin(fixed_pitch),
                          std::cos(fixed_yaw) * std::cos(fixed_pitch));
    for (double body_yaw_deg : {-30.0, -10.0, 0.0, 15.0, 35.0}) {
        const auto q = yaw_quaternion(body_yaw_deg * kPi / 180.0);
        const cv::Vec3d camera = rotate_world_to_body(q, world);
        nxv::AimResult aim;
        aim.valid = true;
        aim.yaw_delta_deg = std::atan2(camera[0], camera[2]) * 180.0 / kPi;
        aim.pitch_delta_deg = std::atan2(-camera[1], std::hypot(camera[0], camera[2])) * 180.0 / kPi;
        double recovered_yaw = 0.0;
        double recovered_pitch = 0.0;
        assert(nxv::camera_aim_to_world(aim, q, cv::Matx33d::eye(),
                                       &recovered_yaw, &recovered_pitch));
        assert(std::abs(std::remainder(recovered_yaw - fixed_yaw, 2.0 * kPi)) < 1.0e-9);
        assert(std::abs(recovered_pitch - fixed_pitch) < 1.0e-9);
    }

    nxv::ControlConfig filter_config;
    nxv::WorldTargetFilter filter(filter_config);
    assert(filter.update(10.0, 0.2, -0.1, 0.002));
    assert(!filter.update(10.01, 1.5, -0.1, 0.002));
    auto prediction = filter.predict(10.1);
    assert(prediction.valid && prediction.target_lost);
    prediction = filter.predict(11.01);
    assert(!prediction.valid);
    assert(!filter.update(11.02, 0.4, -0.2, 0.002));
    assert(!filter.update(11.04, 0.401, -0.201, 0.002));
    assert(filter.update(11.06, 0.399, -0.199, 0.002));
    prediction = filter.predict(11.07);
    assert(prediction.valid);

    nxv::ReferenceMpcAxis mpc(1.0, 2.0, 10.0, false);
    mpc.reset(0.0);
    double previous_accel = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const auto reference = mpc.step(0.5, 0.0);
        assert(std::abs(reference.velocity_rad_s) <= 1.000001);
        assert(std::abs(reference.acceleration_rad_s2) <= 2.000001);
        assert(std::abs(reference.acceleration_rad_s2 - previous_accel) <= 0.010001);
        previous_accel = reference.acceleration_rad_s2;
    }
    nxv::ReferenceMpcAxis wrapped_mpc(1.0, 2.0, 10.0, true);
    wrapped_mpc.reset(kPi - 0.01);
    const auto wrapped_reference = wrapped_mpc.step(-kPi + 0.01, 0.0);
    assert(std::abs(std::remainder(wrapped_reference.position_rad -
                                   (kPi - 0.01), 2.0 * kPi)) < 0.001);
    return 0;
}
