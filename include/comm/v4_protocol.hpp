#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace nxv::v4 {

constexpr uint16_t kMagic = 0xA55A;
constexpr uint8_t kVersion = 4;
constexpr std::size_t kHeaderSize = 10;
constexpr std::size_t kCrcSize = 4;
constexpr std::size_t kMaxPayloadSize = 96;

enum class MessageType : uint8_t {
    Hello = 1,
    HelloAck = 2,
    SyncRequest = 3,
    SyncResponse = 4,
    ControlSetpoint = 5,
    AttitudeState = 6,
    FaultStatus = 7,
};

enum ControlFlags : uint16_t {
    ControlValid = 1u << 0,
    TargetLost = 1u << 1,
    VelocityFeedforward = 1u << 2,
    TorqueFeedforward = 1u << 3,
};

enum AttitudeFlags : uint16_t {
    ImuReady = 1u << 0,
    CalibrationValid = 1u << 1,
    CommandQueueUnderflow = 1u << 2,
    ControlActive = 1u << 3,
};

struct Frame {
    MessageType type = MessageType::Hello;
    uint32_t sequence = 0;
    std::vector<uint8_t> payload;
};

struct ControlSetpoint {
    uint32_t execute_time_us = 0;
    int32_t yaw_position_urad = 0;
    int32_t yaw_velocity_urad_s = 0;
    int16_t yaw_torque_1e4_nm = 0;
    int32_t pitch_position_urad = 0;
    int32_t pitch_velocity_urad_s = 0;
    int16_t pitch_torque_1e4_nm = 0;
    uint16_t flags = 0;
};

struct AttitudeState {
    uint32_t sample_time_us = 0;
    std::array<int32_t, 4> quaternion_q30 {};
    std::array<int16_t, 3> gyro_mrad_s {};
    int32_t yaw_position_urad = 0;
    int32_t pitch_position_urad = 0;
    int32_t yaw_velocity_urad_s = 0;
    int32_t pitch_velocity_urad_s = 0;
    uint32_t last_control_sequence = 0;
    uint16_t flags = 0;
    uint8_t queue_depth = 0;
};

struct SyncRequest {
    uint64_t host_transmit_time_ns = 0;
};

struct SyncResponse {
    uint64_t host_transmit_time_ns = 0;
    uint32_t mcu_receive_time_us = 0;
    uint32_t mcu_transmit_time_us = 0;
};

struct FaultStatus {
    uint32_t mcu_time_us = 0;
    uint32_t fault_bits = 0;
    uint32_t detail = 0;
};

uint32_t crc32c(const uint8_t *data, std::size_t size);
std::vector<uint8_t> encode_frame(MessageType type,
                                  uint32_t sequence,
                                  const std::vector<uint8_t> &payload);
std::vector<uint8_t> encode_control(uint32_t sequence, const ControlSetpoint &setpoint);
std::vector<uint8_t> encode_sync_request(uint32_t sequence, const SyncRequest &request);
std::vector<uint8_t> encode_hello(uint32_t sequence, uint32_t capabilities = 0);

bool decode_control(const Frame &frame, ControlSetpoint *setpoint);
bool decode_attitude(const Frame &frame, AttitudeState *state);
bool decode_sync_request(const Frame &frame, SyncRequest *request);
bool decode_sync_response(const Frame &frame, SyncResponse *response);
bool decode_fault_status(const Frame &frame, FaultStatus *status);

class StreamDecoder {
public:
    std::vector<Frame> push(const uint8_t *data, std::size_t size);
    uint64_t crc_error_count() const { return crc_error_count_; }
    uint64_t framing_error_count() const { return framing_error_count_; }

private:
    std::vector<uint8_t> buffer_;
    uint64_t crc_error_count_ = 0;
    uint64_t framing_error_count_ = 0;
};

}  // namespace nxv::v4
