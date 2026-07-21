#include "comm/v4_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <type_traits>

namespace nxv::v4 {
namespace {

template <typename T>
void append_le(std::vector<uint8_t> *out, T value)
{
    using U = std::make_unsigned_t<T>;
    const U bits = static_cast<U>(value);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        out->push_back(static_cast<uint8_t>((bits >> (i * 8U)) & 0xFFU));
    }
}

template <typename T>
bool read_le(const std::vector<uint8_t> &data, std::size_t *offset, T *value)
{
    if (offset == nullptr || value == nullptr || *offset + sizeof(T) > data.size()) {
        return false;
    }
    using U = std::make_unsigned_t<T>;
    U bits = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        bits |= static_cast<U>(data[*offset + i]) << (i * 8U);
    }
    *value = static_cast<T>(bits);
    *offset += sizeof(T);
    return true;
}

uint16_t read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t read_u32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

}  // namespace

uint32_t crc32c(const uint8_t *data, std::size_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0x82F63B78U : crc >> 1U;
        }
    }
    return ~crc;
}

std::vector<uint8_t> encode_frame(MessageType type,
                                  uint32_t sequence,
                                  const std::vector<uint8_t> &payload)
{
    if (payload.size() > kMaxPayloadSize ||
        payload.size() > std::numeric_limits<uint16_t>::max()) {
        return {};
    }
    std::vector<uint8_t> frame;
    frame.reserve(kHeaderSize + payload.size() + kCrcSize);
    append_le(&frame, kMagic);
    append_le(&frame, kVersion);
    append_le(&frame, static_cast<uint8_t>(type));
    append_le(&frame, static_cast<uint16_t>(payload.size()));
    append_le(&frame, sequence);
    frame.insert(frame.end(), payload.begin(), payload.end());
    append_le(&frame, crc32c(frame.data(), frame.size()));
    return frame;
}

std::vector<uint8_t> encode_control(uint32_t sequence, const ControlSetpoint &s)
{
    std::vector<uint8_t> payload;
    payload.reserve(28);
    append_le(&payload, s.execute_time_us);
    append_le(&payload, s.yaw_position_urad);
    append_le(&payload, s.yaw_velocity_urad_s);
    append_le(&payload, s.yaw_torque_1e4_nm);
    append_le(&payload, s.pitch_position_urad);
    append_le(&payload, s.pitch_velocity_urad_s);
    append_le(&payload, s.pitch_torque_1e4_nm);
    append_le(&payload, s.flags);
    append_le(&payload, static_cast<uint16_t>(0));
    return encode_frame(MessageType::ControlSetpoint, sequence, payload);
}

std::vector<uint8_t> encode_sync_request(uint32_t sequence, const SyncRequest &request)
{
    std::vector<uint8_t> payload;
    append_le(&payload, request.host_transmit_time_ns);
    return encode_frame(MessageType::SyncRequest, sequence, payload);
}

std::vector<uint8_t> encode_hello(uint32_t sequence, uint32_t capabilities)
{
    std::vector<uint8_t> payload;
    append_le(&payload, capabilities);
    return encode_frame(MessageType::Hello, sequence, payload);
}

bool decode_control(const Frame &frame, ControlSetpoint *s)
{
    if (s == nullptr || frame.type != MessageType::ControlSetpoint || frame.payload.size() != 28U) {
        return false;
    }
    std::size_t at = 0;
    uint16_t reserved = 0;
    return read_le(frame.payload, &at, &s->execute_time_us) &&
           read_le(frame.payload, &at, &s->yaw_position_urad) &&
           read_le(frame.payload, &at, &s->yaw_velocity_urad_s) &&
           read_le(frame.payload, &at, &s->yaw_torque_1e4_nm) &&
           read_le(frame.payload, &at, &s->pitch_position_urad) &&
           read_le(frame.payload, &at, &s->pitch_velocity_urad_s) &&
           read_le(frame.payload, &at, &s->pitch_torque_1e4_nm) &&
           read_le(frame.payload, &at, &s->flags) &&
           read_le(frame.payload, &at, &reserved);
}

bool decode_attitude(const Frame &frame, AttitudeState *s)
{
    if (s == nullptr || frame.type != MessageType::AttitudeState || frame.payload.size() != 50U) {
        return false;
    }
    std::size_t at = 0;
    uint8_t reserved = 0;
    if (!read_le(frame.payload, &at, &s->sample_time_us)) return false;
    for (auto &value : s->quaternion_q30) if (!read_le(frame.payload, &at, &value)) return false;
    for (auto &value : s->gyro_mrad_s) if (!read_le(frame.payload, &at, &value)) return false;
    return read_le(frame.payload, &at, &s->yaw_position_urad) &&
           read_le(frame.payload, &at, &s->pitch_position_urad) &&
           read_le(frame.payload, &at, &s->yaw_velocity_urad_s) &&
           read_le(frame.payload, &at, &s->pitch_velocity_urad_s) &&
           read_le(frame.payload, &at, &s->last_control_sequence) &&
           read_le(frame.payload, &at, &s->flags) &&
           read_le(frame.payload, &at, &s->queue_depth) &&
           read_le(frame.payload, &at, &reserved);
}

bool decode_sync_request(const Frame &frame, SyncRequest *request)
{
    if (request == nullptr || frame.type != MessageType::SyncRequest || frame.payload.size() != 8U) {
        return false;
    }
    std::size_t at = 0;
    return read_le(frame.payload, &at, &request->host_transmit_time_ns);
}

bool decode_sync_response(const Frame &frame, SyncResponse *response)
{
    if (response == nullptr || frame.type != MessageType::SyncResponse || frame.payload.size() != 16U) {
        return false;
    }
    std::size_t at = 0;
    return read_le(frame.payload, &at, &response->host_transmit_time_ns) &&
           read_le(frame.payload, &at, &response->mcu_receive_time_us) &&
           read_le(frame.payload, &at, &response->mcu_transmit_time_us);
}

bool decode_fault_status(const Frame &frame, FaultStatus *status)
{
    if (status == nullptr || frame.type != MessageType::FaultStatus ||
        frame.payload.size() != 12U) return false;
    std::size_t at = 0;
    return read_le(frame.payload, &at, &status->mcu_time_us) &&
           read_le(frame.payload, &at, &status->fault_bits) &&
           read_le(frame.payload, &at, &status->detail);
}

std::vector<Frame> StreamDecoder::push(const uint8_t *data, std::size_t size)
{
    if (data != nullptr && size > 0) {
        buffer_.insert(buffer_.end(), data, data + size);
    }
    std::vector<Frame> frames;
    static constexpr std::array<uint8_t, 2> magic_bytes {0x5A, 0xA5};
    while (buffer_.size() >= kHeaderSize + kCrcSize) {
        auto magic = std::search(buffer_.begin(), buffer_.end(),
                                 magic_bytes.begin(), magic_bytes.end());
        if (magic != buffer_.begin()) {
            if (magic == buffer_.end()) {
                buffer_.erase(buffer_.begin(), buffer_.end() - 1);
                ++framing_error_count_;
                break;
            }
            buffer_.erase(buffer_.begin(), magic);
            ++framing_error_count_;
        }
        if (buffer_.size() < kHeaderSize + kCrcSize) break;
        if (buffer_[2] != kVersion) {
            buffer_.erase(buffer_.begin());
            ++framing_error_count_;
            continue;
        }
        const uint16_t payload_size = read_u16(buffer_.data() + 4);
        if (payload_size > kMaxPayloadSize) {
            buffer_.erase(buffer_.begin());
            ++framing_error_count_;
            continue;
        }
        const std::size_t frame_size = kHeaderSize + payload_size + kCrcSize;
        if (buffer_.size() < frame_size) break;
        const uint32_t received_crc = read_u32(buffer_.data() + frame_size - kCrcSize);
        const uint32_t calculated_crc = crc32c(buffer_.data(), frame_size - kCrcSize);
        if (received_crc != calculated_crc) {
            buffer_.erase(buffer_.begin());
            ++crc_error_count_;
            continue;
        }
        Frame frame;
        frame.type = static_cast<MessageType>(buffer_[3]);
        frame.sequence = read_u32(buffer_.data() + 6);
        frame.payload.assign(buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
                             buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + payload_size));
        frames.push_back(std::move(frame));
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    }
    return frames;
}

}  // namespace nxv::v4
