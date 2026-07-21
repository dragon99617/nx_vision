#include "comm/gimbal_link.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace nxv {

GimbalLink::~GimbalLink()
{
    close();
}

uint64_t GimbalLink::steady_now_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool GimbalLink::open(const SerialConfig &config)
{
    close();
    config_ = config;
    if (!serial_.open(config)) return false;
    if (is_v4() && config.enabled && !config.dry_run) {
        running_ = true;
        receiver_thread_ = std::thread(&GimbalLink::receiver_loop, this);
    }
    return true;
}

void GimbalLink::close()
{
    running_ = false;
    if (receiver_thread_.joinable()) receiver_thread_.join();
    serial_.close();
    handshake_complete_ = false;
    remote_attitude_flags_ = 0;
    remote_queue_depth_ = 0;
    last_fault_bits_ = 0;
    attitude_rx_count_ = 0;
    clock_.reset();
    attitude_history_.clear();
}

bool GimbalLink::write_bytes(const std::vector<uint8_t> &bytes)
{
    if (bytes.empty()) return false;
    std::lock_guard<std::mutex> lock(write_mutex_);
    return serial_.write_line(std::string(reinterpret_cast<const char *>(bytes.data()), bytes.size()));
}

bool GimbalLink::write_legacy(const std::string &packet)
{
    std::lock_guard<std::mutex> lock(write_mutex_);
    return serial_.write_line(packet);
}

bool GimbalLink::send_control(const v4::ControlSetpoint &setpoint, uint32_t *sequence)
{
    const uint32_t seq = tx_sequence_.fetch_add(1);
    if (sequence != nullptr) *sequence = seq;
    return write_bytes(v4::encode_control(seq, setpoint));
}

bool GimbalLink::attitude_at_host_time(double host_s,
                                       AttitudeSample *sample,
                                       double *time_error_s) const
{
    uint64_t mcu_us = 0;
    if (!clock_.host_to_mcu(host_s, &mcu_us)) return false;
    return attitude_history_.sample_at(mcu_us, 2000U, sample, time_error_s);
}

bool GimbalLink::host_to_mcu(double host_s, uint64_t *mcu_time_us) const
{
    return clock_.host_to_mcu(host_s, mcu_time_us);
}

bool GimbalLink::synchronized() const
{
    return handshake_complete_ && clock_.valid() && attitude_history_.size() >= 4U;
}

double GimbalLink::synchronization_uncertainty_s() const
{
    return clock_.uncertainty_s();
}

bool GimbalLink::precise_timing() const
{
    return synchronized() &&
           synchronization_uncertainty_s() <= config_.v4_precise_uncertainty_ms * 1.0e-3;
}

double GimbalLink::command_lead_s() const
{
    const double minimum = std::max(0.001, config_.v4_min_lead_ms * 1.0e-3);
    const double maximum = std::max(minimum, config_.v4_max_lead_ms * 1.0e-3);
    const double measured = clock_.minimum_rtt_s();
    double lead = 0.002 + (std::isfinite(measured) ? measured : 0.0);
    const uint8_t queue_depth = remote_queue_depth_.load();
    const uint16_t flags = remote_attitude_flags_.load();
    if ((flags & v4::CommandQueueUnderflow) != 0U) lead += 0.002;
    else if (queue_depth < 3U) lead += 0.001;
    else if (queue_depth > 12U) lead -= 0.001;
    return std::clamp(lead, minimum, maximum);
}

std::size_t GimbalLink::attitude_sample_count() const
{
    return attitude_history_.size();
}

uint32_t GimbalLink::last_applied_control_sequence() const
{
    return last_applied_sequence_;
}

uint64_t GimbalLink::rx_crc_errors() const
{
    return decoder_.crc_error_count();
}

void GimbalLink::handle_frame(const v4::Frame &frame, uint64_t host_receive_ns)
{
    if (frame.type == v4::MessageType::HelloAck) {
        handshake_complete_ = true;
        return;
    }
    if (frame.type == v4::MessageType::SyncResponse) {
        v4::SyncResponse response;
        if (v4::decode_sync_response(frame, &response)) {
            clock_.observe_sync(response, host_receive_ns);
        }
        return;
    }
    if (frame.type == v4::MessageType::AttitudeState) {
        v4::AttitudeState state;
        if (!v4::decode_attitude(frame, &state)) return;
        const uint64_t timestamp = clock_.observe_mcu_time(state.sample_time_us);
        attitude_history_.push(decode_attitude_sample(state, timestamp));
        last_applied_sequence_ = state.last_control_sequence;
        remote_attitude_flags_ = state.flags;
        remote_queue_depth_ = state.queue_depth;
        ++attitude_rx_count_;
        return;
    }
    if (frame.type == v4::MessageType::FaultStatus) {
        v4::FaultStatus status;
        if (v4::decode_fault_status(frame, &status)) last_fault_bits_ = status.fault_bits;
    }
}

void GimbalLink::receiver_loop()
{
    using Clock = std::chrono::steady_clock;
    std::array<uint8_t, 512> bytes {};
    auto next_sync = Clock::now();
    auto next_hello = Clock::now();
    const auto receiver_started = Clock::now();
    bool no_downlink_warning_reported = false;
    bool read_error_reported = false;
    const auto sync_period = std::chrono::microseconds(
        1'000'000 / std::max(1, config_.v4_sync_hz));

    while (running_) {
        const int count = serial_.read_available(bytes.data(), bytes.size());
        const uint64_t receive_ns = steady_now_ns();
        if (count > 0) {
            for (const v4::Frame &frame : decoder_.push(bytes.data(), static_cast<std::size_t>(count))) {
                handle_frame(frame, receive_ns);
            }
        } else if (count < 0 && !read_error_reported) {
            std::cerr << "[v4] serial read failed; check USB disconnect and device ownership\n";
            read_error_reported = true;
        }

        const auto now = Clock::now();
        if (!handshake_complete_ && now >= next_hello) {
            write_bytes(v4::encode_hello(tx_sequence_.fetch_add(1), 0x7U));
            next_hello = now + std::chrono::milliseconds(250);
        }
        if (now >= next_sync) {
            v4::SyncRequest request;
            request.host_transmit_time_ns = steady_now_ns();
            write_bytes(v4::encode_sync_request(tx_sequence_.fetch_add(1), request));
            next_sync = now + sync_period;
        }
        if (!handshake_complete_ && !no_downlink_warning_reported &&
            now - receiver_started > std::chrono::seconds(2)) {
            std::cerr << "[v4] no downlink frames received for 2 s. Stop any other "
                         "nx_runtime/nx_debug_studio process using the same USB device.\n";
            no_downlink_warning_reported = true;
        }
        if (count <= 0) std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
}

}  // namespace nxv
