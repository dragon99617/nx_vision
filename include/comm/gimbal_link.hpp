#pragma once

#include "comm/serial_port.hpp"
#include "comm/time_sync.hpp"
#include "comm/v4_protocol.hpp"
#include "common/config.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace nxv {

class GimbalLink {
public:
    GimbalLink() = default;
    ~GimbalLink();

    bool open(const SerialConfig &config);
    void close();
    bool is_v4() const { return config_.protocol == "v4"; }
    bool write_legacy(const std::string &packet);
    bool send_control(const v4::ControlSetpoint &setpoint, uint32_t *sequence = nullptr);

    bool attitude_at_host_time(double host_steady_s,
                               AttitudeSample *sample,
                               double *time_error_s = nullptr) const;
    bool host_to_mcu(double host_steady_s, uint64_t *mcu_time_us) const;
    bool synchronized() const;
    double synchronization_uncertainty_s() const;
    double command_lead_s() const;
    std::size_t attitude_sample_count() const;
    uint32_t last_applied_control_sequence() const;
    uint64_t rx_crc_errors() const;
    uint32_t last_fault_bits() const { return last_fault_bits_.load(); }

private:
    static uint64_t steady_now_ns();
    void receiver_loop();
    bool write_bytes(const std::vector<uint8_t> &bytes);
    void handle_frame(const v4::Frame &frame, uint64_t host_receive_ns);

    SerialConfig config_;
    SerialPort serial_;
    mutable ClockSynchronizer clock_;
    AttitudeHistory attitude_history_;
    v4::StreamDecoder decoder_;
    std::atomic<bool> running_ {false};
    std::atomic<bool> handshake_complete_ {false};
    std::atomic<uint32_t> tx_sequence_ {1};
    std::atomic<uint32_t> last_applied_sequence_ {0};
    std::atomic<uint16_t> remote_attitude_flags_ {0};
    std::atomic<uint8_t> remote_queue_depth_ {0};
    std::atomic<uint32_t> last_fault_bits_ {0};
    std::thread receiver_thread_;
    std::mutex write_mutex_;
};

}  // namespace nxv
