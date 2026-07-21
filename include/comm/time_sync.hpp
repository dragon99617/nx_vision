#pragma once

#include "comm/v4_protocol.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <mutex>

namespace nxv {

struct AttitudeSample {
    uint64_t mcu_time_us = 0;
    std::array<double, 4> quaternion {1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> gyro_rad_s {};
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rate_rad_s = 0.0;
    double pitch_rate_rad_s = 0.0;
    uint16_t flags = 0;
};

class ClockSynchronizer {
public:
    void reset();
    uint64_t observe_mcu_time(uint32_t raw_time_us);
    void observe_sync(const v4::SyncResponse &response, uint64_t host_receive_time_ns);
    bool host_to_mcu(double host_steady_s, uint64_t *mcu_time_us) const;
    bool mcu_to_host(uint64_t mcu_time_us, double *host_steady_s) const;
    bool valid() const;
    double uncertainty_s() const;
    double minimum_rtt_s() const;
    double drift_ppm() const;
    std::size_t sync_sample_count() const;

private:
    struct Point {
        double host_s = 0.0;
        double mcu_s = 0.0;
        double rtt_s = 0.0;
    };

    uint64_t unwrap_locked(uint32_t raw_time_us);
    void fit_locked();

    mutable std::mutex mutex_;
    std::deque<Point> points_;
    uint64_t last_unwrapped_us_ = 0;
    bool unwrap_initialized_ = false;
    double slope_ = 1.0;
    double intercept_ = 0.0;
    double uncertainty_s_ = 1.0;
    double minimum_rtt_s_ = 1.0;
    bool valid_ = false;
};

class AttitudeHistory {
public:
    void clear();
    void push(const AttitudeSample &sample);
    bool sample_at(uint64_t mcu_time_us,
                   uint64_t max_extrapolation_us,
                   AttitudeSample *sample,
                   double *time_error_s = nullptr) const;
    std::size_t size() const;

private:
    mutable std::mutex mutex_;
    std::deque<AttitudeSample> samples_;
};

AttitudeSample decode_attitude_sample(const v4::AttitudeState &state,
                                      uint64_t unwrapped_mcu_time_us);

}  // namespace nxv
