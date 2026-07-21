#include "comm/time_sync.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nxv {
namespace {

constexpr double kQ30Scale = 1073741824.0;
constexpr uint64_t kWrapUs = uint64_t{1} << 32U;

void normalize(std::array<double, 4> *q)
{
    double norm = 0.0;
    for (double value : *q) norm += value * value;
    norm = std::sqrt(norm);
    if (!(norm > 1.0e-12) || !std::isfinite(norm)) {
        *q = {1.0, 0.0, 0.0, 0.0};
        return;
    }
    for (double &value : *q) value /= norm;
}

std::array<double, 4> slerp(std::array<double, 4> a,
                            std::array<double, 4> b,
                            double alpha)
{
    double dot = 0.0;
    for (std::size_t i = 0; i < 4; ++i) dot += a[i] * b[i];
    if (dot < 0.0) {
        for (double &value : b) value = -value;
        dot = -dot;
    }
    dot = std::clamp(dot, -1.0, 1.0);
    std::array<double, 4> out {};
    if (dot > 0.9995) {
        for (std::size_t i = 0; i < 4; ++i) out[i] = a[i] + alpha * (b[i] - a[i]);
    } else {
        const double angle = std::acos(dot);
        const double sin_angle = std::sin(angle);
        const double wa = std::sin((1.0 - alpha) * angle) / sin_angle;
        const double wb = std::sin(alpha * angle) / sin_angle;
        for (std::size_t i = 0; i < 4; ++i) out[i] = wa * a[i] + wb * b[i];
    }
    normalize(&out);
    return out;
}

std::array<double, 4> extrapolate(const AttitudeSample &sample, double dt_s)
{
    const auto &q = sample.quaternion;
    const double gx = sample.gyro_rad_s[0];
    const double gy = sample.gyro_rad_s[1];
    const double gz = sample.gyro_rad_s[2];
    std::array<double, 4> out {
        q[0] + 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz) * dt_s,
        q[1] + 0.5 * ( q[0] * gx + q[2] * gz - q[3] * gy) * dt_s,
        q[2] + 0.5 * ( q[0] * gy - q[1] * gz + q[3] * gx) * dt_s,
        q[3] + 0.5 * ( q[0] * gz + q[1] * gy - q[2] * gx) * dt_s,
    };
    normalize(&out);
    return out;
}

double lerp(double a, double b, double alpha)
{
    return a + alpha * (b - a);
}

double lerp_angle(double a, double b, double alpha)
{
    return a + alpha * std::remainder(b - a, 2.0 * 3.14159265358979323846);
}

}  // namespace

void ClockSynchronizer::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    points_.clear();
    last_unwrapped_us_ = 0;
    unwrap_initialized_ = false;
    slope_ = 1.0;
    intercept_ = 0.0;
    uncertainty_s_ = 1.0;
    minimum_rtt_s_ = 1.0;
    valid_ = false;
}

uint64_t ClockSynchronizer::unwrap_locked(uint32_t raw)
{
    if (!unwrap_initialized_) {
        last_unwrapped_us_ = raw;
        unwrap_initialized_ = true;
        return last_unwrapped_us_;
    }
    const uint64_t epoch = last_unwrapped_us_ & ~(kWrapUs - 1U);
    uint64_t candidate = epoch | raw;
    if (candidate + kWrapUs / 2U < last_unwrapped_us_) candidate += kWrapUs;
    if (candidate > last_unwrapped_us_ + kWrapUs / 2U && candidate >= kWrapUs) candidate -= kWrapUs;
    if (candidate > last_unwrapped_us_) last_unwrapped_us_ = candidate;
    return candidate;
}

uint64_t ClockSynchronizer::observe_mcu_time(uint32_t raw_time_us)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return unwrap_locked(raw_time_us);
}

void ClockSynchronizer::observe_sync(const v4::SyncResponse &r, uint64_t host_rx_ns)
{
    if (host_rx_ns <= r.host_transmit_time_ns) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const uint32_t mcu_processing_us = r.mcu_transmit_time_us - r.mcu_receive_time_us;
    const double gross_rtt_s = static_cast<double>(host_rx_ns - r.host_transmit_time_ns) * 1.0e-9;
    const double network_rtt_s = std::max(0.0, gross_rtt_s - mcu_processing_us * 1.0e-6);
    minimum_rtt_s_ = std::min(minimum_rtt_s_, network_rtt_s);
    if (network_rtt_s > minimum_rtt_s_ + 0.0015) return;

    const uint32_t midpoint_raw = r.mcu_receive_time_us + mcu_processing_us / 2U;
    const uint64_t midpoint_us = unwrap_locked(midpoint_raw);
    const double host_mid_s = 0.5 * static_cast<double>(r.host_transmit_time_ns + host_rx_ns) * 1.0e-9;
    points_.push_back({host_mid_s, midpoint_us * 1.0e-6, network_rtt_s});
    while (points_.size() > 64U) points_.pop_front();
    fit_locked();
}

void ClockSynchronizer::fit_locked()
{
    if (points_.size() < 4U) {
        valid_ = false;
        return;
    }
    const auto fit = [](const std::deque<Point> &points,
                        const std::vector<bool> *selected,
                        double *slope,
                        double *intercept) -> std::size_t {
        double mean_h = 0.0;
        double mean_m = 0.0;
        std::size_t count = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (selected != nullptr && !(*selected)[i]) continue;
            mean_h += points[i].host_s;
            mean_m += points[i].mcu_s;
            ++count;
        }
        if (count < 2U) return 0U;
        mean_h /= static_cast<double>(count);
        mean_m /= static_cast<double>(count);
        double cov = 0.0;
        double var = 0.0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (selected != nullptr && !(*selected)[i]) continue;
            cov += (points[i].host_s - mean_h) * (points[i].mcu_s - mean_m);
            var += (points[i].host_s - mean_h) * (points[i].host_s - mean_h);
        }
        *slope = var > 1.0e-12 ? cov / var : 1.0;
        *intercept = mean_m - *slope * mean_h;
        return count;
    };

    double initial_slope = 1.0;
    double initial_intercept = 0.0;
    (void)fit(points_, nullptr, &initial_slope, &initial_intercept);
    std::vector<double> residuals;
    residuals.reserve(points_.size());
    for (const Point &p : points_) {
        residuals.push_back(std::abs(p.mcu_s -
                                    (initial_slope * p.host_s + initial_intercept)));
    }
    std::vector<double> sorted = residuals;
    const auto middle = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2U);
    std::nth_element(sorted.begin(), middle, sorted.end());
    const double robust_limit = std::max(0.00010, 3.0 * *middle);
    std::vector<bool> selected(points_.size(), false);
    for (std::size_t i = 0; i < residuals.size(); ++i) selected[i] = residuals[i] <= robust_limit;
    const std::size_t used = fit(points_, &selected, &slope_, &intercept_);
    if (used < 4U) {
        valid_ = false;
        return;
    }
    if (slope_ < 0.999 || slope_ > 1.001) {
        valid_ = false;
        return;
    }
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < points_.size(); ++i) {
        if (!selected[i]) continue;
        const Point &p = points_[i];
        const double residual = p.mcu_s - (slope_ * p.host_s + intercept_);
        sum_sq += residual * residual;
    }
    const double rms = std::sqrt(sum_sq / static_cast<double>(used));
    uncertainty_s_ = std::max(minimum_rtt_s_ * 0.5, rms * 1.96);
    valid_ = used >= 6U && uncertainty_s_ <= 0.0005;
}

bool ClockSynchronizer::host_to_mcu(double host_s, uint64_t *mcu_us) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_ || mcu_us == nullptr) return false;
    const double value = (slope_ * host_s + intercept_) * 1.0e6;
    if (value < 0.0 || !std::isfinite(value)) return false;
    *mcu_us = static_cast<uint64_t>(std::llround(value));
    return true;
}

bool ClockSynchronizer::mcu_to_host(uint64_t mcu_us, double *host_s) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_ || host_s == nullptr || std::abs(slope_) < 1.0e-12) return false;
    *host_s = (mcu_us * 1.0e-6 - intercept_) / slope_;
    return true;
}

bool ClockSynchronizer::valid() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return valid_;
}

double ClockSynchronizer::uncertainty_s() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return uncertainty_s_;
}

double ClockSynchronizer::minimum_rtt_s() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return minimum_rtt_s_;
}

void AttitudeHistory::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
}

void AttitudeHistory::push(const AttitudeSample &sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!samples_.empty() && sample.mcu_time_us <= samples_.back().mcu_time_us) return;
    samples_.push_back(sample);
    while (samples_.size() > 4096U) samples_.pop_front();
}

bool AttitudeHistory::sample_at(uint64_t timestamp,
                                uint64_t max_extrapolation_us,
                                AttitudeSample *out,
                                double *time_error_s) const
{
    if (out == nullptr) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) return false;
    auto upper = std::lower_bound(samples_.begin(), samples_.end(), timestamp,
                                  [](const AttitudeSample &s, uint64_t t) {
                                      return s.mcu_time_us < t;
                                  });
    if (upper == samples_.begin()) {
        const uint64_t gap = upper->mcu_time_us - timestamp;
        if (gap > max_extrapolation_us) return false;
        *out = *upper;
        out->quaternion = extrapolate(*upper, -static_cast<double>(gap) * 1.0e-6);
        out->mcu_time_us = timestamp;
        if (time_error_s) *time_error_s = gap * 1.0e-6;
        return true;
    }
    if (upper == samples_.end()) {
        const AttitudeSample &last = samples_.back();
        const uint64_t gap = timestamp - last.mcu_time_us;
        if (gap > max_extrapolation_us) return false;
        *out = last;
        out->quaternion = extrapolate(last, gap * 1.0e-6);
        out->mcu_time_us = timestamp;
        if (time_error_s) *time_error_s = gap * 1.0e-6;
        return true;
    }
    const AttitudeSample &b = *upper;
    const AttitudeSample &a = *(upper - 1);
    const uint64_t span = b.mcu_time_us - a.mcu_time_us;
    if (span == 0 || span > 10000U) return false;
    const double alpha = static_cast<double>(timestamp - a.mcu_time_us) / span;
    *out = a;
    out->mcu_time_us = timestamp;
    out->quaternion = slerp(a.quaternion, b.quaternion, alpha);
    for (std::size_t i = 0; i < 3; ++i) out->gyro_rad_s[i] = lerp(a.gyro_rad_s[i], b.gyro_rad_s[i], alpha);
    out->yaw_rad = lerp_angle(a.yaw_rad, b.yaw_rad, alpha);
    out->pitch_rad = lerp(a.pitch_rad, b.pitch_rad, alpha);
    out->yaw_rate_rad_s = lerp(a.yaw_rate_rad_s, b.yaw_rate_rad_s, alpha);
    out->pitch_rate_rad_s = lerp(a.pitch_rate_rad_s, b.pitch_rate_rad_s, alpha);
    if (time_error_s) *time_error_s = 0.0;
    return true;
}

std::size_t AttitudeHistory::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

AttitudeSample decode_attitude_sample(const v4::AttitudeState &s, uint64_t timestamp)
{
    AttitudeSample out;
    out.mcu_time_us = timestamp;
    for (std::size_t i = 0; i < 4; ++i) out.quaternion[i] = s.quaternion_q30[i] / kQ30Scale;
    normalize(&out.quaternion);
    for (std::size_t i = 0; i < 3; ++i) out.gyro_rad_s[i] = s.gyro_mrad_s[i] * 1.0e-3;
    out.yaw_rad = s.yaw_position_urad * 1.0e-6;
    out.pitch_rad = s.pitch_position_urad * 1.0e-6;
    out.yaw_rate_rad_s = s.yaw_velocity_urad_s * 1.0e-6;
    out.pitch_rate_rad_s = s.pitch_velocity_urad_s * 1.0e-6;
    out.flags = s.flags;
    return out;
}

}  // namespace nxv
