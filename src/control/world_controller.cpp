#include "control/world_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace nxv {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double value)
{
    return std::remainder(value, 2.0 * kPi);
}

double angle_error(double target, double current, bool wrap)
{
    return wrap ? wrap_pi(target - current) : target - current;
}

cv::Vec3d rotate_body_to_world(const std::array<double, 4> &q, const cv::Vec3d &body)
{
    const cv::Vec3d qv(q[1], q[2], q[3]);
    const cv::Vec3d t = 2.0 * qv.cross(body);
    return body + q[0] * t + qv.cross(t);
}

int32_t to_i32_scaled(double value, double scale)
{
    if (!std::isfinite(value)) return 0;
    const double scaled = std::clamp(value * scale,
                                     static_cast<double>(std::numeric_limits<int32_t>::min()),
                                     static_cast<double>(std::numeric_limits<int32_t>::max()));
    return static_cast<int32_t>(std::llround(scaled));
}

int16_t torque_to_wire(double torque_nm)
{
    const double scaled = std::clamp(torque_nm * 10000.0, -32768.0, 32767.0);
    return static_cast<int16_t>(std::lround(scaled));
}

void configure_control_thread(const ControlConfig &config)
{
#if defined(__linux__)
    if (config.cpu_affinity >= 0 && config.cpu_affinity < CPU_SETSIZE) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(config.cpu_affinity, &set);
        (void)pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    }
    if (config.realtime_priority > 0) {
        sched_param parameters {};
        parameters.sched_priority = std::min(config.realtime_priority,
                                             sched_get_priority_max(SCHED_FIFO));
        (void)pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
    }
#else
    (void)config;
#endif
}

}  // namespace

bool camera_aim_to_world(const AimResult &aim,
                         const std::array<double, 4> &q,
                         const cv::Matx33d &body_from_camera,
                         double *yaw,
                         double *pitch)
{
    if (!aim.valid || yaw == nullptr || pitch == nullptr) return false;
    const double camera_yaw = aim.yaw_delta_deg * kPi / 180.0;
    const double camera_pitch = aim.pitch_delta_deg * kPi / 180.0;
    const double cp = std::cos(camera_pitch);
    const cv::Vec3d camera(std::sin(camera_yaw) * cp,
                           -std::sin(camera_pitch),
                           std::cos(camera_yaw) * cp);
    const cv::Vec3d body = body_from_camera * camera;
    const cv::Vec3d world = rotate_body_to_world(q, body);
    const double horizontal = std::hypot(world[0], world[2]);
    const double norm = std::hypot(horizontal, world[1]);
    if (!(norm > 1.0e-9) || !std::isfinite(norm)) return false;
    *yaw = wrap_pi(std::atan2(world[0], world[2]));
    *pitch = std::atan2(-world[1], horizontal);
    return std::isfinite(*yaw) && std::isfinite(*pitch);
}

WorldTargetFilter::WorldTargetFilter(ControlConfig config)
    : config_(std::move(config))
{
    yaw_.wrap = true;
}

void WorldTargetFilter::reset()
{
    yaw_ = {};
    yaw_.wrap = true;
    pitch_ = {};
    last_measurement_s_ = -1.0;
    reacquire_count_ = 0;
}

void WorldTargetFilter::predict_axis(Axis *a, double timestamp_s, double q)
{
    if (!a->initialized || timestamp_s <= a->timestamp_s) return;
    const double dt = std::min(0.25, timestamp_s - a->timestamp_s);
    a->angle += a->rate * dt;
    if (a->wrap) a->angle = wrap_pi(a->angle);
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double p00 = a->p00 + dt * (a->p10 + a->p01) + dt2 * a->p11 + 0.25 * dt4 * q;
    const double p01 = a->p01 + dt * a->p11 + 0.5 * dt3 * q;
    const double p10 = a->p10 + dt * a->p11 + 0.5 * dt3 * q;
    const double p11 = a->p11 + dt2 * q;
    a->p00 = p00;
    a->p01 = p01;
    a->p10 = p10;
    a->p11 = p11;
    a->timestamp_s = timestamp_s;
}

bool WorldTargetFilter::update_axis(Axis *a,
                                    double timestamp_s,
                                    double measurement,
                                    double variance,
                                    double acceleration_variance)
{
    if (!a->initialized) {
        a->initialized = true;
        a->angle = a->wrap ? wrap_pi(measurement) : measurement;
        a->rate = 0.0;
        a->p00 = variance;
        a->p11 = acceleration_variance;
        a->timestamp_s = timestamp_s;
        return true;
    }
    predict_axis(a, timestamp_s, acceleration_variance);
    const double residual = angle_error(measurement, a->angle, a->wrap);
    const double s = a->p00 + variance;
    const double threshold = std::max(3.0 * kPi / 180.0, 4.0 * std::sqrt(std::max(s, 0.0)));
    if (std::abs(residual) > threshold || s <= 1.0e-12) return false;
    const double k0 = a->p00 / s;
    const double k1 = a->p10 / s;
    a->angle += k0 * residual;
    a->rate += k1 * residual;
    if (a->wrap) a->angle = wrap_pi(a->angle);
    const double p00 = a->p00;
    const double p01 = a->p01;
    a->p00 -= k0 * p00;
    a->p01 -= k0 * p01;
    a->p10 -= k1 * p00;
    a->p11 -= k1 * p01;
    return true;
}

bool WorldTargetFilter::update(double timestamp_s,
                               double yaw,
                               double pitch,
                               double measurement_std)
{
    if (!std::isfinite(timestamp_s) || !std::isfinite(yaw) || !std::isfinite(pitch)) return false;
    const double age = last_measurement_s_ < 0.0 ? 0.0 : timestamp_s - last_measurement_s_;
    if (yaw_.initialized && age > config_.target_predict_full_s) {
        const bool consistent = reacquire_count_ == 0 ||
            (std::abs(wrap_pi(yaw - reacquire_yaw_)) < 2.0 * kPi / 180.0 &&
             std::abs(pitch - reacquire_pitch_) < 2.0 * kPi / 180.0);
        reacquire_count_ = consistent ? reacquire_count_ + 1 : 1;
        reacquire_yaw_ = yaw;
        reacquire_pitch_ = pitch;
        if (reacquire_count_ < 3) return false;
        reset();
    }
    const double variance = std::max(1.0e-8, measurement_std * measurement_std);
    const double accel_std = 60.0 * kPi / 180.0;
    Axis yaw_candidate = yaw_;
    Axis pitch_candidate = pitch_;
    const bool yaw_ok = update_axis(&yaw_candidate, timestamp_s, yaw, variance, accel_std * accel_std);
    const bool pitch_ok = update_axis(&pitch_candidate, timestamp_s, pitch, variance, accel_std * accel_std);
    if (!yaw_ok || !pitch_ok) return false;
    yaw_ = yaw_candidate;
    pitch_ = pitch_candidate;
    last_measurement_s_ = timestamp_s;
    reacquire_count_ = 0;
    return true;
}

WorldAimResult WorldTargetFilter::predict(double timestamp_s) const
{
    WorldAimResult out;
    if (!yaw_.initialized || !pitch_.initialized || last_measurement_s_ < 0.0) {
        out.failure_reason = "world filter not initialized";
        return out;
    }
    const double age = std::max(0.0, timestamp_s - last_measurement_s_);
    if (age > config_.target_hold_end_s) {
        out.target_lost = true;
        out.failure_reason = "target loss timeout";
        return out;
    }
    double rate_scale = 1.0;
    double predict_dt = std::min(age, config_.target_predict_full_s);
    if (age > config_.target_predict_full_s) {
        if (age < config_.target_velocity_decay_end_s) {
            rate_scale = 1.0 - (age - config_.target_predict_full_s) /
                (config_.target_velocity_decay_end_s - config_.target_predict_full_s);
            const double decay_span = age - config_.target_predict_full_s;
            predict_dt += 0.5 * decay_span * (1.0 + rate_scale);
        } else {
            const double decay_span = config_.target_velocity_decay_end_s - config_.target_predict_full_s;
            predict_dt += 0.5 * decay_span;
            rate_scale = 0.0;
        }
    }
    out.valid = true;
    out.target_lost = age > 0.05;
    out.timestamp_s = timestamp_s;
    out.yaw_rad = wrap_pi(yaw_.angle + yaw_.rate * predict_dt);
    out.pitch_rad = pitch_.angle + pitch_.rate * predict_dt;
    out.yaw_rate_rad_s = yaw_.rate * rate_scale;
    out.pitch_rate_rad_s = pitch_.rate * rate_scale;
    return out;
}

ReferenceMpcAxis::ReferenceMpcAxis(double max_rate,
                                   double max_accel,
                                   double max_jerk,
                                   bool wrap)
    : max_rate_(std::abs(max_rate)),
      max_accel_(std::abs(max_accel)),
      max_jerk_(std::abs(max_jerk)),
      wrap_(wrap)
{
    calculate_gain(0.001);
}

double ReferenceMpcAxis::wrap_pi(double value)
{
    return std::remainder(value, 2.0 * kPi);
}

void ReferenceMpcAxis::calculate_gain(double dt)
{
    double p00 = 1000.0;
    double p01 = 0.0;
    double p10 = 0.0;
    double p11 = 10.0;
    const double q00 = 100.0;
    const double q11 = 1.0;
    const double r = 0.02;
    for (int step = 0; step < 50; ++step) {
        const double b0 = 0.5 * dt * dt;
        const double b1 = dt;
        const double pb0 = p00 * b0 + p01 * b1;
        const double pb1 = p10 * b0 + p11 * b1;
        const double s = r + b0 * pb0 + b1 * pb1;
        const double ba0 = b0 * (p00 + p01 * 0.0) + b1 * (p10 + p11 * 0.0);
        const double ba1 = b0 * (p00 * dt + p01) + b1 * (p10 * dt + p11);
        gain_position_ = ba0 / s;
        gain_velocity_ = ba1 / s;

        const double a00 = 1.0;
        const double a01 = dt;
        const double a10 = 0.0;
        const double a11 = 1.0;
        const double ap00 = p00;
        const double ap01 = p01;
        const double ap10 = dt * p00 + p10;
        const double ap11 = dt * p01 + p11;
        const double ata00 = ap00;
        const double ata01 = ap00 * dt + ap01;
        const double ata10 = ap10;
        const double ata11 = ap10 * dt + ap11;
        const double k0 = gain_position_;
        const double k1 = gain_velocity_;
        p00 = q00 + ata00 - ba0 * k0;
        p01 = ata01 - ba0 * k1;
        p10 = ata10 - ba1 * k0;
        p11 = q11 + ata11 - ba1 * k1;
        (void)a00; (void)a01; (void)a10; (void)a11;
    }
}

void ReferenceMpcAxis::reset(double position)
{
    initialized_ = true;
    position_ = wrap_ ? wrap_pi(position) : position;
    velocity_ = 0.0;
    acceleration_ = 0.0;
}

MpcReference ReferenceMpcAxis::step(double target_position, double target_velocity, double dt)
{
    if (!initialized_) reset(target_position);
    double error_position = position_ - target_position;
    if (wrap_) error_position = wrap_pi(error_position);
    const double error_velocity = velocity_ - target_velocity;
    double desired_accel = -(gain_position_ * error_position + gain_velocity_ * error_velocity);
    desired_accel = std::clamp(desired_accel, -max_accel_, max_accel_);
    const double max_delta = max_jerk_ * dt;
    acceleration_ += std::clamp(desired_accel - acceleration_, -max_delta, max_delta);
    if ((velocity_ >= max_rate_ && acceleration_ > 0.0) ||
        (velocity_ <= -max_rate_ && acceleration_ < 0.0)) acceleration_ = 0.0;
    position_ += velocity_ * dt + 0.5 * acceleration_ * dt * dt;
    velocity_ = std::clamp(velocity_ + acceleration_ * dt, -max_rate_, max_rate_);
    if (wrap_) position_ = wrap_pi(position_);
    if (std::abs(error_position) < 1.0e-6 && std::abs(error_velocity) < 1.0e-5) {
        position_ = wrap_ ? wrap_pi(target_position) : target_position;
        velocity_ = target_velocity;
        acceleration_ = 0.0;
    }
    return {position_, velocity_, acceleration_};
}

WorldController::WorldController(RuntimeConfig config, GimbalLink *link)
    : config_(std::move(config)),
      link_(link),
      filter_(config_.control),
      yaw_mpc_(config_.control.yaw_max_rate_rad_s,
               config_.control.max_accel_rad_s2,
               config_.control.max_jerk_rad_s3,
               true),
      pitch_mpc_(config_.control.pitch_max_rate_rad_s,
                 config_.control.max_accel_rad_s2,
                 config_.control.max_jerk_rad_s3,
                 false)
{
}

WorldController::~WorldController()
{
    stop();
}

double WorldController::steady_now_s()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void WorldController::start()
{
    if (running_ || link_ == nullptr || !link_->is_v4() || !config_.control.enabled) return;
    running_ = true;
    thread_ = std::thread(&WorldController::control_loop, this);
}

void WorldController::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void WorldController::submit_vision(const FrameBundle &frame, const PipelineResult &result)
{
    if (!result.aim.valid || link_ == nullptr || !link_->synchronized()) return;
    const double exposure_s = frame.exposure_midpoint_s > 0.0
                                  ? frame.exposure_midpoint_s
                                  : frame.timestamp_s;
    if (config_.app.require_precise_timestamps &&
        frame.timestamp_quality < TimestampQuality::ExposureMidpoint) return;
    if (config_.app.require_precise_timestamps && !link_->precise_timing()) return;
    AttitudeSample attitude;
    double time_error_s = 0.0;
    if (!link_->attitude_at_host_time(exposure_s, &attitude, &time_error_s) ||
        (attitude.flags & v4::CalibrationValid) == 0U) return;
    double yaw = 0.0;
    double pitch = 0.0;
    if (!camera_aim_to_world(result.aim,
                             attitude.quaternion,
                             config_.control.body_from_camera,
                             &yaw,
                             &pitch)) return;
    const double quality = std::clamp(result.rectangle.score / 150.0, 0.2, 1.0);
    double measurement_std = (0.15 * kPi / 180.0) / quality;
    measurement_std *= std::clamp(1.0 + result.pose.reprojection_error_px / 2.0,
                                  1.0,
                                  4.0);
    measurement_std += std::abs(time_error_s) * std::hypot(attitude.yaw_rate_rad_s,
                                                           attitude.pitch_rate_rad_s);
    measurement_std += frame.camera_mapping_error_s *
        std::hypot(attitude.yaw_rate_rad_s, attitude.pitch_rate_rad_s);
    if (!result.depth.valid) measurement_std *= 1.25;
    std::lock_guard<std::mutex> lock(filter_mutex_);
    if (filter_.update(exposure_s, yaw, pitch, measurement_std)) {
        last_world_measurement_.valid = true;
        last_world_measurement_.timestamp_s = exposure_s;
        last_world_measurement_.yaw_rad = yaw;
        last_world_measurement_.pitch_rad = pitch;
        last_world_measurement_.attitude_error_s = time_error_s;
    }
}

double WorldController::feedforward_torque(bool pitch, const MpcReference &r) const
{
    if (!config_.control.torque_feedforward_enabled) return 0.0;
    const double inertia = pitch ? config_.control.pitch_inertia : config_.control.yaw_inertia;
    const double viscous = pitch ? config_.control.pitch_viscous : config_.control.yaw_viscous;
    const double coulomb = pitch ? config_.control.pitch_coulomb : config_.control.yaw_coulomb;
    const double sign = r.velocity_rad_s > 1.0e-3 ? 1.0 : (r.velocity_rad_s < -1.0e-3 ? -1.0 : 0.0);
    double torque = inertia * r.acceleration_rad_s2 + viscous * r.velocity_rad_s + coulomb * sign;
    if (pitch) torque += config_.control.pitch_gravity *
        std::sin(r.position_rad - config_.control.pitch_gravity_zero_rad);
    return std::clamp(torque, pitch ? -1.0 : -0.5, pitch ? 1.0 : 0.5);
}

void WorldController::control_loop()
{
    using Clock = std::chrono::steady_clock;
    configure_control_thread(config_.control);
    auto next = Clock::now();
    while (running_) {
        next += std::chrono::milliseconds(1);
        const double now_s = steady_now_s();
        const double execute_host_s = now_s + link_->command_lead_s();
        uint64_t execute_mcu_us = 0;
        const bool timing_ok = link_->host_to_mcu(execute_host_s, &execute_mcu_us);
        WorldAimResult target;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            target = filter_.predict(execute_host_s);
        }
        v4::ControlSetpoint packet;
        packet.execute_time_us = static_cast<uint32_t>(execute_mcu_us);
        if (target.valid && timing_ok) {
            if (!yaw_mpc_.initialized() || !pitch_mpc_.initialized()) {
                AttitudeSample current;
                if (link_->attitude_at_host_time(now_s, &current)) {
                    yaw_mpc_.reset(current.yaw_rad);
                    pitch_mpc_.reset(current.pitch_rad);
                } else {
                    yaw_mpc_.reset(target.yaw_rad);
                    pitch_mpc_.reset(target.pitch_rad);
                }
            }
            const MpcReference yaw = yaw_mpc_.step(target.yaw_rad, target.yaw_rate_rad_s);
            const MpcReference pitch = pitch_mpc_.step(target.pitch_rad, target.pitch_rate_rad_s);
            packet.yaw_position_urad = to_i32_scaled(yaw.position_rad, 1.0e6);
            packet.yaw_velocity_urad_s = to_i32_scaled(yaw.velocity_rad_s, 1.0e6);
            packet.pitch_position_urad = to_i32_scaled(pitch.position_rad, 1.0e6);
            packet.pitch_velocity_urad_s = to_i32_scaled(pitch.velocity_rad_s, 1.0e6);
            packet.yaw_torque_1e4_nm = torque_to_wire(feedforward_torque(false, yaw));
            packet.pitch_torque_1e4_nm = torque_to_wire(feedforward_torque(true, pitch));
            packet.flags = v4::ControlValid;
            if (target.target_lost) packet.flags |= v4::TargetLost;
            if (config_.control.velocity_feedforward_enabled) packet.flags |= v4::VelocityFeedforward;
            if (config_.control.torque_feedforward_enabled) packet.flags |= v4::TorqueFeedforward;
            ++sent_count_;
        } else {
            ++invalid_count_;
        }
        link_->send_control(packet);
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            latest_target_ = target;
        }
        std::this_thread::sleep_until(next);
        if (Clock::now() > next + std::chrono::milliseconds(10)) next = Clock::now();
    }
}

WorldAimResult WorldController::latest_world_target() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return latest_target_;
}

std::string WorldController::status_text() const
{
    const WorldAimResult state = latest_world_target();
    std::ostringstream out;
    out << "V4 hs=" << (link_ && link_->handshake_complete() ? 1 : 0)
        << " sync=" << (link_ && link_->synchronized() ? 1 : 0)
        << " precise=" << (link_ && link_->precise_timing() ? 1 : 0)
        << " hist=" << (link_ ? link_->attitude_sample_count() : 0)
        << " rx=" << (link_ ? link_->attitude_rx_count() : 0)
        << " sync_n=" << (link_ ? link_->sync_sample_count() : 0)
        << std::fixed << std::setprecision(3)
        << " rtt_ms=" << (link_ ? link_->minimum_rtt_s() * 1.0e3 : 0.0)
        << " unc95_ms=" << (link_ ? link_->synchronization_uncertainty_s() * 1.0e3 : 0.0)
        << " drift_ppm=" << (link_ ? link_->clock_drift_ppm() : 0.0)
        << " valid=" << (state.valid ? 1 : 0)
        << " faults=0x" << std::hex << (link_ ? link_->last_fault_bits() : 0U)
        << std::dec;
    if (state.valid) {
        out << std::fixed << std::setprecision(3)
            << " world_deg=" << state.yaw_rad * 180.0 / kPi
            << "," << state.pitch_rad * 180.0 / kPi;
    }
    return out.str();
}

}  // namespace nxv
