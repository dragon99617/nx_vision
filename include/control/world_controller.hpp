#pragma once

#include "comm/gimbal_link.hpp"
#include "common/config.hpp"
#include "common/types.hpp"

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace nxv {

bool camera_aim_to_world(const AimResult &aim,
                         const std::array<double, 4> &body_to_world,
                         const cv::Matx33d &body_from_camera,
                         double *yaw_rad,
                         double *pitch_rad);

class WorldTargetFilter {
public:
    explicit WorldTargetFilter(ControlConfig config = {});
    void reset();
    bool update(double timestamp_s,
                double yaw_rad,
                double pitch_rad,
                double measurement_std_rad);
    WorldAimResult predict(double timestamp_s) const;

private:
    struct Axis {
        bool initialized = false;
        bool wrap = false;
        double angle = 0.0;
        double rate = 0.0;
        double p00 = 0.0;
        double p01 = 0.0;
        double p10 = 0.0;
        double p11 = 0.0;
        double timestamp_s = 0.0;
    };

    static void predict_axis(Axis *axis, double timestamp_s, double acceleration_variance);
    static bool update_axis(Axis *axis,
                            double timestamp_s,
                            double measurement,
                            double variance,
                            double acceleration_variance);

    ControlConfig config_;
    Axis yaw_;
    Axis pitch_;
    double last_measurement_s_ = -1.0;
    int reacquire_count_ = 0;
    double reacquire_yaw_ = 0.0;
    double reacquire_pitch_ = 0.0;
};

struct MpcReference {
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double acceleration_rad_s2 = 0.0;
};

class ReferenceMpcAxis {
public:
    ReferenceMpcAxis(double max_rate_rad_s,
                     double max_accel_rad_s2,
                     double max_jerk_rad_s3,
                     bool wrap_angle,
                     double position_weight = 1000.0,
                     double velocity_weight = 20.0,
                     double acceleration_weight = 1.0,
                     double jerk_weight = 0.001,
                     double target_rate_limit_rad_s = 1.0,
                     double target_rate_filter_tau_s = 0.04);
    void reset(double position_rad);
    MpcReference step(double target_position_rad, double target_velocity_rad_s, double dt_s = 0.001);
    bool initialized() const { return initialized_; }

private:
    static double wrap_pi(double value);
    void calculate_gain(double dt_s);

    double max_rate_;
    double max_accel_;
    double max_jerk_;
    bool wrap_;
    double position_weight_;
    double velocity_weight_;
    double acceleration_weight_;
    double jerk_weight_;
    double target_rate_limit_;
    double target_rate_filter_tau_;
    bool initialized_ = false;
    double position_ = 0.0;
    double velocity_ = 0.0;
    double acceleration_ = 0.0;
    double filtered_target_velocity_ = 0.0;
    double gain_position_ = 0.0;
    double gain_velocity_ = 0.0;
    double gain_acceleration_ = 0.0;
};

class WorldController {
public:
    WorldController(RuntimeConfig config, GimbalLink *link);
    ~WorldController();
    void start();
    void stop();
    void submit_vision(const FrameBundle &frame, const PipelineResult &result);
    WorldAimResult latest_world_target() const;
    std::string status_text() const;

private:
    struct VisionDiagnostics {
        uint64_t accepted_count = 0;
        uint64_t rejected_count = 0;
        std::string last_status = "none";
        int timestamp_quality = 0;
        bool aim_valid = false;
        bool measurement_valid = false;
        double camera_mapping_error_s = 0.0;
        double exposure_age_s = 0.0;
        double attitude_error_s = 0.0;
        double camera_yaw_deg = 0.0;
        double camera_pitch_deg = 0.0;
        double telemetry_yaw_deg = 0.0;
        double quaternion_forward_yaw_deg = 0.0;
        double measured_world_yaw_deg = 0.0;
        double measured_world_pitch_deg = 0.0;
        double continuous_world_yaw_deg = 0.0;
    };

    static double steady_now_s();
    void control_loop();
    double feedforward_torque(bool pitch, const MpcReference &reference) const;

    RuntimeConfig config_;
    GimbalLink *link_ = nullptr;
    mutable std::mutex filter_mutex_;
    WorldTargetFilter filter_;
    WorldAimResult last_world_measurement_;
    ReferenceMpcAxis yaw_mpc_;
    ReferenceMpcAxis pitch_mpc_;
    std::atomic<bool> running_ {false};
    std::thread thread_;
    mutable std::mutex status_mutex_;
    WorldAimResult latest_target_;
    mutable std::mutex vision_diag_mutex_;
    VisionDiagnostics vision_diag_;
    uint64_t sent_count_ = 0;
    uint64_t invalid_count_ = 0;
};

}  // namespace nxv
