#pragma once

#include "common/types.hpp"

#include <string>

namespace nxv {

struct AppConfig {
    std::string camera_backend = "opencv";
    int camera_index = 0;
    int capture_width = 1280;
    int capture_height = 800;
    int capture_fps = 60;
    int depth_fps = 30;
    bool allow_mixed_rgbd_fps = true;
    std::string input_image;
    std::string task = "basic_static_aim";
    double color_exposure_fallback_us = 5000.0;
    double color_exposure_metadata_scale_us = 100.0;
    double sensor_timestamp_scale_us = 1.0;
    double camera_timestamp_phase_offset_us = 0.0;
    double camera_mapping_max_residual_us = 500.0;
    bool require_precise_timestamps = true;
};

struct VisionParams {
    int processing_width = 640;
    int processing_height = 400;
    int threshold = 120;
    int blur_kernel = 5;
    int morph_kernel = 5;
    int canny_low = 50;
    int canny_high = 150;
    double min_area_px = 500.0;
    double approx_epsilon_ratio = 0.018;
    double max_side_ratio = 5.0;
    double min_angle_deg = 55.0;
    double max_angle_deg = 125.0;
    bool use_threshold_inverse = true;
};

struct SerialConfig {
    std::string device = "/dev/ttyTHS0";
    int baudrate = 115200;
    bool enabled = false;
    bool dry_run = true;
    int angle_scale_cdeg = 100;
    std::string protocol = "v2";
    int v4_sync_hz = 10;
    double v4_min_lead_ms = 4.0;
    double v4_max_lead_ms = 10.0;
    double v4_precise_uncertainty_ms = 0.5;
};

struct ControlConfig {
    bool enabled = true;
    bool velocity_feedforward_enabled = true;
    bool torque_feedforward_enabled = false;
    double yaw_max_rate_rad_s = 1.0471975511965976;
    double pitch_max_rate_rad_s = 0.6981317007977318;
    double yaw_max_accel_rad_s2 = 4.1887902047863905;
    double pitch_max_accel_rad_s2 = 3.141592653589793;
    double yaw_max_jerk_rad_s3 = 41.88790204786391;
    double pitch_max_jerk_rad_s3 = 31.41592653589793;
    double yaw_mpc_position_weight = 250.0;
    double pitch_mpc_position_weight = 250.0;
    double mpc_velocity_weight = 2.0;
    double mpc_acceleration_weight = 0.015;
    double yaw_target_rate_limit_rad_s = 0.5235987755982988;
    double pitch_target_rate_limit_rad_s = 0.3490658503988659;
    double mpc_target_rate_filter_tau_s = 0.08;
    double target_predict_full_s = 0.20;
    double target_velocity_decay_end_s = 0.50;
    double target_hold_end_s = 1.0;
    double yaw_inertia = 0.0;
    double yaw_viscous = 0.0;
    double yaw_coulomb = 0.0;
    double pitch_inertia = 0.0;
    double pitch_viscous = 0.0;
    double pitch_coulomb = 0.0;
    double pitch_gravity = 0.0;
    double pitch_gravity_zero_rad = 0.0;
    int realtime_priority = 30;
    int cpu_affinity = -1;
    cv::Matx33d body_from_camera = cv::Matx33d::eye();
};

struct DebugViewConfig {
    bool show_windows = true;
    int panel_width = 480;
    int panel_height = 300;
    double debug_fps = 15.0;
    std::string snapshot_dir = "debug_view/snapshots";
};

struct DepthConfig {
    bool enabled = true;
    double min_depth_mm = 150.0;
    double max_depth_mm = 8000.0;
    double roi_shrink_ratio = 0.55;
    int min_valid_samples = 30;
    bool fallback_to_pnp = true;
    bool reuse_last_depth = true;
    double max_reused_depth_age_s = 0.10;
};

struct RuntimeConfig {
    AppConfig app;
    VisionParams vision;
    CameraIntrinsics intrinsics;
    TargetGeometry target;
    LaserExtrinsic laser;
    SerialConfig serial;
    DebugViewConfig debug;
    DepthConfig depth;
    ControlConfig control;
};

RuntimeConfig load_config(const std::string &config_dir);

}  // namespace nxv
