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
};

RuntimeConfig load_config(const std::string &config_dir);

}  // namespace nxv
