#include "common/config.hpp"

#include <opencv2/core.hpp>

#include <filesystem>
#include <iostream>

namespace nxv {
namespace {

std::string path_join(const std::string &dir, const std::string &file)
{
    return (std::filesystem::path(dir) / file).string();
}

template <typename T>
void read_value(const cv::FileStorage &fs, const std::string &key, T *value)
{
    if (!fs.isOpened() || fs[key].empty() || value == nullptr) {
        return;
    }
    fs[key] >> *value;
}

void load_app(const std::string &dir, AppConfig *config)
{
    cv::FileStorage fs(path_join(dir, "app.yaml"), cv::FileStorage::READ);
    read_value(fs, "camera_backend", &config->camera_backend);
    read_value(fs, "camera_index", &config->camera_index);
    read_value(fs, "capture_width", &config->capture_width);
    read_value(fs, "capture_height", &config->capture_height);
    read_value(fs, "capture_fps", &config->capture_fps);
    read_value(fs, "depth_fps", &config->depth_fps);
    read_value(fs, "allow_mixed_rgbd_fps", &config->allow_mixed_rgbd_fps);
    read_value(fs, "input_image", &config->input_image);
    read_value(fs, "task", &config->task);
    read_value(fs, "color_exposure_fallback_us", &config->color_exposure_fallback_us);
    read_value(fs, "color_exposure_metadata_scale_us", &config->color_exposure_metadata_scale_us);
    read_value(fs, "sensor_timestamp_scale_us", &config->sensor_timestamp_scale_us);
    read_value(fs, "camera_timestamp_phase_offset_us", &config->camera_timestamp_phase_offset_us);
    read_value(fs, "camera_mapping_max_residual_us", &config->camera_mapping_max_residual_us);
    read_value(fs, "require_precise_timestamps", &config->require_precise_timestamps);
}

void load_vision(const std::string &dir, VisionParams *config)
{
    cv::FileStorage fs(path_join(dir, "vision_params.yaml"), cv::FileStorage::READ);
    read_value(fs, "processing_width", &config->processing_width);
    read_value(fs, "processing_height", &config->processing_height);
    read_value(fs, "threshold", &config->threshold);
    read_value(fs, "blur_kernel", &config->blur_kernel);
    read_value(fs, "morph_kernel", &config->morph_kernel);
    read_value(fs, "canny_low", &config->canny_low);
    read_value(fs, "canny_high", &config->canny_high);
    read_value(fs, "min_area_px", &config->min_area_px);
    read_value(fs, "approx_epsilon_ratio", &config->approx_epsilon_ratio);
    read_value(fs, "max_side_ratio", &config->max_side_ratio);
    read_value(fs, "min_angle_deg", &config->min_angle_deg);
    read_value(fs, "max_angle_deg", &config->max_angle_deg);
    read_value(fs, "use_threshold_inverse", &config->use_threshold_inverse);
}

void load_intrinsics(const std::string &dir, CameraIntrinsics *config)
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;
    int width = 0;
    int height = 0;

    cv::FileStorage fs(path_join(dir, "camera_intrinsics.yaml"), cv::FileStorage::READ);
    read_value(fs, "width", &width);
    read_value(fs, "height", &height);
    read_value(fs, "fx", &fx);
    read_value(fs, "fy", &fy);
    read_value(fs, "cx", &cx);
    read_value(fs, "cy", &cy);
    read_value(fs, "k1", &k1);
    read_value(fs, "k2", &k2);
    read_value(fs, "p1", &p1);
    read_value(fs, "p2", &p2);
    read_value(fs, "k3", &k3);

    if (fx > 0.0 && fy > 0.0) {
        config->camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        config->dist_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
        config->image_size = cv::Size(width, height);
        config->valid = true;
    }
}

void load_target(const std::string &dir, TargetGeometry *config)
{
    cv::FileStorage fs(path_join(dir, "target_geometry.yaml"), cv::FileStorage::READ);
    read_value(fs, "width_mm", &config->width_mm);
    read_value(fs, "height_mm", &config->height_mm);
    read_value(fs, "circle_radius_mm", &config->circle_radius_mm);
}

void load_laser(const std::string &dir, LaserExtrinsic *config)
{
    cv::FileStorage fs(path_join(dir, "laser_extrinsic.yaml"), cv::FileStorage::READ);
    read_value(fs, "origin_x_mm", &config->origin_cam_mm.x);
    read_value(fs, "origin_y_mm", &config->origin_cam_mm.y);
    read_value(fs, "origin_z_mm", &config->origin_cam_mm.z);
}

void load_serial(const std::string &dir, SerialConfig *config)
{
    cv::FileStorage fs(path_join(dir, "serial.yaml"), cv::FileStorage::READ);
    read_value(fs, "device", &config->device);
    read_value(fs, "baudrate", &config->baudrate);
    read_value(fs, "enabled", &config->enabled);
    read_value(fs, "dry_run", &config->dry_run);
    read_value(fs, "angle_scale_cdeg", &config->angle_scale_cdeg);
    read_value(fs, "protocol", &config->protocol);
    read_value(fs, "v4_sync_hz", &config->v4_sync_hz);
    read_value(fs, "v4_min_lead_ms", &config->v4_min_lead_ms);
    read_value(fs, "v4_max_lead_ms", &config->v4_max_lead_ms);
    read_value(fs, "v4_precise_uncertainty_ms", &config->v4_precise_uncertainty_ms);
}

void load_control(const std::string &dir, ControlConfig *config)
{
    cv::FileStorage fs(path_join(dir, "control.yaml"), cv::FileStorage::READ);
    read_value(fs, "enabled", &config->enabled);
    read_value(fs, "velocity_feedforward_enabled", &config->velocity_feedforward_enabled);
    read_value(fs, "torque_feedforward_enabled", &config->torque_feedforward_enabled);
    read_value(fs, "yaw_max_rate_rad_s", &config->yaw_max_rate_rad_s);
    read_value(fs, "pitch_max_rate_rad_s", &config->pitch_max_rate_rad_s);
    read_value(fs, "max_accel_rad_s2", &config->max_accel_rad_s2);
    read_value(fs, "max_jerk_rad_s3", &config->max_jerk_rad_s3);
    read_value(fs, "mpc_position_weight", &config->mpc_position_weight);
    read_value(fs, "mpc_velocity_weight", &config->mpc_velocity_weight);
    read_value(fs, "mpc_acceleration_weight", &config->mpc_acceleration_weight);
    read_value(fs, "target_predict_full_s", &config->target_predict_full_s);
    read_value(fs, "target_velocity_decay_end_s", &config->target_velocity_decay_end_s);
    read_value(fs, "target_hold_end_s", &config->target_hold_end_s);
    read_value(fs, "yaw_inertia", &config->yaw_inertia);
    read_value(fs, "yaw_viscous", &config->yaw_viscous);
    read_value(fs, "yaw_coulomb", &config->yaw_coulomb);
    read_value(fs, "pitch_inertia", &config->pitch_inertia);
    read_value(fs, "pitch_viscous", &config->pitch_viscous);
    read_value(fs, "pitch_coulomb", &config->pitch_coulomb);
    read_value(fs, "pitch_gravity", &config->pitch_gravity);
    read_value(fs, "pitch_gravity_zero_rad", &config->pitch_gravity_zero_rad);
    read_value(fs, "realtime_priority", &config->realtime_priority);
    read_value(fs, "cpu_affinity", &config->cpu_affinity);

    cv::Mat rotation;
    if (fs.isOpened() && !fs["body_from_camera"].empty()) {
        fs["body_from_camera"] >> rotation;
        if (rotation.rows == 3 && rotation.cols == 3) {
            rotation.convertTo(rotation, CV_64F);
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    config->body_from_camera(row, col) = rotation.at<double>(row, col);
                }
            }
        }
    }
}

void load_debug(const std::string &dir, DebugViewConfig *config)
{
    cv::FileStorage fs(path_join(dir, "debug_view.yaml"), cv::FileStorage::READ);
    read_value(fs, "show_windows", &config->show_windows);
    read_value(fs, "panel_width", &config->panel_width);
    read_value(fs, "panel_height", &config->panel_height);
    read_value(fs, "debug_fps", &config->debug_fps);
    read_value(fs, "snapshot_dir", &config->snapshot_dir);
}

void load_depth(const std::string &dir, DepthConfig *config)
{
    cv::FileStorage fs(path_join(dir, "depth.yaml"), cv::FileStorage::READ);
    read_value(fs, "enabled", &config->enabled);
    read_value(fs, "min_depth_mm", &config->min_depth_mm);
    read_value(fs, "max_depth_mm", &config->max_depth_mm);
    read_value(fs, "roi_shrink_ratio", &config->roi_shrink_ratio);
    read_value(fs, "min_valid_samples", &config->min_valid_samples);
    read_value(fs, "fallback_to_pnp", &config->fallback_to_pnp);
    read_value(fs, "reuse_last_depth", &config->reuse_last_depth);
    read_value(fs, "max_reused_depth_age_s", &config->max_reused_depth_age_s);
}

}  // namespace

std::vector<cv::Point3f> TargetGeometry::outer_object_points() const
{
    const float half_w = static_cast<float>(width_mm * 0.5);
    const float half_h = static_cast<float>(height_mm * 0.5);
    return {
        {-half_w, -half_h, 0.0f},
        { half_w, -half_h, 0.0f},
        { half_w,  half_h, 0.0f},
        {-half_w,  half_h, 0.0f},
    };
}

cv::Point3d TargetGeometry::center_mm() const
{
    return cv::Point3d(0.0, 0.0, 0.0);
}

RuntimeConfig load_config(const std::string &config_dir)
{
    RuntimeConfig config;
    load_app(config_dir, &config.app);
    load_vision(config_dir, &config.vision);
    load_intrinsics(config_dir, &config.intrinsics);
    load_target(config_dir, &config.target);
    load_laser(config_dir, &config.laser);
    load_serial(config_dir, &config.serial);
    load_debug(config_dir, &config.debug);
    load_depth(config_dir, &config.depth);
    load_control(config_dir, &config.control);
    return config;
}

}  // namespace nxv
