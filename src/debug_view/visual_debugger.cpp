#include "debug_view/visual_debugger.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>

namespace nxv {

VisualDebugger::VisualDebugger(DebugViewConfig config)
    : config_(std::move(config))
{
}

void VisualDebugger::update_serial_fps()
{
    const auto now = std::chrono::steady_clock::now();
    if (!has_serial_time_) {
        last_serial_time_ = now;
        has_serial_time_ = true;
        return;
    }

    const double dt_s = std::chrono::duration<double>(now - last_serial_time_).count();
    last_serial_time_ = now;
    if (dt_s <= 1e-6) {
        return;
    }

    const double instant_fps = 1.0 / dt_s;
    serial_fps_ = serial_fps_ <= 0.0 ? instant_fps : serial_fps_ * 0.85 + instant_fps * 0.15;
}

cv::Mat VisualDebugger::make_serial_panel(const std::string &serial_packet) const
{
    std::string packet = serial_packet;
    for (char &ch : packet) {
        if (ch == '\n' || ch == '\r') {
            ch = ' ';
        }
    }

    std::ostringstream fps_text;
    fps_text << "Serial FPS: " << std::fixed << std::setprecision(1) << serial_fps_ << " Hz";

    cv::Mat serial_panel(config_.panel_height, config_.panel_width, CV_8UC3, cv::Scalar(15, 15, 15));
    cv::putText(serial_panel, "Packet: " + packet, cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(serial_panel, fps_text.str(), cv::Point(12, 84), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(0, 220, 120), 2, cv::LINE_AA);
    return serial_panel;
}

int VisualDebugger::show(const PipelineResult &result, const CameraIntrinsics &intrinsics)
{
    if (!config_.show_windows) {
        return -1;
    }
    update_serial_fps();

    std::map<std::string, cv::Mat> panels = result.panels;
    panels["main"] = overlay_.draw_main_overlay(result.panels.count("raw") ? result.panels.at("raw") : cv::Mat(), result);
    panels["pose"] = overlay_.draw_pose_overlay(result.panels.count("raw") ? result.panels.at("raw") : cv::Mat(), result, intrinsics);
    panels["serial"] = make_serial_panel(result.serial_packet);

    const cv::Mat overview = layout_.make_overview(panels, config_.panel_width, config_.panel_height);
    try {
        cv::imshow("nx_debug_studio", overview);
        return cv::waitKey(1);
    } catch (const cv::Exception &error) {
        config_.show_windows = false;
        if (!window_error_reported_) {
            std::cerr << "[debug] OpenCV window backend unavailable; disabling debug windows: "
                      << error.what() << "\n";
            window_error_reported_ = true;
        }
        return -1;
    }
}

bool VisualDebugger::save_snapshot(const PipelineResult &result, const CameraIntrinsics &intrinsics) const
{
    std::filesystem::create_directories(config_.snapshot_dir);

    std::map<std::string, cv::Mat> panels = result.panels;
    OverlayDrawer overlay;
    PanelLayout layout;
    panels["main"] = overlay.draw_main_overlay(result.panels.count("raw") ? result.panels.at("raw") : cv::Mat(), result);
    panels["pose"] = overlay.draw_pose_overlay(result.panels.count("raw") ? result.panels.at("raw") : cv::Mat(), result, intrinsics);
    panels["serial"] = make_serial_panel(result.serial_packet);

    const cv::Mat overview = layout.make_overview(panels, config_.panel_width, config_.panel_height);
    return cv::imwrite((std::filesystem::path(config_.snapshot_dir) / "snapshot.png").string(), overview);
}

}  // namespace nxv
