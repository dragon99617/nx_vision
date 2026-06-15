#include "debug_view/visual_debugger.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iostream>
#include <map>
#include <utility>

namespace nxv {

VisualDebugger::VisualDebugger(DebugViewConfig config)
    : config_(std::move(config))
{
}

int VisualDebugger::show(const PipelineResult &result, const CameraIntrinsics &intrinsics)
{
    if (!config_.show_windows) {
        return -1;
    }

    std::map<std::string, cv::Mat> panels = result.panels;
    panels["main"] = overlay_.draw_main_overlay(result.panels.count("raw") ? result.panels.at("raw") : cv::Mat(), result);
    panels["pose"] = overlay_.draw_pose_overlay(result.panels.count("raw") ? result.panels.at("raw") : cv::Mat(), result, intrinsics);

    cv::Mat serial_panel(config_.panel_height, config_.panel_width, CV_8UC3, cv::Scalar(15, 15, 15));
    cv::putText(serial_panel, result.serial_packet, cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    panels["serial"] = serial_panel;

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

    cv::Mat serial_panel(config_.panel_height, config_.panel_width, CV_8UC3, cv::Scalar(15, 15, 15));
    cv::putText(serial_panel, result.serial_packet, cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    panels["serial"] = serial_panel;

    const cv::Mat overview = layout.make_overview(panels, config_.panel_width, config_.panel_height);
    return cv::imwrite((std::filesystem::path(config_.snapshot_dir) / "snapshot.png").string(), overview);
}

}  // namespace nxv
