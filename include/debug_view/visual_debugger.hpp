#pragma once

#include "common/config.hpp"
#include "common/types.hpp"
#include "debug_view/overlay_drawer.hpp"
#include "debug_view/panel_layout.hpp"

#include <chrono>

namespace nxv {

class VisualDebugger {
public:
    explicit VisualDebugger(DebugViewConfig config);
    void record_serial_update();
    int show(const PipelineResult &result, const CameraIntrinsics &intrinsics);
    bool save_snapshot(const PipelineResult &result, const CameraIntrinsics &intrinsics) const;

private:
    cv::Mat make_serial_panel(const std::string &serial_packet) const;

    DebugViewConfig config_;
    OverlayDrawer overlay_;
    PanelLayout layout_;
    std::chrono::steady_clock::time_point last_serial_time_;
    double serial_fps_ = 0.0;
    bool has_serial_time_ = false;
    bool window_error_reported_ = false;
};

}  // namespace nxv
