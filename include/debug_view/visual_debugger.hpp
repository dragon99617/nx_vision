#pragma once

#include "common/config.hpp"
#include "common/types.hpp"
#include "debug_view/overlay_drawer.hpp"
#include "debug_view/panel_layout.hpp"

namespace nxv {

class VisualDebugger {
public:
    explicit VisualDebugger(DebugViewConfig config);
    int show(const PipelineResult &result, const CameraIntrinsics &intrinsics);
    bool save_snapshot(const PipelineResult &result, const CameraIntrinsics &intrinsics) const;

private:
    DebugViewConfig config_;
    OverlayDrawer overlay_;
    PanelLayout layout_;
    bool window_error_reported_ = false;
};

}  // namespace nxv
