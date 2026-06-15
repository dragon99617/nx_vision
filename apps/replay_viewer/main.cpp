#include "camera/orbbec_camera.hpp"
#include "common/app_context.hpp"
#include "debug_view/visual_debugger.hpp"
#include "tasks/task_base.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    const std::string config_dir = argc > 1 ? argv[1] : "config";
    nxv::AppContext context = nxv::make_app_context(config_dir);
    if (argc > 2) {
        context.config.app.input_image = argv[2];
    }

    if (context.config.app.input_image.empty()) {
        std::cerr << "Set input_image in config/app.yaml or pass a replay config directory.\n";
        return 1;
    }

    nxv::OrbbecCamera camera;
    if (!camera.open(context.config.app)) {
        std::cerr << "Failed to open replay image\n";
        return 1;
    }

    std::unique_ptr<nxv::TaskBase> task = nxv::make_task(context.config.app.task, context.config);
    nxv::VisualDebugger debugger(context.config.debug);

    for (;;) {
        nxv::FrameBundle frame;
        if (!camera.grab(&frame)) {
            return 1;
        }
        nxv::PipelineResult result = task->update(frame);
        const int key = debugger.show(result, context.config.intrinsics);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == 's') {
            debugger.save_snapshot(result, context.config.intrinsics);
        }
    }

    return 0;
}
