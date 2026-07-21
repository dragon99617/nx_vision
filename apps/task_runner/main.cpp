#include "camera/orbbec_camera.hpp"
#include "comm/gimbal_link.hpp"
#include "common/app_context.hpp"
#include "control/world_controller.hpp"
#include "debug_view/visual_debugger.hpp"
#include "tasks/task_base.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    const std::string config_dir = argc > 1 ? argv[1] : "config";
    const std::string task_name = argc > 2 ? argv[2] : "";

    nxv::AppContext context = nxv::make_app_context(config_dir);
    if (!task_name.empty()) {
        context.config.app.task = task_name;
    }

    nxv::OrbbecCamera camera;
    if (!camera.open(context.config.app)) {
        std::cerr << "Failed to open camera/input\n";
        return 1;
    }

    nxv::GimbalLink link;
    if (!link.open(context.config.serial)) {
        std::cerr << "Failed to exclusively open gimbal serial device. Stop other nx_vision processes first.\n";
        return 1;
    }

    std::unique_ptr<nxv::TaskBase> task = nxv::make_task(context.config.app.task, context.config);
    nxv::WorldController world(context.config, &link);
    world.start();
    nxv::VisualDebugger debugger(context.config.debug);
    std::cout << "Task runner: " << task->name() << "\n";

    for (;;) {
        nxv::FrameBundle frame;
        if (!camera.grab(&frame)) {
            continue;
        }
        nxv::PipelineResult result = task->update(frame);
        if (link.is_v4()) {
            world.submit_vision(frame, result);
            result.world_aim = world.latest_world_target();
            result.serial_packet = world.status_text();
        } else {
            link.write_legacy(result.serial_packet);
        }
        const int key = debugger.show(result, context.config.intrinsics);
        if (key == 'q' || key == 27) {
            break;
        }
    }
}

