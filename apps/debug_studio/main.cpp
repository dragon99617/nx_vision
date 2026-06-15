#include "camera/orbbec_camera.hpp"
#include "comm/serial_port.hpp"
#include "common/app_context.hpp"
#include "debug_view/visual_debugger.hpp"
#include "tasks/task_base.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    const std::string config_dir = argc > 1 ? argv[1] : "config";
    nxv::AppContext context = nxv::make_app_context(config_dir);

    nxv::OrbbecCamera camera;
    if (!camera.open(context.config.app)) {
        std::cerr << "Failed to open camera/input\n";
        return 1;
    }

    nxv::SerialPort serial;
    serial.open(context.config.serial);

    std::unique_ptr<nxv::TaskBase> task = nxv::make_task(context.config.app.task, context.config);
    nxv::VisualDebugger debugger(context.config.debug);

    std::cout << "Debug studio running task: " << task->name() << "\n";
    std::cout << "Keys: q=quit, s=snapshot\n";

    for (;;) {
        nxv::FrameBundle frame;
        if (!camera.grab(&frame)) {
            continue;
        }

        nxv::PipelineResult result = task->update(frame);
        serial.write_line(result.serial_packet);

        const int key = debugger.show(result, context.config.intrinsics);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == 's') {
            debugger.save_snapshot(result, context.config.intrinsics);
        }
    }
}

