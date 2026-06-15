#include "camera/orbbec_camera.hpp"
#include "comm/serial_port.hpp"
#include "common/app_context.hpp"
#include "tasks/task_base.hpp"

#include <chrono>
#include <iostream>
#include <thread>

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
    if (!serial.open(context.config.serial)) {
        std::cerr << "Failed to open serial\n";
        return 1;
    }

    std::unique_ptr<nxv::TaskBase> task = nxv::make_task(context.config.app.task, context.config);
    std::cout << "Running task: " << task->name() << "\n";

    for (;;) {
        nxv::FrameBundle frame;
        if (!camera.grab(&frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        nxv::PipelineResult result = task->update(frame);
        serial.write_line(result.serial_packet);
    }
}

