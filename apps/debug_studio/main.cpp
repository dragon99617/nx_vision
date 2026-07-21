#include "camera/orbbec_camera.hpp"
#include "comm/gimbal_link.hpp"
#include "common/app_context.hpp"
#include "control/world_controller.hpp"
#include "debug_view/visual_debugger.hpp"
#include "tasks/task_base.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(const Clock::time_point &start, const Clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

struct PerfStats {
    double grab_ms = 0.0;
    double pipeline_ms = 0.0;
    double serial_ms = 0.0;
    double debug_ms = 0.0;
    double total_ms = 0.0;
    int frames = 0;
    int debug_frames = 0;
    Clock::time_point last_report = Clock::now();

    void add(double grab, double pipeline, double serial, double debug, double total, bool rendered)
    {
        grab_ms += grab;
        pipeline_ms += pipeline;
        serial_ms += serial;
        debug_ms += debug;
        total_ms += total;
        ++frames;
        if (rendered) {
            ++debug_frames;
        }
    }

    void maybe_report()
    {
        const auto now = Clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - last_report).count();
        if (elapsed_s < 1.0 || frames <= 0) {
            return;
        }

        std::cout << std::fixed << std::setprecision(2)
                  << "[perf] update_hz=" << static_cast<double>(frames) / elapsed_s
                  << " debug_hz=" << static_cast<double>(debug_frames) / elapsed_s
                  << " grab_ms=" << grab_ms / frames
                  << " pipeline_ms=" << pipeline_ms / frames
                  << " serial_ms=" << serial_ms / frames
                  << " debug_ms=" << debug_ms / frames
                  << " total_ms=" << total_ms / frames << "\n";

        grab_ms = 0.0;
        pipeline_ms = 0.0;
        serial_ms = 0.0;
        debug_ms = 0.0;
        total_ms = 0.0;
        frames = 0;
        debug_frames = 0;
        last_report = now;
    }
};

}  // namespace

int main(int argc, char **argv)
{
    const std::string config_dir = argc > 1 ? argv[1] : "config";
    nxv::AppContext context = nxv::make_app_context(config_dir);

    nxv::OrbbecCamera camera;
    if (!camera.open(context.config.app)) {
        std::cerr << "Failed to open camera/input\n";
        return 1;
    }

    nxv::GimbalLink link;
    if (!link.open(context.config.serial)) {
        std::cerr << "Failed to exclusively open gimbal serial device. Stop other nx_runtime/nx_debug_studio processes first.\n";
        return 1;
    }

    std::unique_ptr<nxv::TaskBase> task = nxv::make_task(context.config.app.task, context.config);
    nxv::WorldController world(context.config, &link);
    world.start();
    nxv::VisualDebugger debugger(context.config.debug);

    std::cout << "Debug studio running task: " << task->name() << "\n";
    std::cout << "Keys: q=quit, s=snapshot\n";

    const double debug_period_s = context.config.debug.debug_fps > 0.0 ? 1.0 / context.config.debug.debug_fps : 0.0;
    Clock::time_point last_debug_frame = Clock::now() - std::chrono::seconds(1);
    PerfStats perf;

    for (;;) {
        const auto frame_start = Clock::now();
        nxv::FrameBundle frame;
        if (!camera.grab(&frame)) {
            continue;
        }
        const auto after_grab = Clock::now();

        const auto now = Clock::now();
        const bool render_debug = context.config.debug.show_windows &&
                                  (debug_period_s <= 0.0 ||
                                   std::chrono::duration<double>(now - last_debug_frame).count() >= debug_period_s);

        nxv::PipelineResult result = task->update(frame, render_debug);
        const auto after_pipeline = Clock::now();
        bool sent = false;
        if (link.is_v4()) {
            world.submit_vision(frame, result);
            result.world_aim = world.latest_world_target();
            result.serial_packet = world.status_text();
            sent = true;
        } else {
            sent = link.write_legacy(result.serial_packet);
        }
        if (sent) {
            debugger.record_serial_update();
        }
        const auto after_serial = Clock::now();

        double debug_ms = 0.0;
        bool rendered = false;
        if (render_debug) {
            const auto debug_start = Clock::now();
            const int key = debugger.show(result, context.config.intrinsics);
            const auto debug_end = Clock::now();
            debug_ms = elapsed_ms(debug_start, debug_end);
            rendered = true;
            last_debug_frame = debug_end;
            if (key == 'q' || key == 27) {
                break;
            }
            if (key == 's') {
                debugger.save_snapshot(result, context.config.intrinsics);
            }
        }
        const auto frame_end = Clock::now();
        perf.add(elapsed_ms(frame_start, after_grab),
                 elapsed_ms(after_grab, after_pipeline),
                 elapsed_ms(after_pipeline, after_serial),
                 debug_ms,
                 elapsed_ms(frame_start, frame_end),
                 rendered);
        perf.maybe_report();
    }
}
