#include "tasks/task_base.hpp"

#include <cmath>
#include <utility>

namespace nxv {
namespace {

constexpr double kPi = 3.14159265358979323846;

class BasicTraceOnlyTask final : public TaskBase {
public:
    using TaskBase::TaskBase;
    std::string name() const override { return "basic_trace_only"; }
    PipelineResult update(const FrameBundle &frame, bool make_debug_panels) override
    {
        (void)frame;
        (void)make_debug_panels;
        PipelineResult result;
        result.serial_packet = "A,0,0,0,0\n";
        return result;
    }
};

class CenterAimTask final : public TaskBase {
public:
    CenterAimTask(RuntimeConfig config, std::string task_name)
        : TaskBase(std::move(config)), task_name_(std::move(task_name))
    {
    }

    std::string name() const override { return task_name_; }
    PipelineResult update(const FrameBundle &frame, bool make_debug_panels) override
    {
        return pipeline_.process(frame, make_debug_panels);
    }

private:
    std::string task_name_;
};

class CircleSyncTask final : public TaskBase {
public:
    using TaskBase::TaskBase;
    std::string name() const override { return "adv_circle_sync"; }
    PipelineResult update(const FrameBundle &frame, bool make_debug_panels) override
    {
        const double now = frame.timestamp_s;
        const double period_s = 20.0;
        const double phase = std::fmod(now, period_s) / period_s;
        const double angle = 2.0 * kPi * phase;
        const double r = config_.target.circle_radius_mm;
        const cv::Point3d board_point(r * std::cos(angle), r * std::sin(angle), 0.0);
        return pipeline_.process_board_point(frame, board_point, make_debug_panels);
    }
};

}  // namespace

TaskBase::TaskBase(RuntimeConfig config)
    : config_(std::move(config)), pipeline_(config_)
{
}

std::unique_ptr<TaskBase> make_task(const std::string &task_name, const RuntimeConfig &config)
{
    if (task_name == "basic_trace_only") {
        return std::make_unique<BasicTraceOnlyTask>(config);
    }
    if (task_name == "adv_circle_sync") {
        return std::make_unique<CircleSyncTask>(config);
    }
    if (task_name == "basic_static_aim" ||
        task_name == "basic_track_aim" ||
        task_name == "adv_one_lap" ||
        task_name == "adv_two_laps") {
        return std::make_unique<CenterAimTask>(config, task_name);
    }
    return std::make_unique<CenterAimTask>(config, "basic_static_aim");
}

}  // namespace nxv
