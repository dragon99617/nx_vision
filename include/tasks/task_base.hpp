#pragma once

#include "common/config.hpp"
#include "pipeline/aim_pipeline.hpp"

#include <memory>
#include <string>

namespace nxv {

class TaskBase {
public:
    explicit TaskBase(RuntimeConfig config);
    virtual ~TaskBase() = default;

    virtual std::string name() const = 0;
    virtual PipelineResult update(const FrameBundle &frame, bool make_debug_panels = true) = 0;

protected:
    RuntimeConfig config_;
    AimPipeline pipeline_;
};

std::unique_ptr<TaskBase> make_task(const std::string &task_name, const RuntimeConfig &config);

}  // namespace nxv
