#pragma once

#include "common/config.hpp"

namespace nxv {

struct AppContext {
    RuntimeConfig config;
};

AppContext make_app_context(const std::string &config_dir);

}  // namespace nxv

