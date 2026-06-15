#include "common/app_context.hpp"

namespace nxv {

AppContext make_app_context(const std::string &config_dir)
{
    AppContext context;
    context.config = load_config(config_dir);
    return context;
}

}  // namespace nxv

