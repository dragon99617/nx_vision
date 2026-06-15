#include "camera/orbbec_camera.hpp"
#include "common/app_context.hpp"

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

    for (int attempt = 0; attempt < 120; ++attempt) {
        nxv::FrameBundle frame;
        if (camera.grab(&frame) && !frame.color_bgr.empty()) {
            std::cout << "camera ok: " << frame.color_bgr.cols << "x" << frame.color_bgr.rows
                      << " timestamp_s=" << frame.timestamp_s << "\n";
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "Camera opened, but no color frame arrived\n";
    return 2;
}
