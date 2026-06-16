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

    nxv::FrameBundle last_color_frame;
    int color_frames = 0;
    int new_depth_frames = 0;
    int reused_depth_frames = 0;
    int missing_depth_frames = 0;
    const auto started = std::chrono::steady_clock::now();

    for (int attempt = 0; attempt < 180; ++attempt) {
        nxv::FrameBundle frame;
        if (camera.grab(&frame) && !frame.color_bgr.empty()) {
            last_color_frame = frame;
            ++color_frames;
            if (frame.depth_mm.empty()) {
                ++missing_depth_frames;
            } else if (frame.depth_reused) {
                ++reused_depth_frames;
            } else {
                ++new_depth_frames;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (std::chrono::steady_clock::now() - started > std::chrono::seconds(2)) {
            break;
        }
    }

    if (!last_color_frame.color_bgr.empty()) {
        const double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::cout << "camera ok: " << last_color_frame.color_bgr.cols << "x" << last_color_frame.color_bgr.rows
                  << " frames=" << color_frames
                  << " fps=" << (elapsed_s > 0.0 ? static_cast<double>(color_frames) / elapsed_s : 0.0)
                  << " new_depth=" << new_depth_frames
                  << " reused_depth=" << reused_depth_frames
                  << " missing_depth=" << missing_depth_frames;
        if (!last_color_frame.depth_mm.empty()) {
            std::cout << " depth=" << last_color_frame.depth_mm.cols << "x" << last_color_frame.depth_mm.rows
                      << (last_color_frame.depth_reused ? " reused" : " new")
                      << " depth_age_s=" << last_color_frame.depth_age_s;
        } else {
            std::cout << " depth=unavailable";
        }
        std::cout << "\n";
        return 0;
    }

    std::cerr << "Camera opened, but no color frame arrived\n";
    return 2;
}
