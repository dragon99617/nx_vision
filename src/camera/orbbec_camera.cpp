#include "camera/orbbec_camera.hpp"

#include <opencv2/imgcodecs.hpp>

#include <chrono>

namespace nxv {

bool OrbbecCamera::open(const AppConfig &config)
{
    if (!config.input_image.empty()) {
        return load_static_image(config.input_image);
    }
    return open_opencv_camera(config);
}

bool OrbbecCamera::load_static_image(const std::string &path)
{
    static_image_ = cv::imread(path, cv::IMREAD_COLOR);
    static_mode_ = !static_image_.empty();
    opened_ = static_mode_;
    return opened_;
}

bool OrbbecCamera::open_opencv_camera(const AppConfig &config)
{
    capture_.open(config.camera_index);
    if (!capture_.isOpened()) {
        opened_ = false;
        return false;
    }
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.capture_width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.capture_height);
    capture_.set(cv::CAP_PROP_FPS, config.capture_fps);
    opened_ = true;
    return true;
}

bool OrbbecCamera::grab(FrameBundle *frame)
{
    if (!opened_ || frame == nullptr) {
        return false;
    }

    if (static_mode_) {
        frame->color_bgr = static_image_.clone();
    } else if (!capture_.read(frame->color_bgr) || frame->color_bgr.empty()) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    frame->timestamp_s = std::chrono::duration<double>(now).count();
    return true;
}

void OrbbecCamera::close()
{
    if (capture_.isOpened()) {
        capture_.release();
    }
    opened_ = false;
}

bool OrbbecCamera::is_open() const
{
    return opened_;
}

}  // namespace nxv

