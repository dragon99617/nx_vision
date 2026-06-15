#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

#include <opencv2/videoio.hpp>

namespace nxv {

class OrbbecCamera {
public:
    bool open(const AppConfig &config);
    bool grab(FrameBundle *frame);
    void close();
    bool is_open() const;

private:
    bool load_static_image(const std::string &path);
    bool open_opencv_camera(const AppConfig &config);

    cv::VideoCapture capture_;
    cv::Mat static_image_;
    bool static_mode_ = false;
    bool opened_ = false;
};

}  // namespace nxv

