#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

#include <opencv2/videoio.hpp>

#include <memory>

#ifdef NXVISION_WITH_ORBBEC
namespace ob {
class Pipeline;
}
#endif

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
    bool open_orbbec_camera(const AppConfig &config);
    bool grab_orbbec(FrameBundle *frame);

    cv::VideoCapture capture_;
    cv::Mat static_image_;
#ifdef NXVISION_WITH_ORBBEC
    std::shared_ptr<ob::Pipeline> orbbec_pipeline_;
#endif
    bool static_mode_ = false;
    bool orbbec_mode_ = false;
    bool opened_ = false;
};

}  // namespace nxv
