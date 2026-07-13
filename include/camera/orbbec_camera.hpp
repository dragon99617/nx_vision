#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

#include <opencv2/videoio.hpp>

#include <memory>

#ifdef NXVISION_WITH_ORBBEC
namespace ob {
class Align;
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
    double map_system_timestamp_us(uint64_t timestamp_us,
                                   double receive_time_s,
                                   bool *reliable);
    void assign_fallback_metadata(FrameBundle *frame);

    AppConfig app_config_;
    cv::VideoCapture capture_;
    cv::Mat static_image_;
    cv::Mat last_depth_mm_;
    double last_depth_timestamp_s_ = 0.0;
    double system_to_steady_offset_s_ = 0.0;
    bool has_system_to_steady_offset_ = false;
    uint32_t next_sequence_ = 0;
#ifdef NXVISION_WITH_ORBBEC
    std::shared_ptr<ob::Pipeline> orbbec_pipeline_;
    std::shared_ptr<ob::Align> depth_to_color_align_;
#endif
    bool static_mode_ = false;
    bool orbbec_mode_ = false;
    bool software_depth_align_ = false;
    bool opened_ = false;
};

}  // namespace nxv
