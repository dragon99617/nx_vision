#pragma once

#include <opencv2/core.hpp>

#include <map>
#include <string>

namespace nxv {

class PanelLayout {
public:
    cv::Mat make_overview(const std::map<std::string, cv::Mat> &panels,
                          int panel_width,
                          int panel_height) const;
};

}  // namespace nxv

