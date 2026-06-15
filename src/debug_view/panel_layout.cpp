#include "debug_view/panel_layout.hpp"

#include <opencv2/imgproc.hpp>

#include <vector>

namespace nxv {
namespace {

cv::Mat normalize_panel(const cv::Mat &input, int width, int height, const std::string &label)
{
    cv::Mat bgr;
    if (input.empty()) {
        bgr = cv::Mat(height, width, CV_8UC3, cv::Scalar(15, 15, 15));
    } else if (input.channels() == 1) {
        cv::cvtColor(input, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = input.clone();
    }

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(width, height));
    cv::rectangle(resized, cv::Rect(0, 0, width, 28), cv::Scalar(0, 0, 0), -1);
    cv::putText(resized, label, cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    return resized;
}

}  // namespace

cv::Mat PanelLayout::make_overview(const std::map<std::string, cv::Mat> &panels,
                                   int panel_width,
                                   int panel_height) const
{
    const std::vector<std::string> order = {
        "main", "binary", "edges",
        "combined", "contours", "perspective",
        "pose", "serial",
    };

    const int cols = 3;
    const int rows = 3;
    cv::Mat canvas(rows * panel_height, cols * panel_width, CV_8UC3, cv::Scalar(30, 30, 30));

    for (size_t i = 0; i < order.size(); ++i) {
        auto it = panels.find(order[i]);
        cv::Mat panel = normalize_panel(it == panels.end() ? cv::Mat() : it->second,
                                        panel_width,
                                        panel_height,
                                        order[i]);
        const int row = static_cast<int>(i) / cols;
        const int col = static_cast<int>(i) % cols;
        panel.copyTo(canvas(cv::Rect(col * panel_width, row * panel_height, panel_width, panel_height)));
    }
    return canvas;
}

}  // namespace nxv

