//图像预处理实现
#include "vision/preprocess.hpp"

#include <opencv2/imgproc.hpp>

namespace nxv {

namespace {

int make_odd_kernel(int value)
{
    if (value < 1) {
        value = 1;
    }
    if ((value % 2) == 0) {
        value += 1;
    }
    return value;
}

}  // namespace

PreprocessOutput run_preprocess(const cv::Mat &color_bgr, const VisionParams &params)
{
    PreprocessOutput output;
    if (color_bgr.empty()) {
        return output;
    }

    //resize
    if (params.processing_width > 0 && params.processing_height > 0 &&
        (color_bgr.cols != params.processing_width || color_bgr.rows != params.processing_height)) {
        cv::resize(color_bgr, output.resized_bgr, cv::Size(params.processing_width, params.processing_height));
        output.scale_x_to_src = static_cast<double>(color_bgr.cols) / static_cast<double>(params.processing_width);
        output.scale_y_to_src = static_cast<double>(color_bgr.rows) / static_cast<double>(params.processing_height);
    } else {
        output.resized_bgr = color_bgr;
    }

    //灰度，模糊，二值化，形态学闭运算，边缘检测，合成
    cv::cvtColor(output.resized_bgr, output.gray, cv::COLOR_BGR2GRAY);
    const int blur_kernel = make_odd_kernel(params.blur_kernel);
    cv::GaussianBlur(output.gray, output.blurred, cv::Size(blur_kernel, blur_kernel), 0.0);

    const int thresh_type = params.use_threshold_inverse ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    cv::threshold(output.blurred, output.binary, params.threshold, 255, thresh_type);

    const int morph_kernel = make_odd_kernel(params.morph_kernel);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel, morph_kernel));
    cv::morphologyEx(output.binary, output.closed, cv::MORPH_CLOSE, kernel);

    cv::Canny(output.closed, output.edges, params.canny_low, params.canny_high);
    cv::bitwise_or(output.closed, output.edges, output.combined);
    return output;
}

}  // namespace nxv

