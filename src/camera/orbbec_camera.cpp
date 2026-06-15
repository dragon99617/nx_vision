#include "camera/orbbec_camera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef NXVISION_WITH_ORBBEC
#include <libobsensor/ObSensor.hpp>
#endif

#include <chrono>
#include <cctype>
#include <iostream>

namespace nxv {
namespace {

std::string to_lower(std::string value)
{
    for (char &ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

void set_steady_timestamp(FrameBundle *frame)
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    frame->timestamp_s = std::chrono::duration<double>(now).count();
}

#ifdef NXVISION_WITH_ORBBEC
void log_orbbec_error(const char *context, const ob::Error &error)
{
    std::cerr << context << ": " << error.what() << " [function=" << error.getFunction()
              << ", args=" << error.getArgs() << ", status=" << error.getStatus()
              << ", type=" << error.getExceptionType() << "]\n";
}

cv::Mat color_frame_to_bgr(const std::shared_ptr<ob::Frame> &color_frame)
{
    if (!color_frame) {
        return {};
    }

    const auto video_frame = color_frame->as<ob::VideoFrame>();
    const int width = static_cast<int>(video_frame->getWidth());
    const int height = static_cast<int>(video_frame->getHeight());
    uint8_t *data = color_frame->getData();
    const auto format = color_frame->getFormat();

    switch (format) {
    case OB_FORMAT_BGR:
        return cv::Mat(height, width, CV_8UC3, data).clone();
    case OB_FORMAT_RGB: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height, width, CV_8UC3, data), bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    case OB_FORMAT_BGRA: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height, width, CV_8UC4, data), bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }
    case OB_FORMAT_RGBA: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height, width, CV_8UC4, data), bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    case OB_FORMAT_YUYV:
    case OB_FORMAT_YUY2: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height, width, CV_8UC2, data), bgr, cv::COLOR_YUV2BGR_YUY2);
        return bgr;
    }
    case OB_FORMAT_UYVY: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height, width, CV_8UC2, data), bgr, cv::COLOR_YUV2BGR_UYVY);
        return bgr;
    }
    case OB_FORMAT_NV12: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height + height / 2, width, CV_8UC1, data), bgr, cv::COLOR_YUV2BGR_NV12);
        return bgr;
    }
    case OB_FORMAT_NV21: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height + height / 2, width, CV_8UC1, data), bgr, cv::COLOR_YUV2BGR_NV21);
        return bgr;
    }
    case OB_FORMAT_Y8: {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height, width, CV_8UC1, data), bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    case OB_FORMAT_MJPG: {
        const cv::Mat encoded(1, static_cast<int>(color_frame->getDataSize()), CV_8UC1, data);
        return cv::imdecode(encoded, cv::IMREAD_COLOR);
    }
    default:
        std::cerr << "Unsupported Orbbec color format: " << static_cast<int>(format) << "\n";
        return {};
    }
}
#endif

}  // namespace

bool OrbbecCamera::open(const AppConfig &config)
{
    if (!config.input_image.empty()) {
        return load_static_image(config.input_image);
    }
    if (to_lower(config.camera_backend) == "orbbec") {
        return open_orbbec_camera(config);
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

bool OrbbecCamera::open_orbbec_camera(const AppConfig &config)
{
#ifdef NXVISION_WITH_ORBBEC
    auto start_pipeline = [this](const AppConfig &app_config, bool use_requested_profile) {
        auto pipeline = std::make_shared<ob::Pipeline>();
        auto stream_config = std::make_shared<ob::Config>();
        if (use_requested_profile) {
            const uint32_t width = app_config.capture_width > 0 ? static_cast<uint32_t>(app_config.capture_width) : OB_WIDTH_ANY;
            const uint32_t height = app_config.capture_height > 0 ? static_cast<uint32_t>(app_config.capture_height) : OB_HEIGHT_ANY;
            const uint32_t fps = app_config.capture_fps > 0 ? static_cast<uint32_t>(app_config.capture_fps) : OB_FPS_ANY;
            stream_config->enableVideoStream(
                OB_STREAM_COLOR,
                width,
                height,
                fps,
                OB_FORMAT_ANY);
        } else {
            stream_config->enableVideoStream(OB_STREAM_COLOR);
        }
        pipeline->start(stream_config);
        orbbec_pipeline_ = std::move(pipeline);
    };

    try {
        try {
            start_pipeline(config, true);
        } catch (const ob::Error &error) {
            log_orbbec_error("Requested Orbbec color profile failed, falling back to SDK default", error);
            start_pipeline(config, false);
        }
        orbbec_mode_ = true;
        opened_ = true;
        return true;
    } catch (const ob::Error &error) {
        log_orbbec_error("Failed to open Orbbec camera", error);
    } catch (const std::exception &error) {
        std::cerr << "Failed to open Orbbec camera: " << error.what() << "\n";
    }

    orbbec_pipeline_.reset();
    orbbec_mode_ = false;
    opened_ = false;
    return false;
#else
    (void)config;
    std::cerr << "Orbbec backend requested, but this build was compiled without Orbbec SDK support.\n";
    opened_ = false;
    return false;
#endif
}

bool OrbbecCamera::grab(FrameBundle *frame)
{
    if (!opened_ || frame == nullptr) {
        return false;
    }

    if (static_mode_) {
        frame->color_bgr = static_image_.clone();
        set_steady_timestamp(frame);
        return true;
    }

    if (orbbec_mode_) {
        return grab_orbbec(frame);
    } else if (!capture_.read(frame->color_bgr) || frame->color_bgr.empty()) {
        return false;
    }

    set_steady_timestamp(frame);
    return true;
}

bool OrbbecCamera::grab_orbbec(FrameBundle *frame)
{
#ifdef NXVISION_WITH_ORBBEC
    try {
        auto frames = orbbec_pipeline_->waitForFrameset(100);
        if (!frames) {
            return false;
        }

        auto color_frame = frames->getFrame(OB_FRAME_COLOR);
        frame->color_bgr = color_frame_to_bgr(color_frame);
        if (frame->color_bgr.empty()) {
            return false;
        }

        const uint64_t timestamp_us = color_frame->getSystemTimeStampUs();
        if (timestamp_us > 0) {
            frame->timestamp_s = static_cast<double>(timestamp_us) / 1'000'000.0;
        } else {
            set_steady_timestamp(frame);
        }
        return true;
    } catch (const ob::Error &error) {
        log_orbbec_error("Failed to grab Orbbec frame", error);
    } catch (const std::exception &error) {
        std::cerr << "Failed to grab Orbbec frame: " << error.what() << "\n";
    }
#else
    (void)frame;
#endif
    return false;
}

void OrbbecCamera::close()
{
#ifdef NXVISION_WITH_ORBBEC
    if (orbbec_pipeline_) {
        try {
            orbbec_pipeline_->stop();
        } catch (const ob::Error &error) {
            log_orbbec_error("Failed to stop Orbbec pipeline", error);
        }
        orbbec_pipeline_.reset();
    }
#endif
    if (capture_.isOpened()) {
        capture_.release();
    }
    orbbec_mode_ = false;
    opened_ = false;
}

bool OrbbecCamera::is_open() const
{
    return opened_;
}

}  // namespace nxv
