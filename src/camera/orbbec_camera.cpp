#include "camera/orbbec_camera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef NXVISION_WITH_ORBBEC
#include <libobsensor/ObSensor.hpp>
#endif

#include <chrono>
#include <cctype>
#include <iostream>
#include <memory>

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

bool video_profile_matches(const std::shared_ptr<ob::StreamProfile> &profile, const AppConfig &config)
{
    const auto video = profile->as<ob::VideoStreamProfile>();
    if (config.capture_width > 0 && static_cast<int>(video->getWidth()) != config.capture_width) {
        return false;
    }
    if (config.capture_height > 0 && static_cast<int>(video->getHeight()) != config.capture_height) {
        return false;
    }
    if (config.capture_fps > 0 && static_cast<int>(video->getFps()) != config.capture_fps) {
        return false;
    }
    return true;
}

bool depth_profile_supported_for_d2c(const std::shared_ptr<ob::Pipeline> &pipeline,
                                     const std::shared_ptr<ob::StreamProfile> &color_profile,
                                     const std::shared_ptr<ob::StreamProfile> &depth_profile)
{
    auto supported = pipeline->getD2CDepthProfileList(color_profile, ALIGN_D2C_HW_MODE);
    const auto depth_video = depth_profile->as<ob::VideoStreamProfile>();
    for (uint32_t i = 0; i < supported->getCount(); ++i) {
        auto candidate = supported->getProfile(i)->as<ob::VideoStreamProfile>();
        if (candidate->getWidth() == depth_video->getWidth() &&
            candidate->getHeight() == depth_video->getHeight() &&
            candidate->getFps() == depth_video->getFps() &&
            candidate->getFormat() == depth_video->getFormat()) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<ob::Config> make_hw_d2c_config(const std::shared_ptr<ob::Pipeline> &pipeline,
                                               const AppConfig &app_config)
{
    auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    auto depth_profiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);

    for (uint32_t i = 0; i < color_profiles->getCount(); ++i) {
        auto color_profile = color_profiles->getProfile(i);
        if (!video_profile_matches(color_profile, app_config)) {
            continue;
        }
        const auto color_video = color_profile->as<ob::VideoStreamProfile>();
        for (uint32_t j = 0; j < depth_profiles->getCount(); ++j) {
            auto depth_profile = depth_profiles->getProfile(j);
            const auto depth_video = depth_profile->as<ob::VideoStreamProfile>();
            if (color_video->getFps() != depth_video->getFps()) {
                continue;
            }
            if (!depth_profile_supported_for_d2c(pipeline, color_profile, depth_profile)) {
                continue;
            }

            auto stream_config = std::make_shared<ob::Config>();
            stream_config->enableStream(color_profile);
            stream_config->enableStream(depth_profile);
            stream_config->setAlignMode(ALIGN_D2C_HW_MODE);
            stream_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
            return stream_config;
        }
    }
    return nullptr;
}

std::shared_ptr<ob::Config> make_rgbd_config(const AppConfig &app_config, bool use_requested_profile)
{
    auto stream_config = std::make_shared<ob::Config>();
    if (use_requested_profile) {
        const uint32_t width = app_config.capture_width > 0 ? static_cast<uint32_t>(app_config.capture_width) : OB_WIDTH_ANY;
        const uint32_t height = app_config.capture_height > 0 ? static_cast<uint32_t>(app_config.capture_height) : OB_HEIGHT_ANY;
        const uint32_t fps = app_config.capture_fps > 0 ? static_cast<uint32_t>(app_config.capture_fps) : OB_FPS_ANY;
        stream_config->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_ANY);
    } else {
        stream_config->enableVideoStream(OB_STREAM_COLOR);
    }
    stream_config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);
    stream_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    return stream_config;
}

std::shared_ptr<ob::Config> make_color_config(const AppConfig &app_config, bool use_requested_profile)
{
    auto stream_config = std::make_shared<ob::Config>();
    if (use_requested_profile) {
        const uint32_t width = app_config.capture_width > 0 ? static_cast<uint32_t>(app_config.capture_width) : OB_WIDTH_ANY;
        const uint32_t height = app_config.capture_height > 0 ? static_cast<uint32_t>(app_config.capture_height) : OB_HEIGHT_ANY;
        const uint32_t fps = app_config.capture_fps > 0 ? static_cast<uint32_t>(app_config.capture_fps) : OB_FPS_ANY;
        stream_config->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_ANY);
    } else {
        stream_config->enableVideoStream(OB_STREAM_COLOR);
    }
    return stream_config;
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

cv::Mat depth_frame_to_mm(const std::shared_ptr<ob::Frame> &depth_frame)
{
    if (!depth_frame) {
        return {};
    }
    const auto video_frame = depth_frame->as<ob::VideoFrame>();
    const int width = static_cast<int>(video_frame->getWidth());
    const int height = static_cast<int>(video_frame->getHeight());

    if (depth_frame->getFormat() != OB_FORMAT_Y16) {
        std::cerr << "Unsupported Orbbec depth format: " << static_cast<int>(depth_frame->getFormat()) << "\n";
        return {};
    }

    const auto depth = depth_frame->as<ob::DepthFrame>();
    const cv::Mat raw(height, width, CV_16UC1, depth_frame->getData());
    cv::Mat depth_mm;
    raw.convertTo(depth_mm, CV_32FC1, depth->getValueScale());
    return depth_mm;
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
    try {
        auto pipeline = std::make_shared<ob::Pipeline>();
        try {
            pipeline->enableFrameSync();
        } catch (const ob::Error &error) {
            log_orbbec_error("Orbbec frame sync unavailable; continuing without sync", error);
        }

        auto stream_config = make_hw_d2c_config(pipeline, config);
        if (stream_config) {
            pipeline->start(stream_config);
            orbbec_pipeline_ = std::move(pipeline);
            software_depth_align_ = false;
            orbbec_mode_ = true;
            opened_ = true;
            std::cout << "Orbbec RGBD enabled with hardware depth-to-color alignment\n";
            return true;
        }
    } catch (const ob::Error &error) {
        log_orbbec_error("Hardware depth-to-color alignment failed", error);
    } catch (const std::exception &error) {
        std::cerr << "Hardware depth-to-color alignment failed: " << error.what() << "\n";
    }

    try {
        auto pipeline = std::make_shared<ob::Pipeline>();
        try {
            pipeline->enableFrameSync();
        } catch (const ob::Error &error) {
            log_orbbec_error("Orbbec frame sync unavailable; continuing without sync", error);
        }

        try {
            pipeline->start(make_rgbd_config(config, true));
        } catch (const ob::Error &error) {
            log_orbbec_error("Requested Orbbec RGBD profile failed, falling back to SDK default RGBD", error);
            pipeline->start(make_rgbd_config(config, false));
        }
        orbbec_pipeline_ = std::move(pipeline);
        depth_to_color_align_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
        software_depth_align_ = true;
        orbbec_mode_ = true;
        opened_ = true;
        std::cout << "Orbbec RGBD enabled with software depth-to-color alignment\n";
        return true;
    } catch (const ob::Error &error) {
        log_orbbec_error("Failed to open Orbbec RGBD streams", error);
    } catch (const std::exception &error) {
        std::cerr << "Failed to open Orbbec RGBD streams: " << error.what() << "\n";
    }

    try {
        auto pipeline = std::make_shared<ob::Pipeline>();
        try {
            pipeline->start(make_color_config(config, true));
        } catch (const ob::Error &error) {
            log_orbbec_error("Requested Orbbec color profile failed, falling back to SDK default color", error);
            pipeline->start(make_color_config(config, false));
        }
        orbbec_pipeline_ = std::move(pipeline);
        depth_to_color_align_.reset();
        software_depth_align_ = false;
        orbbec_mode_ = true;
        opened_ = true;
        std::cerr << "Orbbec opened in color-only mode; depth frame unavailable\n";
        return true;
    } catch (const ob::Error &error) {
        log_orbbec_error("Failed to open Orbbec camera", error);
    } catch (const std::exception &error) {
        std::cerr << "Failed to open Orbbec camera: " << error.what() << "\n";
    }

    orbbec_pipeline_.reset();
    depth_to_color_align_.reset();
    software_depth_align_ = false;
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

        std::shared_ptr<ob::FrameSet> aligned_frames = frames;
        if (software_depth_align_ && depth_to_color_align_) {
            auto aligned = depth_to_color_align_->process(frames);
            if (aligned) {
                aligned_frames = aligned->as<ob::FrameSet>();
            }
        }

        auto color_frame = aligned_frames->getFrame(OB_FRAME_COLOR);
        frame->color_bgr = color_frame_to_bgr(color_frame);
        if (frame->color_bgr.empty()) {
            return false;
        }
        frame->depth_mm = depth_frame_to_mm(aligned_frames->getFrame(OB_FRAME_DEPTH));

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
    depth_to_color_align_.reset();
#endif
    if (capture_.isOpened()) {
        capture_.release();
    }
    orbbec_mode_ = false;
    software_depth_align_ = false;
    opened_ = false;
}

bool OrbbecCamera::is_open() const
{
    return opened_;
}

}  // namespace nxv
