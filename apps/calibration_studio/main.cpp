#include "camera/orbbec_camera.hpp"
#include "common/app_context.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iostream>

int main(int argc, char **argv)
{
    const std::string config_dir = argc > 1 ? argv[1] : "config";
    nxv::AppContext context = nxv::make_app_context(config_dir);

    nxv::OrbbecCamera camera;
    if (!camera.open(context.config.app)) {
        std::cerr << "Failed to open camera/input for calibration\n";
        return 1;
    }

    std::filesystem::create_directories("data/calibration_images");
    int saved = 0;
    std::cout << "Calibration studio: q=quit, s=save calibration image\n";

    for (;;) {
        nxv::FrameBundle frame;
        if (!camera.grab(&frame) || frame.color_bgr.empty()) {
            continue;
        }

        cv::Mat view = frame.color_bgr.clone();
        cv::putText(view,
                    "Calibration Studio  q=quit  s=save",
                    cv::Point(18, 34),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(0, 255, 255),
                    2,
                    cv::LINE_AA);
        cv::line(view, cv::Point(view.cols / 2, 0), cv::Point(view.cols / 2, view.rows), cv::Scalar(0, 255, 0), 1);
        cv::line(view, cv::Point(0, view.rows / 2), cv::Point(view.cols, view.rows / 2), cv::Scalar(0, 255, 0), 1);

        cv::imshow("nx_calibration", view);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == 's') {
            const std::string path = "data/calibration_images/frame_" + std::to_string(saved++) + ".png";
            cv::imwrite(path, frame.color_bgr);
            std::cout << "saved " << path << "\n";
        }
    }

    return 0;
}
