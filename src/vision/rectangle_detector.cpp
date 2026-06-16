#include "vision/rectangle_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace nxv {
namespace {

double polygon_angle_deg(const cv::Point2f &prev, const cv::Point2f &cur, const cv::Point2f &next)
{
    const cv::Point2f a = prev - cur;
    const cv::Point2f b = next - cur;
    const double dot = a.x * b.x + a.y * b.y;
    const double na = std::sqrt(a.x * a.x + a.y * a.y);
    const double nb = std::sqrt(b.x * b.x + b.y * b.y);
    if (na <= 1e-6 || nb <= 1e-6) {
        return 0.0;
    }
    const double cosine = std::max(-1.0, std::min(1.0, dot / (na * nb)));
    return std::acos(cosine) * 180.0 / CV_PI;
}

cv::Point2f diagonal_center(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() != 4) {
        return cv::Point2f(0.0f, 0.0f);
    }
    cv::Point2f a = pts[0];
    cv::Point2f b = pts[2];
    cv::Point2f c = pts[1];
    cv::Point2f d = pts[3];
    const double a1 = b.y - a.y;
    const double b1 = a.x - b.x;
    const double c1 = b.x * a.y - a.x * b.y;
    const double a2 = d.y - c.y;
    const double b2 = c.x - d.x;
    const double c2 = d.x * c.y - c.x * d.y;
    const double denom = a1 * b2 - a2 * b1;
    if (std::abs(denom) < 1e-6) {
        return (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;
    }
    return cv::Point2f(static_cast<float>((b1 * c2 - b2 * c1) / denom),
                       static_cast<float>((a2 * c1 - a1 * c2) / denom));
}

double side_ratio(const std::vector<cv::Point2f> &pts)
{
    double min_len = std::numeric_limits<double>::max();
    double max_len = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const cv::Point2f delta = pts[(i + 1) % pts.size()] - pts[i];
        const double len = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        min_len = std::min(min_len, len);
        max_len = std::max(max_len, len);
    }
    if (min_len <= 1e-6) {
        return std::numeric_limits<double>::max();
    }
    return max_len / min_len;
}

std::vector<cv::Point> to_points(const std::vector<cv::Point2f> &pts)
{
    std::vector<cv::Point> out;
    out.reserve(pts.size());
    for (const cv::Point2f &pt : pts) {
        out.emplace_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
    }
    return out;
}

}  // namespace

std::vector<cv::Point2f> RectangleDetector::order_corners(const std::vector<cv::Point2f> &points)
{
    std::vector<cv::Point2f> ordered(4);
    std::vector<cv::Point2f> pts = points;
    std::sort(pts.begin(), pts.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.y < b.y;
    });

    std::vector<cv::Point2f> top = {pts[0], pts[1]};
    std::vector<cv::Point2f> bottom = {pts[2], pts[3]};
    std::sort(top.begin(), top.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.x < b.x;
    });
    std::sort(bottom.begin(), bottom.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.x < b.x;
    });

    ordered[0] = top[0];
    ordered[1] = top[1];
    ordered[2] = bottom[1];
    ordered[3] = bottom[0];
    return ordered;
}

RectangleDetection RectangleDetector::detect(const PreprocessOutput &preprocess,
                                             const VisionParams &params,
                                             bool make_debug_images) const
{
    RectangleDetection best;
    if (preprocess.combined.empty()) {
        return best;
    }

    //找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(preprocess.combined, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    //评估轮廓，选分数最高的
    if (make_debug_images) {
        cv::cvtColor(preprocess.combined, best.contour_overlay, cv::COLOR_GRAY2BGR);
    }
    double best_score = -1.0;

    for (size_t i = 0; i < contours.size(); ++i) {
        const double area = std::abs(cv::contourArea(contours[i]));
        if (area < params.min_area_px) {
            continue;
        }

        //保留 4 个点且凸的轮廓
        const double perimeter = cv::arcLength(contours[i], true);
        std::vector<cv::Point> approx_int;
        cv::approxPolyDP(contours[i], approx_int, params.approx_epsilon_ratio * perimeter, true);
        if (approx_int.size() != 4 || !cv::isContourConvex(approx_int)) {
            continue;
        }

        std::vector<cv::Point2f> pts;
        pts.reserve(4);
        for (const cv::Point &point : approx_int) {
            pts.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
        }
        pts = order_corners(pts);

        //检查四个角是否接近 90 度且长宽比是否合理
        bool angles_ok = true;
        double angle_deviation = 0.0;
        for (size_t j = 0; j < 4; ++j) {
            const double angle = polygon_angle_deg(pts[(j + 3) % 4], pts[j], pts[(j + 1) % 4]);
            angles_ok = angles_ok && angle >= params.min_angle_deg && angle <= params.max_angle_deg;
            angle_deviation += std::abs(angle - 90.0);
        }
        if (!angles_ok || side_ratio(pts) > params.max_side_ratio) {
            continue;
        }

        double child_bonus = 0.0;
        if (!hierarchy.empty() && hierarchy[i][2] >= 0) {
            child_bonus = 20.0;
        }
        const double score = area * 0.001 + (100.0 - angle_deviation * 0.25) + child_bonus;

        if (make_debug_images) {
            cv::polylines(best.contour_overlay, to_points(pts), true, cv::Scalar(0, 180, 255), 1);
        }

        if (score > best_score) {
            best_score = score;
            best.valid = true;
            best.score = score;
            best.corners = pts;
            best.center = diagonal_center(pts);
        }
    }

    if (best.valid) {
        for (cv::Point2f &pt : best.corners) {
            pt.x *= static_cast<float>(preprocess.scale_x_to_src);
            pt.y *= static_cast<float>(preprocess.scale_y_to_src);
        }
        best.center.x *= static_cast<float>(preprocess.scale_x_to_src);
        best.center.y *= static_cast<float>(preprocess.scale_y_to_src);

        std::vector<cv::Point2f> debug_pts = best.corners;
        for (cv::Point2f &pt : debug_pts) {
            pt.x /= static_cast<float>(preprocess.scale_x_to_src);
            pt.y /= static_cast<float>(preprocess.scale_y_to_src);
        }
        if (make_debug_images) {
            cv::polylines(best.contour_overlay, to_points(debug_pts), true, cv::Scalar(0, 255, 0), 2);

            const float view_w = 445.0f;
            const float view_h = 315.0f;
            std::vector<cv::Point2f> dst = {{0.0f, 0.0f}, {view_w, 0.0f}, {view_w, view_h}, {0.0f, view_h}};
            cv::Mat H = cv::getPerspectiveTransform(debug_pts, dst);
            cv::warpPerspective(preprocess.resized_bgr, best.perspective_view, H, cv::Size(static_cast<int>(view_w), static_cast<int>(view_h)));
        }
    }

    return best;
}

}  // namespace nxv
