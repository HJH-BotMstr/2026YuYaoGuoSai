// 摄像头实时仪表盘识别 (C++ 翻译自 detect/realtime_gauge.py)
// 编译: g++ -O2 -std=c++17 realtime_gauge.cpp -o realtime_gauge $(pkg-config --cflags --libs opencv4)

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <stdexcept>

const int ANGLE_RES = 720;
const int RADIUS_RES = 100;

// 色区定义: (lo, hi, 中文状态, 标签)
struct Zone { double lo, hi; const char* text; const char* tag; };
Zone ZONES[] = {
    {315, 360, "仪表盘偏高，状态异常", "RED"},
    {0,   60,  "仪表盘偏高，状态异常", "RED"},
    {220, 315, "仪表盘正常，状态良好", "GREEN"},
    {120, 220, "仪表盘偏低，状态异常", "YELLOW"},
};

// ── 霍夫圆检测 ──────────────────────────────────────────
cv::Vec3f detect_circle(const cv::Mat& gray_in) {
    cv::Mat gray;
    if (gray_in.channels() == 3)
        cv::cvtColor(gray_in, gray, cv::COLOR_BGR2GRAY);
    else
        gray = gray_in.clone();

    int h = gray.rows, w = gray.cols;
    cv::Mat enhanced, smooth;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(gray, enhanced);
    cv::bilateralFilter(enhanced, smooth, 9, 75, 75);

    cv::Mat edges;
    cv::Canny(smooth, edges, 40, 120);

    struct Strategy { double p1, p2; int min_r; bool use_edges; };
    Strategy strategies[] = {
        {80, 30, 60, true}, {60, 25, 55, false}, {100, 35, 60, true},
        {50, 20, 50, false}, {70, 28, 55, false},
    };

    std::vector<cv::Vec3f> all_circles;
    for (auto& s : strategies) {
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(s.use_edges ? edges : smooth, circles, cv::HOUGH_GRADIENT,
                          1, std::max(w, h), s.p1, s.p2, s.min_r);
        for (auto& c : circles) all_circles.push_back(c);
    }

    if (all_circles.empty()) throw std::runtime_error("no circle");

    std::vector<float> radii;
    for (auto& c : all_circles) radii.push_back(c[2]);
    std::sort(radii.begin(), radii.end());
    float median_r = radii[radii.size() / 2];

    cv::Vec3f best = all_circles[0];
    float best_diff = std::abs(all_circles[0][2] - median_r);
    for (auto& c : all_circles) {
        float diff = std::abs(c[2] - median_r);
        if (diff < best_diff) { best_diff = diff; best = c; }
    }
    return best;
}

// ── ROI 裁剪 ────────────────────────────────────────────
cv::Mat extract_roi(const cv::Mat& image, double cx, double cy, double r) {
    int h = image.rows, w = image.cols;
    int side = int(r * 2.2);
    int half = side / 2;
    int x1 = int(cx) - half, y1 = int(cy) - half;

    int pad_left = std::max(0, -x1), pad_top = std::max(0, -y1);
    int pad_right = std::max(0, x1 + side - w), pad_bottom = std::max(0, y1 + side - h);

    int x1c = std::max(0, x1), y1c = std::max(0, y1);
    int x2c = std::min(w, x1 + side), y2c = std::min(h, y1 + side);
    cv::Mat roi = image(cv::Rect(x1c, y1c, x2c - x1c, y2c - y1c)).clone();

    if (pad_left || pad_top || pad_right || pad_bottom)
        cv::copyMakeBorder(roi, roi, pad_top, pad_bottom, pad_left, pad_right, cv::BORDER_REPLICATE);
    if (roi.rows != side || roi.cols != side)
        cv::resize(roi, roi, cv::Size(side, side), 0, 0, cv::INTER_LINEAR);
    cv::resize(roi, roi, cv::Size(500, 500), 0, 0, cv::INTER_LINEAR);
    return roi;
}

// ── LAB 增强 ────────────────────────────────────────────
cv::Mat enhance_roi(const cv::Mat& roi) {
    cv::Mat lab, lab_f;
    cv::cvtColor(roi, lab, cv::COLOR_BGR2Lab);
    lab.convertTo(lab_f, CV_32F);

    std::vector<cv::Mat> channels(3);
    cv::split(lab_f, channels);

    channels[0].convertTo(channels[0], CV_8U);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(channels[0], channels[0]);
    channels[0].convertTo(channels[0], CV_32F);

    channels[1] = (channels[1] - 128) * 2.0 + 128;
    channels[2] = (channels[2] - 128) * 2.0 + 128;

    cv::merge(channels, lab_f);
    lab_f = cv::max(cv::min(lab_f, 255), 0);
    lab_f.convertTo(lab, CV_8U);
    cv::cvtColor(lab, lab, cv::COLOR_Lab2BGR);
    return lab;
}

// ── K-means 分割（已废弃，改用 LAB 阈值）───────────────
/*
void kmeans_segment(const cv::Mat& bgr, cv::Mat& labels, cv::Mat& centers) { ... }
*/

struct ColorCenters { cv::Point2f red, yellow; bool ok; };

// ── LAB 阈值分割 → 红黄质心 ─────────────────────────────
ColorCenters lab_threshold_centers(const cv::Mat& roi) {
    ColorCenters cc; cc.ok = false;
    cv::Mat lab;
    cv::cvtColor(roi, lab, cv::COLOR_BGR2Lab);

    // LAB 阈值 (OpenCV: L[0-255], a[0-255], b[0-255])
    // 红: L[0,100] a[7,83] b[-55,71] → L[0,255] a[135,211] b[73,199]
    // 黄: L[0,100] a[-57,47] b[17,97] → L[0,255] a[71,175] b[145,225]
    cv::Mat red_mask, yellow_mask;
    cv::inRange(lab, cv::Scalar(0, 135, 73), cv::Scalar(255, 211, 199), red_mask);
    cv::inRange(lab, cv::Scalar(0, 71, 145), cv::Scalar(255, 175, 225), yellow_mask);

    auto centroid_from_mask = [&](cv::Mat& mask) -> cv::Point2f {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        cv::erode(mask, mask, kernel, cv::Point(-1,-1), 1);

        // 有效区域仅限圆环带
        cv::circle(mask, cv::Point(250, 250), 100, cv::Scalar(0), -1);
        cv::Mat outer = cv::Mat::zeros(500, 500, CV_8U);
        cv::circle(outer, cv::Point(250, 250), 220, cv::Scalar(255), -1);
        cv::bitwise_and(mask, outer, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return cv::Point2f(0,0);
        auto& largest = *std::max_element(contours.begin(), contours.end(),
            [](auto& a, auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
        cv::Moments M = cv::moments(largest);
        return cv::Point2f(M.m10 / M.m00, M.m01 / M.m00);
    };

    cc.red = centroid_from_mask(red_mask);
    cc.yellow = centroid_from_mask(yellow_mask);
    cc.ok = true;
    return cc;
}

// ── 上方向 ──────────────────────────────────────────────
double compute_up(const cv::Point2f& red) {
    return fmod(atan2(red.y - 250.0, red.x - 250.0) * 180.0 / CV_PI + 270.0, 360.0);
}

// ── 极坐标展开 ──────────────────────────────────────────
cv::Mat polar_unwrap(const cv::Mat& gray) {
    int h = gray.rows, w = gray.cols;
    double cx = w / 2.0, cy = h / 2.0;
    double max_r = std::min(cx, cy);

    cv::Mat map_x(RADIUS_RES, ANGLE_RES, CV_32F);
    cv::Mat map_y(RADIUS_RES, ANGLE_RES, CV_32F);
    for (int row = 0; row < RADIUS_RES; row++) {
        double r = row * max_r / RADIUS_RES;
        for (int col = 0; col < ANGLE_RES; col++) {
            double rad = col * 360.0 / ANGLE_RES * CV_PI / 180.0;
            map_x.at<float>(row, col) = cx + r * cos(rad);
            map_y.at<float>(row, col) = cy + r * sin(rad);
        }
    }
    cv::Mat polar;
    cv::remap(gray, polar, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    return polar;
}

// ── 指针角度 ────────────────────────────────────────────
double detect_ptr(const cv::Mat& polar) {
    int clip = RADIUS_RES * 0.20;
    cv::Mat polar_clip = polar(cv::Rect(0, clip, ANGLE_RES, RADIUS_RES - clip));
    cv::Mat col_means;
    cv::reduce(polar_clip, col_means, 0, cv::REDUCE_AVG, CV_32F);

    double min_val; cv::Point min_loc;
    cv::minMaxLoc(col_means, &min_val, nullptr, &min_loc, nullptr);
    int min_col = min_loc.x;

    double offset = 0;
    if (min_col > 0 && min_col < ANGLE_RES - 1) {
        double y0 = col_means.at<float>(min_col - 1);
        double y1 = col_means.at<float>(min_col);
        double y2 = col_means.at<float>(min_col + 1);
        double d = y0 - 2.0 * y1 + y2;
        if (std::abs(d) > 1e-10) offset = (y0 - y2) / (2.0 * d);
    }
    return fmod((min_col + offset) * 360.0 / ANGLE_RES, 360.0);
}

// ── 分类 ────────────────────────────────────────────────
struct GaugeResult { const char* text; const char* tag; };
GaugeResult classify(double ptr_angle, double up_angle) {
    double rel = fmod(ptr_angle - up_angle + 270.0, 360.0);
    for (auto& z : ZONES) {
        if (rel >= z.lo && rel < z.hi) return {z.text, z.tag};
    }
    return {"状态未知", "UNKNOWN"};
}

// ── 主循环 ──────────────────────────────────────────────
int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return 1;
    }

    std::cout << "摄像头实时识别中... 按 q 退出" << std::endl;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Vec3f circle = detect_circle(gray);
        double cx = circle[0], cy = circle[1], r = circle[2];

        cv::Mat roi = extract_roi(frame, cx, cy, r);
        cv::Mat roi_enh = enhance_roi(roi);

        auto cc = lab_threshold_centers(roi_enh);
        double up_angle = compute_up(cc.red);

        cv::Mat gray_roi;
        cv::cvtColor(roi, gray_roi, cv::COLOR_BGR2GRAY);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(gray_roi, gray_roi);
        double ptr_angle = detect_ptr(polar_unwrap(gray_roi));

        // ── 绘制标注 ──
        cv::circle(frame, cv::Point(cx, cy), r, cv::Scalar(0,255,0), 2);
        cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0,255,255), -1);

        double scale = r * 2.2 / 500.0;
        int off_x = cx - r * 2.2 / 2;
        int off_y = cy - r * 2.2 / 2;
        auto map_pt = [&](const cv::Point2f& p) {
            return cv::Point(off_x + p.x * scale, off_y + p.y * scale);
        };

        cv::circle(frame, map_pt(cc.red), 8, cv::Scalar(0,0,255), -1);
        cv::circle(frame, map_pt(cc.yellow), 8, cv::Scalar(0,255,255), -1);

        double up_rad = up_angle * CV_PI / 180.0;
        double up_len = r * 0.7;
        cv::arrowedLine(frame, cv::Point(cx, cy),
            cv::Point(cx + up_len*cos(up_rad), cy + up_len*sin(up_rad)),
            cv::Scalar(255,0,0), 2, 8, 0, 0.1);

        double ptr_rad = ptr_angle * CV_PI / 180.0;
        double ptr_len = r * 0.85;
        cv::arrowedLine(frame, cv::Point(cx, cy),
            cv::Point(cx + ptr_len*cos(ptr_rad), cy + ptr_len*sin(ptr_rad)),
            cv::Scalar(0,0,255), 3, 8, 0, 0.1);

        auto result = classify(ptr_angle, up_angle);
        std::cout << result.text << "  ptr=" << ptr_angle << " up=" << up_angle << std::endl;

        cv::imshow("Gauge Recognition", frame);
        if (cv::waitKey(1) == 'q') break;
    }

    cv::destroyAllWindows();
    std::cout << std::endl;
    return 0;
}
