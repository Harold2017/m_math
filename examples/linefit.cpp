//
// Created by Harold on 2021/4/24.
//

#include <iostream>
#include <iomanip>

#include "m_linefit.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <random>

// code from https://github.com/zhigangjiang/CV-Experiment/blob/main/fit_line.cpp
std::vector<cv::Point2f> generate_data(float k, float b, int n, float line_noise_level = 10, float random_noise_count = 200, bool add_ellipse = false) {
    std::default_random_engine e(static_cast<unsigned int>(time(nullptr)));
    std::uniform_int_distribution<> u(-line_noise_level, line_noise_level);

    std::vector<cv::Point2f> points;
    for (int i = -n; i < n; i++) {
        points.push_back(cv::Point2f(i, k * i + b + u(e)));

        if (!add_ellipse) continue;
        float a_ = 300, b_ = 200;
        float q = 1 - i * i / (a_ * a_);
        if (q > 0) {
            float y = sqrt(b_ * b_ * q);
            float r = atan(k);
            float c = cos(r);
            float s = sin(r);

            auto p1 = cv::Point2f(i, y);
            p1 = cv::Point2f(p1.x * c + p1.y * (-s), p1.x * s + p1.y * c + b);
            points.push_back(p1);
            auto p2 = cv::Point2f(i, -y);
            p2 = cv::Point2f(p2.x * c + p2.y * (-s), p2.x * s + p2.y * c + b);
            points.push_back(p2);
        }
    }

    std::uniform_int_distribution<> u_(-n, n);
    for (int i = 0; i < random_noise_count; i++) {
        points.push_back(cv::Point2f(u_(e), u_(e)));
    }
    std::shuffle(points.begin(), points.end(), e);
    return points;
}

void draw_data(cv::Mat& board, const std::vector<cv::Point2f>& points) {
    int h = board.cols, w = board.rows;
    for (auto const& point : points) {
        cv::Point2f p = cv::Point2f(h / 2 + point.x, w / 2 - point.y);//origin change center
        cv::circle(board, p, 1, cv::Scalar(255, 255, 255), -1);
    }
}

void draw_line(cv::Mat& board, float k, float b, cv::Scalar color) {
    int h = board.cols, w = board.rows;
    cv::Point2f p1 = cv::Point2f(-h, -w * k + b);
    p1 = cv::Point2f(h / 2 + p1.x, w / 2 - p1.y);
    cv::Point2f p2 = cv::Point2f(h, w * k + b);
    p2 = cv::Point2f(h / 2 + p2.x, w / 2 - p2.y);
    cv::line(board, p1, p2, color, 1);
}


float calc_error(const cv::Point2f& point, float k, float b) {
    // distance of point to line
    float A = k, B = -1, C = b;
    return abs(A * point.x + B * point.y + C) / sqrt(A * A + B * B);
}

float calc_error_y2(const cv::Point2f& point, float k, float b) {
    return pow(abs(b + point.x * k - point.y), 2);
}

double calc_loss(const std::vector<cv::Point2f>& points, double w1, double w2) {
    // MSE
    double sse = 0;
    for (auto const& point : points) {
        double se = calc_error_y2(point, w2, w1);
        sse += se;
    }
    double mes = sse / (2 * points.size());
    return mes;

}

int main() {
    std::cout << std::fixed << std::setprecision(8);

    float k = 0.5;
    float b = 20;
    cv::Mat board = cv::Mat(800, 800, CV_8UC3);

    //std::vector<cv::Point2f> points = generate_data(k, b, 300, 10, 0, false);
    std::vector<cv::Point2f> points = generate_data(k, b, 300, 10, 400, true);

    std::cout << "original   k:" << k << " b:" << b << std::endl;
    draw_data(board, points);
    draw_line(board, k, b, cv::Scalar(255, 255, 255));

    float k_, b_;

    //if (M_MATH::least_squares_a(points.begin(), points.end(),
    //if (M_MATH::least_squares_b(points.begin(), points.end(),
    if (M_MATH::LineFitLeastSquares(points.begin(), points.end(),
        [](cv::Point2f const& pt) { return pt.x; },
        [](cv::Point2f const& pt) { return pt.y; },
        k_, b_)) {
        std::cout << "HM least squares:" << k_ << " b:" << b_ << std::endl;
        draw_line(board, k_, b_, cv::Scalar(255, 0, 0));
    }

    // cv::fitLine is much more robust than least sqaures under random noise
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
    k_ = line[1] / line[0];
    b_ = line[3] - (line[1] / line[0]) * line[2];
    std::cout << "CV fitLine:" << k_ << " b:" << b_ << std::endl;
    draw_line(board, k_, b_, cv::Scalar(0, 255, 0));

    // HM RANSAC
    std::vector<Eigen::Vector2d> points_eig;
    points_eig.reserve(points.size());
    for (auto const& pt : points)
        points_eig.emplace_back(pt.x, pt.y);
    auto kb = M_MATH::LineFitRANSAC(points_eig, 10, 10, 1000);  // distance threshold == noise_level
    k_ = kb(0);
    b_ = kb(1);
    std::cout << "HM RANSAC:" << k_ << " b:" << b_ << std::endl;
    draw_line(board, k_, b_, cv::Scalar(0, 0, 255));

    // HM gradient descent
    // almost the same with least squares since they both use y diff MSE as loss
    if (M_MATH::LineFitGradientDescent(points.begin(), points.end(),
        [](cv::Point2f const& pt) { return pt.x; },
        [](cv::Point2f const& pt) { return pt.y; },
        k_, b_,
        0.00004, 1e8, 1e-8)) {
        std::cout << "HM gradient descent:" << k_ << " b:" << b_ << std::endl;
        draw_line(board, k_, b_, cv::Scalar(255, 0, 255));
    }

    cv::imshow("board", board);
    cv::waitKey();

    return 0;
}