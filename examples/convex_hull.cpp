//
// Created by Harold on 2021/6/3.
//
#include <iostream>

#include "m_convex_hull.h"

#include <opencv2/imgproc.hpp>

#include "stopwatch.h"

// Sklansky from opencv
std::vector<Eigen::Vector2d> ConvexHull(std::vector<Eigen::Vector2d> const& polygon_pts) {
    std::vector<cv::Point2d> pts(polygon_pts.size());
    std::transform(polygon_pts.begin(), polygon_pts.end(), pts.begin(), [](Eigen::Vector2d const& p) { return cv::Point2d{ p(0), p(1) }; });
    cv::Mat mat(pts);
    mat.convertTo(mat, CV_32FC1);
    std::vector<int> indices;
    cv::convexHull(mat, indices, false, false);
    std::vector<Eigen::Vector2d> res(indices.size());
    std::transform(indices.begin(), indices.end(), res.begin(), [&](int idx) { return polygon_pts[idx]; });
    return res;
}

int main() {
    srand((unsigned int)time(0));
    size_t N = 1000;
    std::vector<Eigen::Vector2d> points(N);
    std::generate(points.begin(), points.end(), []() { return Eigen::Vector2d::Random(); });
    //std::vector<Eigen::Vector2d> points{ {-7, 8}, {-4, 6}, {2, 6}, {6, 4}, {8, 6}, {7, -2}, {4, -6}, {8, -7}, {0, 0}, {3, -2}, {6, -10}, {0, -6}, {-9, -5}, {-8, -2}, {-8, 0}, {-10, 3}, {-2, 2}, {-10, 4} };

    std::vector<Eigen::Vector2d> ch_1, ch_2;
    {
        TIME_BLOCK("HM");
        ch_1 = M_MATH::ConvexHull(points);
    }
    {
        TIME_BLOCK("CV");
        ch_2 = ConvexHull(points);
    }

    std::cout << (ch_1.size() == ch_2.size()) << std::endl;
    
    /*
    std::cout << "ch_1: " << ch_1.size() << '\n';
    for (auto const& p : ch_1) std::cout << p.transpose() << ", ";
    std::cout << std::endl;

    std::cout << "ch_2: " << ch_2.size() << '\n';
    for (auto const& p : ch_2) std::cout << p.transpose() << ", ";
    std::cout << std::endl;
    */

    return 0;
}