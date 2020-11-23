//
// Created by Harold on 2020/11/23.
//

#include <random>
#include <vector>
#include "m_geo_opencv.h"

using namespace M_MATH;

int main() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(-1,1);
    auto gen = [&]() -> float {
        return dist(rng);
    };

    int const N = 1000;
    std::vector<cv::Point3f> pts;
    pts.reserve(N);
    for (auto i = 0; i < N; ++i)
        pts.emplace_back(gen(), gen(), gen());

    // xy plane
    auto pts_x = PlanePts(pts.begin(), pts.end(), cv::Point3f{0, 0, 1}, cv::Point3f{0, 0, 0}, 0.05f);
    for (auto const& e : pts_x)
        assert(std::abs(e.z) <= 0.05);

    // x axis direction
    auto pts_xaxis = LinePts(pts.begin(), pts.end(),
                             cv::Point3f{0, 0, 1}, cv::Point3f{0, 0, 0},
                             cv::Point3f{0, 1, 0}, cv::Point3f{0, 0, 0},
                             0.05f,
                             0.05f);
    for (auto const& e : pts_xaxis)
        assert(std::abs(e.z) <= 0.05 && std::abs(e.y) <= 0.05);

    return 0;
}
