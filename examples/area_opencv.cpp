//
// Created by Harold on 2020/10/14.
//

#include <random>
#include <iostream>
#include "m_area_opencv.h"

int main() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::normal_distribution<float> dist(-0.05,0.05);
    auto gen = [&]() -> float {
        return dist(rng);
    };

    size_t N = 100;
    std::vector<float> pts(N);
    for (auto i = 0; i < N; ++i)
        pts[i] = gen();

    auto area = M_MATH::Area(pts.data(), 10, 10);

    std::cout << area << std::endl;

    return 0;
}
