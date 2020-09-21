//
// Created by Harold on 2020/9/21.
//

#include <iostream>
#include <random>
#include <vector>
#include "m_filter.h"

using namespace M_MATH;

int main() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(-0.5,0.5);
    auto gen = [&]() -> float {
        return dist(rng);
    };

    size_t const N = 100;
    size_t const window_size = 11;
    size_t const res_size = N;
    std::vector<double> x;
    std::vector<double> n_x;
    x.reserve(N);
    // notice resize here to make sure n_x.data() array is initialized
    n_x.resize(res_size);
    for (auto i = 0; i < N; ++i)
        x.emplace_back(gen());

    std::cout << "original: ";
    for (auto e : x)
        std::cout << e << ' ';
    std::cout << std::endl;

    if (!GaussianFilter::filter(x.data(), x.size(), window_size, 3,
                           M_MATH::GaussianFilter::NO_PAD, res_size,
                           n_x.data()))
        std::cout << "error" << std::endl;

    std::cout << "filtered: ";
    for (auto e : n_x)
        std::cout << e << ' ';
    std::cout << std::endl;

    return 0;
}
