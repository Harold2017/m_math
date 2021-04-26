//
// Created by Harold on 2021/4/26.
//

#include "m_mr.h"
#include "../utils/rand_gen.h"

#include <iostream>

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

int main() {
    size_t M = 100;  // rows
    size_t N = 300;  // cols
    float mean = 125.0;
    float stddev = 125.0 / 3.0;  // 99.7% of values will be inside [0, +250] interval
    std::vector<cv::Point3f> pts;
    pts.reserve(M * N);
    for (auto i = 0; i < M; ++i)
        for (auto j = 0; j < N; ++j)
            pts.emplace_back(static_cast<float>(i), static_cast<float>(j), normal_gen<float>(mean, stddev));

    //auto cnt = std::count_if(pts.begin(), pts.end(), [](cv::Point3f const& p) { return p.z >= 0 && p.z <= 250; });
    //std::cout << double(cnt) / double(pts.size()) * 100.0 << std::endl;  // 99.7%

    M_MATH::Mr<float> mr;
    mr.Init(pts.begin(), pts.end(), [](cv::Point3f const& it) { return it.z; });
    mr.CalculateParams();

    std::cout << mr << std::endl;

    return 0;
}