//
// Created by Harold on 2020/10/15.
//

#include <iostream>
#include <random>
#include <vector>
#include <opencv2/highgui.hpp>
#include "m_filters_opencv.h"
#include "m_opencv_utils.h"

using namespace M_MATH;

int main() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(-0.5,0.5);
    auto gen = [&]() -> float {
        return dist(rng);
    };

    // 1D
    size_t const N = 100;
    size_t const window_size = 11;
    size_t const res_size = N;
    std::vector<float> x;
    std::vector<float> n_x;
    x.reserve(N);
    n_x.resize(res_size);
    for (auto i = 0; i < N; ++i)
        x.emplace_back(gen());

    std::cout << "original: ";
    for (auto e : x)
        std::cout << e << ' ';
    std::cout << std::endl;

    auto I = ToMat(1, N, x.data());
    cv::Mat I_filtered;
    Gaussian1D(I, I_filtered, window_size);
    cv::imshow("original", I);
    cv::imshow("filtered", I_filtered);
    cv::waitKey();

    ToVec(I_filtered, n_x);

    std::cout << "filtered: ";
    for (auto e : n_x)
        std::cout << e << ' ';
    std::cout << std::endl;

    // 2D
    auto I1 = ToMat(10, 10, x.data());
    cv::Mat I_filtered1;
    Gaussian2D(I1, I_filtered1, 3, 3, 0.5, 0.5);
    cv::imshow("original", I1);
    cv::imshow("filtered", I_filtered1);
    cv::waitKey();

    ToVec(I_filtered1, n_x);

    std::cout << "filtered: ";
    for (auto e : n_x)
        std::cout << e << ' ';
    std::cout << std::endl;

    cv::imshow("original", I1);
    // high pass
    cv::Mat out;
    HighPass2D(I1, out, 3);
    //std::cout << out << std::endl;
    cv::imshow("high pass", out);

    // low pass
    cv::Mat out1;
    LowPass2D(I1, out1, 3);
    //std::cout << out1 << std::endl;
    cv::imshow("low pass", out1);

    // band pass
    cv::Mat out2;
    BandPass2D(I1, out2, 3, 5);
    //std::cout << out2 << std::endl;
    cv::imshow("band pass", out1);

    // band reject
    cv::Mat out3;
    BandReject2D(I1, out3, 3, 5);
    //std::cout << out3 << std::endl;
    cv::imshow("band reject", out1);

    cv::waitKey();

    return 0;
}
