//
// Created by Harold on 2020/10/15.
//

#include <iostream>
#include <random>
#include <vector>
#include <opencv2/highgui.hpp>
#include "m_filters_opencv.h"

using namespace M_MATH;

// all use float inside
template<typename T>
inline cv::Mat ToMat(size_t rows, size_t cols, T *data) {
    return cv::Mat(rows, cols, CV_32FC1, data);
}

template<typename T>
void ToVec(cv::Mat const& in, std::vector<T> & out) {
    if (in.isContinuous()) {
        // array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
        out.assign((float*)in.data, (float*)in.data + in.total()*in.channels());
    } else {
        for (int i = 0; i < in.rows; ++i) {
            out.insert(out.end(), in.ptr<float>(i), in.ptr<float>(i)+in.cols*in.channels());
        }
    }
}

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
    std::vector<double> x;
    std::vector<double> n_x;
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

    return 0;
}
