//
// Created by Harold on 2020/9/16.
//

#include <iostream>
#include <vector>
#include <memory>
#include "m_fft.h"

using M_MATH::FFT1D;
using M_MATH::FFT2D;

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

template<typename T>
std::shared_ptr<double> _2dVec2Array(std::vector<std::vector<T>> const& vec) {
    auto arr = std::make_shared<double>(vec.size() * vec[0].size());
    double* end = arr.get();
    for (auto const& row : vec) {
        std::copy(row.begin(), row.end(), end);
        end += row.size();
    }
    return arr;
}

int main() {
    int N = 100;
    std::vector<double> origin(2*N, 0);
    std::vector<double> real(N, 0);
    std::vector<double> img(N, 0);

    real[0] = origin[0] = 1.0;
    for (auto i = 1; i < 10; i++) {
        real[i] = real[N-i] = origin[2*i] = origin[2*(N-i)] = 1.0;
    }

    FFT1D::fft(real.data(), img.data(), N);
    print_v(real);
    print_v(img);

    std::cout << std::endl;

    FFT1D::ifft(real.data(), img.data(), N);
    print_v(real);
    print_v(img);

    std::cout << std::endl;

    FFT1D::rfft(origin.data(), real.data(), img.data(), origin.size());
    print_v(real);
    print_v(img);

    std::cout << std::endl;

    print_v(origin);
    std::vector<double> origin2(origin.size());
    // notice: here accuracy loss
    FFT1D::irfft(origin2.data(), real.data(), img.data(), origin2.size());
    print_v(origin2);

    std::cout << std::endl;

    double ori[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    double ori2[9];
    double re[6];
    double im[6];
    std::complex<double> cpx[6];

    FFT2D::rfft(ori, re, im, 3, 3);
    FFT2D::rfft(ori, cpx, 3, 3);
    for (auto i = 0; i < 6; ++i)
        std::cout << "( " << re[i] << ", " << im[i] << ", " << cpx[i] << " )" << ' ';
    std::cout << std::endl;

    FFT2D::irfft(ori2, re, im, 3, 3);
    for (double i : ori2)
        std::cout << i << ' ';
    std::cout << std::endl;
    FFT2D::irfft(ori2, cpx, 3, 3);
    for (double i : ori2)
        std::cout << i << ' ';
    std::cout << std::endl;

    std::cout << std::endl;

    double ori1[5][2] = {{1, 2}, {3, 4}, {5, 6}, {7, 8}, {9, 10}};
    double ori3[5][2];
    double re1[10];
    double im1[10];

    FFT2D::rfft(reinterpret_cast<double *>(ori1), re1, im1, 5, 2);
    for (auto i = 0; i < 10; ++i)
        std::cout << "( " << re1[i] << ", " << im1[i] << " )" << ' ';

    std::cout << std::endl;

    FFT2D::irfft(reinterpret_cast<double *>(ori3), re1, im1, 5, 2);
    for (auto & i : ori3)
        for (double j : i)
            std::cout << j << ' ';

    std::cout << std::endl;

    std::vector<std::vector<double>> ori5 = {{1, 2}, {3, 4}, {5, 6}, {7, 8}, {9, 10}};
    auto ptr = _2dVec2Array(ori5);
    double re2[10], im2[10];
    FFT2D::rfft(ptr.get(), re2, im2, ori5.size(), ori5[0].size());
    for (auto i = 0; i < 10; ++i)
        std::cout << "( " << re2[i] << ", " << im2[i] << " )" << ' ';

    return 0;
}
