//
// Created by Harold on 2020/10/2.
//

#include <iostream>
#include "m_autoc.h"

using namespace M_MATH;

template <typename T, size_t size> void print_a(const T (&array)[size])
{
    for(size_t i = 0; i < size; ++i)
        std::cout << array[i] << " ";
    std::cout << std::endl;
}

void auto_corr_2d(double *pOrigin, double *pAc, int xdim, int ydim) {
    auto fft_size = xdim * (ydim/2+1);
    std::vector<std::complex<double>> vCpx(fft_size);
    FFT2D::rfft(pOrigin, vCpx.data(), xdim, ydim);
    for (auto i = 0; i < fft_size; ++i) {
        std::cout << vCpx[i] << ' ';
        vCpx[i] = std::abs(vCpx[i]) * std::abs(vCpx[i]);
    }
    std::cout << std::endl;
    FFT2D::irfft(pAc, vCpx.data(), xdim, ydim);
    auto N = xdim * ydim;
    for (auto i = 0; i < N; ++i)
        std::cout << pAc[i] << ' ';
    std::cout << std::endl;
    fft_shift_2d(pAc, xdim, ydim);
    for (auto i = 0; i < N; ++i)
        pAc[i] /= N;
}

int main() {
    double A[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    double B[9];
    ::auto_corr_2d(A, B, 3, 3);
    print_a(B);

    double C[6] = {1, 2, 3, 4, 5, 6};
    double D[6];
    // FIXME: different result with MATLAB
    ::auto_corr_2d(C, D, 2, 3);
    print_a(D);

    return 0;
}
