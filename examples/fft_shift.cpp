//
// Created by Harold on 2020/10/2.
//

#include <iostream>
#include "m_fft_shift.h"
#include "m_fft.h"

using M_MATH::FFT_SHIFT;
using M_MATH::FFT2D;

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

int main() {
    std::vector<std::complex<double>> vc{0, 1, 2, 3, 4, 5, 6};
    FFT_SHIFT::fft_shift_1d(vc.data(), vc.size());
    print_v(vc);
    FFT_SHIFT::ifft_shift_1d(vc.data(), vc.size());
    print_v(vc);

    std::vector<std::complex<double>> vc2{1, 2, 3, 4, 5, 6, 7, 8, 9};
    FFT_SHIFT::fft_shift_2d(vc2.data(), 3, 3);
    print_v(vc2);
    FFT_SHIFT::ifft_shift_2d(vc2.data(), 3, 3);
    print_v(vc2);
    FFT_SHIFT::ifft_shift_2d(vc2.data(), 3, 3);
    print_v(vc2);

    std::vector<std::complex<double>> vo(14);
    FFT_SHIFT::half2full_1d(vc.data(), vo.data(), 14);
    print_v(vo);

    std::vector<std::complex<double>> vc1{0, 1, 2, 3, 4};
    std::vector<std::complex<double>> vo2(9);
    FFT_SHIFT::half2full_1d(vc1.data(), vo2.data(), 9);
    print_v(vo2);

    std::vector<double> A{1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<std::complex<double>> cpx(6);
    FFT2D::rfft(A.data(), cpx.data(), 3, 3);
    print_v(cpx);
    // FIXME: incorrect here, need full spectrum instead of fftw's half spectrum
    FFT_SHIFT::fft_shift_2d(cpx.data(), 3, 2);
    print_v(cpx);

    return 0;
}
