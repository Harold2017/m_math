//
// Created by Harold on 2020/10/2.
//

#include <iostream>
#include "m_fft_shift.h"

using namespace M_MATH;

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

int main() {
    std::vector<std::complex<double>> vc{0, 1, 2, 3, 4, 5, 6};
    fft_shift_1d(vc.data(), vc.size());
    print_v(vc);
    ifft_shift_1d(vc.data(), vc.size());
    print_v(vc);

    std::vector<std::complex<double>> vc2{1, 2, 3, 4, 5, 6, 7, 8, 9};
    fft_shift_2d(vc2.data(), 3, 3);
    print_v(vc2);
    ifft_shift_2d(vc2.data(), 3, 3);
    print_v(vc2);
    ifft_shift_2d(vc2.data(), 3, 3);
    print_v(vc2);

    return 0;
}
