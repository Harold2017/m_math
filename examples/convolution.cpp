//
// Created by Harold on 2021/5/13.
//

#include "m_convolution.h"

#include <iostream>

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

int main() {
    std::vector<float> f = { 1, 2, 3, 4, 5, 6 };
    std::vector<float> g = { 7, 8, 9, 10 };

    auto full = M_MATH::Convolution1DFull(f, g);
    auto same = M_MATH::Convolution1DSame(f, g);
    auto valid = M_MATH::Convolution1DValid(f, g);

    print_v(full);
    print_v(same);
    print_v(valid);

    auto full_fft = M_MATH::Convolution1DFullFFT(f, g);
    print_v(full_fft);

    return 0;
}