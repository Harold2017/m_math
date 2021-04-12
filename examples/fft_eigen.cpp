//
// Created by Harold on 2021/4/11.
//

#include <iostream>
#include "m_fft_eigen.hpp"

int main() {
    Eigen::Matrix3f in;
    in << 1, 2, 3,
          4, 5, 6,
          7, 8, 9;
    std::cout << in << std::endl;

    auto out = M_MATH::ForwardFFT(in);
    std::cout << out << std::endl;

    auto inv = M_MATH::BackwardFFT(out);
    std::cout << inv << std::endl;

    auto shifted = M_MATH::fftshift(out);
    std::cout << shifted << std::endl;

    auto ishifted = M_MATH::ifftshift(shifted);
    std::cout << ishifted << std::endl;

    return 0;
}
