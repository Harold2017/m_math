//
// Created by Harold on 2021/4/16.
//

#include <iostream>
#include "m_p1d_eigen.h"

int main() {
    auto _seq = Eigen::VectorXf::Random(20).eval();
    std::cout << _seq << std::endl;
    std::vector<float> seq(_seq.data(), _seq.data() + _seq.size());

    auto peaks = M_MATH::get_persistent_homology(seq);
    for (auto const peak : peaks)
        std::cout << peak << '\n';
    std::cout << std::endl;

    return 0;
}