//
// Created by Harold on 2020/10/2.
//

#include <iostream>
#include "m_correlation.h"
#include "m_opencv_utils.h"

using namespace M_MATH;

int main() {
    std::vector<float> A = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    auto I = ToMat(3, 3, A.data());
    std::cout << I << std::endl;
    auto res = Correlation(I, I);
    std::cout << res << std::endl;

    //std::vector<float> B = {1, 2, 3, 4, 5, 6};

    return 0;
}
