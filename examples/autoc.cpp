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
    auto res = XCorrelation(I, I);
    std::cout << res << std::endl;

    std::vector<float> B = {1, 2, 3, 4, 5, 6};
    auto I1 = ToMat(3, 2, A.data());
    std::cout << I1 << std::endl;
    auto res1 = XCorrelation(I, I);
    std::cout << res1 << std::endl;

    return 0;
}
