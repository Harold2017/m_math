//
// Created by Harold on 2021/4/30.
//

#include "m_cov_attr.h"

#include <iostream>

int main() {
    srand((unsigned)time(0));
    std::vector<Eigen::Vector3d> pts(100);
    std::generate(pts.begin(), pts.end(), []() { return Eigen::Vector3d::Random(); });

    M_MATH::CovarianceAttributes ca;
    ca.Compute(pts, 50, 5);
    std::cout << ca << std::endl;

    return 0;
}