//
// Created by Harold on 2021/4/9.
//

#include <iostream>
#include "m_plane_fit_eigen.h"
#include "m_plane_fit.hpp"

int main() {
    srand((unsigned int)time(0));
    std::vector<cv::Point3d> pts;
    std::vector<Eigen::Vector3d> pts1 = { Eigen::Vector3d::Random(), Eigen::Vector3d::Random(), Eigen::Vector3d::Random() };
    for (auto const& pt : pts1)
        pts.push_back(cv::Point3d(pt.x(), pt.y(), pt.z()));
    auto res = M_MATH::PlaneFit(pts);
    auto res1 = M_MATH::PlaneFit(pts1);
    auto res2 = M_MATH::PlaneFitSVD(pts1);

    std::cout << res.first << '\n' << res.second << '\n' << std::endl;
    std::cout << res1.first << '\n' << res1.second << '\n' << std::endl;
    std::cout << res2.first << '\n' << res2.second << '\n' << std::endl;

    return 0;
}