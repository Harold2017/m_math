//
// Created by Harold on 2021/5/28.
//

#include "m_pt_in_polygon.h"

#include <iostream>

using namespace M_MATH;

int main() {
    std::vector<Eigen::Vector2d> polygon{ { 0, 0 }, { 10, 0 }, { 10, 10 }, { 0, 10 } };
    std::cout << IsPtInsidePolygon(Eigen::Vector2d{ 20, 20 }, polygon) << '\n'
              << IsPtInsidePolygon(Eigen::Vector2d{ 5, 5 }, polygon)
              << std::endl;

    std::vector<Eigen::Vector2d> polygon1{ { 0, 0 }, { 10, 0 }, { 10, 10 }, { 0, 10 }, { 0, 0 } };
    std::cout << IsPtInsidePolygon(Eigen::Vector2d{ 20, 20 }, polygon1) << '\n'
              << IsPtInsidePolygon(Eigen::Vector2d{ 5, 5 }, polygon1)
              << std::endl;

    return 0;
}