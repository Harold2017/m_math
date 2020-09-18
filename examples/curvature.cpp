//
// Created by Harold on 2020/9/18.
//

#include <iostream>
#include "m_curvature.h"

using namespace M_MATH;

int main() {
    auto a = Point2D<float>(0, 0);
    auto b = Point2D<float>(1, 0);
    auto c = Point2D<float>(0.5, 0.5);

    auto mc = menger_curvature(a, b, c);
    std::cout << "menger curvature: " << mc << std::endl;

    double x[3] = {0, 0.5, 1};
    double y[3] = {0, 0.5, 0};

    auto cc = curvature(x, y, 3);
    std::cout << "curvature: " << cc << std::endl;

    return 0;
}
