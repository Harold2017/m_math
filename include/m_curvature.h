//
// Created by Harold on 2020/9/18.
//

#ifndef M_MATH_M_CURVATURE_H
#define M_MATH_M_CURVATURE_H

#include <cmath>
#include "point.h"
#include "m_curvefit.h"

namespace M_MATH {
    // triple points menger curvature = 4*triangleArea/(sideLength1*sideLength2*sideLength3)
    template<typename T>
    double menger_curvature(Point2D<T> const& pt1, Point2D<T> const& pt2, Point2D<T> const& pt3) {
        return 4 * (((pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt2.y - pt1.y) * (pt3.x - pt1.x)) / 2) /
               (std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2)) *
                std::sqrt(std::pow(pt3.x - pt2.x, 2) + std::pow(pt3.y - pt2.y, 2)) *
                std::sqrt(std::pow(pt3.x - pt1.x, 2) + std::pow(pt3.y - pt1.y, 2)));
    }

    // curvature, input should be a peak or valley, result is signed
    double curvature(double const* x, double const* y, size_t length) {
        // use quadratic curve fitting, y = ax2 + bx + c
        M_MATH::CurveFit cf;
        cf.polyfit(x, y, length, 2);
        // k(x) = 2a / pow(1 + pow(2ax + b, 2), 3/2), max or min is at x = -b/2a
        return 2.0 * cf.get_coefficient(2);
    }
}

#endif //M_MATH_M_CURVATURE_H
