//
// Created by Harold on 2020/12/30.
//

#include "m_interp_hm.h"
#include <opencv2/core.hpp>

int main() {
    std::vector<cv::Point_<float>> pts = {
            {0.5, 10.0},
            {1.5, 20.0},
            {3.5, 28.0},
            {7.5, 32.0},
    };

    assert(10.0 == M_MATH::Interpolation::Interpolate(pts.begin(), pts.end(), 0.2));
    assert(32.0 == M_MATH::Interpolation::Interpolate(pts.begin(), pts.end(), 8.0));
    assert(15.0 == M_MATH::Interpolation::Interpolate(pts.begin(), pts.end(), 1.0));
    assert(30.0 == M_MATH::Interpolation::Interpolate(pts.begin(), pts.end(), 5.5));
    assert(24.0 == M_MATH::Interpolation::Interpolate(pts.begin(), pts.end(), 2.5));

    return 0;
}
