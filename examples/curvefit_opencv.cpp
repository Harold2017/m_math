//
// Created by Harold on 2020/10/16.
//

#include <iostream>
#include "m_curvefit_opencv.h"
#include "utils.h"

int main() {
    auto x = linspace(0.1, 2.0, 10);
    auto y = linspace(1.0, 20.0, 10);

    auto cf = M_MATH::CurveFitCV();
    cf.polyfit(x.data(), y.data(), x.size(), 1);
    std::cout << "slope: " << cf.get_slope() << ", intercept: " << cf.get_intercept() << std::endl;

    std::vector<double> y1(x.size());
    for (auto i = 0; i < y1.size(); ++i)
        y1[i] = std::exp(x[i]);

    cf.polyfit(x.data(), y1.data(), x.size(), 5);
    std::cout << "y1 at x = " << x[5] << ": " << cf.getY(x[5]) <<
              ", diff: " << y1[5] - cf.getY(x[5]) << std::endl;

    std::vector<double> y1f(x.size());
    cf.getYs(x.begin(), x.end(), y1f.begin());
    for (auto i = 0; i < x.size(); i++)
        if ((y1f[i] - y1[i]) > 1e-3)
            std::cout << "failed at: " << i << ", y1f = " << y1f[i] << ", y1 = " << y1[i] << std::endl;

    return 0;
}
