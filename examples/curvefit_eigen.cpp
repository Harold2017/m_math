//
// Created by Harold on 2021/4/8.
//

#include <iostream>
#include "m_curvefit_eigen.h"
#include "m_curvefit_opencv.h"
#include "utils.h"

int main() {
    auto x = linspace(0.1, 2.0, 10);
    auto y = linspace(1.0, 20.0, 10);

    auto cf = M_MATH::CurveFitCV();
    auto cf1 = M_MATH::CurveFitEigen();
    cf.polyfit(x.data(), y.data(), x.size(), 1);
    cf1.polyfit(x.data(), y.data(), x.size(), 1);
    std::cout << "slope: " << cf.get_slope() << ", intercept: " << cf.get_intercept() << std::endl;
    std::cout << "eigen slope: " << cf1.get_slope() << ", intercept: " << cf1.get_intercept() << std::endl;
    std::cout << "R2: " << cf.get_RSquare(x.data(), y.data(), x.size()) << std::endl;
    std::cout << "eigen R2: " << cf1.get_RSquare(x.data(), y.data(), x.size()) << std::endl;

    std::vector<double> y1(x.size());
    for (auto i = 0; i < y1.size(); ++i)
        y1[i] = std::exp(x[i]);

    // fit with poly_n = 1
    cf.polyfit(x.data(), y1.data(), x.size(), 1);
    cf1.polyfit(x.data(), y1.data(), x.size(), 1);
    std::cout << "y1 at x = " << x[5] << ": " << cf.getY(x[5]) <<
        ", diff: " << y1[5] - cf.getY(x[5]) << std::endl;
    std::cout << "R2: " << cf.get_RSquare(x.data(), y1.data(), x.size()) << std::endl;
    std::cout << "eigen y1 at x = " << x[5] << ": " << cf1.getY(x[5]) <<
        ", diff: " << y1[5] - cf1.getY(x[5]) << std::endl;
    std::cout << "eigen R2: " << cf1.get_RSquare(x.data(), y1.data(), x.size()) << std::endl;

    // fit with poly_n = 2
    cf.polyfit(x.data(), y1.data(), x.size(), 2);
    cf1.polyfit(x.data(), y1.data(), x.size(), 2);
    std::cout << "y1 at x = " << x[5] << ": " << cf.getY(x[5]) <<
        ", diff: " << y1[5] - cf.getY(x[5]) << std::endl;
    std::cout << "R2: " << cf.get_RSquare(x.data(), y1.data(), x.size()) << std::endl;
    std::cout << "eigen y1 at x = " << x[5] << ": " << cf1.getY(x[5]) <<
        ", diff: " << y1[5] - cf1.getY(x[5]) << std::endl;
    std::cout << "eigen R2: " << cf1.get_RSquare(x.data(), y1.data(), x.size()) << std::endl;

    // fit with poly_n = 5
    cf.polyfit(x.data(), y1.data(), x.size(), 5);
    cf1.polyfit(x.data(), y1.data(), x.size(), 5);
    std::cout << "y1 at x = " << x[5] << ": " << cf.getY(x[5]) <<
        ", diff: " << y1[5] - cf.getY(x[5]) << std::endl;
    std::cout << "R2: " << cf.get_RSquare(x.data(), y1.data(), x.size()) << std::endl;
    std::cout << "eigen y1 at x = " << x[5] << ": " << cf1.getY(x[5]) <<
        ", diff: " << y1[5] - cf1.getY(x[5]) << std::endl;
    std::cout << "eigen R2: " << cf1.get_RSquare(x.data(), y1.data(), x.size()) << std::endl;

    std::vector<double> y1f(x.size());
    cf.getYs(x.begin(), x.end(), y1f.begin());
    for (auto i = 0; i < x.size(); i++)
        if ((y1f[i] - y1[i]) > 1e-3)
            std::cout << "failed at: " << i << ", y1f = " << y1f[i] << ", y1 = " << y1[i] << std::endl;
    cf1.getYs(x.begin(), x.end(), y1f.begin());
    for (auto i = 0; i < x.size(); i++)
        if ((y1f[i] - y1[i]) > 1e-3)
            std::cout << "failed at: " << i << ", y1f = " << y1f[i] << ", y1 = " << y1[i] << std::endl;

    return 0;
}