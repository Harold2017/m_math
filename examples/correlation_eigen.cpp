//
// Created by Harold on 2021/4/12.
//

#include <iostream>
#include "m_correlation.h"
#include "m_correlation_eigen.hpp"
#include <opencv2/core/eigen.hpp>

int main() {
    Eigen::MatrixXf in(3, 3), in1(3, 3);
    in << 1, 2, 3,
          4, 5, 6,
          7, 8, 9;
    in1 << 3, 2, 1,
           6, 5, 4,
           9, 8, 7;
    auto cvIn = cv::Mat(in.rows(), in.cols(), CV_32FC1);
    auto cvIn1 = cv::Mat(in1.rows(), in1.cols(), CV_32FC1);
    cv::eigen2cv(in, cvIn);
    cv::eigen2cv(in1, cvIn1);

    auto corr = M_MATH::XCorrelation(cvIn, cvIn1);
    std::cout << corr << std::endl;

    auto corr1 = M_MATH::XCorrelationFFT(in, in1);
    std::cout << corr1 << std::endl;

    auto corr2 = M_MATH::XCorrelation(in, in1);
    std::cout << corr2 << std::endl;

    corr = M_MATH::XCorrelation(cvIn, cvIn);
    std::cout << corr << std::endl;

    corr1 = M_MATH::XCorrelationFFT(in, in);
    std::cout << corr1 << std::endl;

    corr2 = M_MATH::XCorrelation(in, in);
    std::cout << corr2 << std::endl;

    return 0;
}
