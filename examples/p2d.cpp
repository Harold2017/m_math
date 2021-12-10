//
// Created by Harold on 2021/12/10.
//

#include <iostream>
#include "m_p2d.h"

int main() {
    cv::theRNG().state = cv::getTickCount();
    cv::Mat mat = cv::Mat::zeros(5, 8, CV_8U);
    cv::randu(mat, -10, 10);
    std::cout << mat << std::endl;

    cv::Mat res = M_MATH::p2d::GetLocalMaxima(mat, 0);
    std::cout << res << std::endl;

    return 0;
}
