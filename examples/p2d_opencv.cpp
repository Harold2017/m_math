//
// Created by Harold on 2020/10/13.
//

#include <iostream>
#include "m_p2d_opencv.h"

int main() {
    cv::Mat temp = cv::Mat::zeros(15, 15, CV_32FC1);
    temp.at<float>(7, 7) = 1;
    temp.at<float>(3, 5) = 6;
    temp.at<float>(8, 10) = 4;
    temp.at<float>(11, 13) = 7;
    temp.at<float>(10, 3) = 18;
    temp.at<float>(7, 13) = 3;

    cv::Mat res;
    M_MATH::p2d::GetLocalMaxima(temp, res, 3);
    std::cout << res << std::endl;

    return 0;
}
