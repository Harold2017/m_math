//
// Created by Harold on 2021/6/28.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "m_area_auto_covariance.h"
#include "m_opencv_utils.h"

using namespace M_MATH;

int main() {
    std::vector<float> A = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    auto I = ToMat(3, 3, A.data());
    std::cout << I << std::endl;
    auto res = AreaAutoCovariance<float>(I);
    std::cout << res << std::endl;

    I = imread( cv::samples::findFile( "lena.jpg" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }
    //cv::imshow("original", I);

    I.convertTo(I, CV_32F);

    auto AACV = AreaAutoCovariance<float>(I);
    cv::normalize(AACV, AACV, 1, 0, cv::NORM_MINMAX);
    cv::threshold(AACV, AACV, 0.2, 1, cv::THRESH_BINARY);
    auto AACV_8U = M_MATH::To8U(AACV);
    cv::imshow("aacv", AACV_8U);
    cv::waitKey();

    return 0;
}