//
// Created by Harold on 2020/10/19.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "m_correlation.h"

int main() {
    cv::Mat I = imread( cv::samples::findFile( "lena.jpg" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }
    cv::imshow("original", I);


    cv::Mat I1;
    I.copyTo(I1);
    M_MATH::translateImg(I1, 30, 20);
    cv::imshow("transferred", I1);

    auto corr = M_MATH::XCorrelation(I, I1);
    cv::normalize(corr, corr, 1, 0, cv::NORM_MINMAX);
    cv::imshow("corr", corr);
    cv::waitKey();

    return 0;
}
