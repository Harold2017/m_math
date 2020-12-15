//
// Created by Harold on 2020/12/15.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#define DEBUG_PLOT

#include "m_filters_opencv_1.h"

int main() {
    cv::Mat I = imread( cv::samples::findFile( "lena.jpg" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }
    cv::imshow("original", I);

    cv::Mat out;
    M_MATH::Filter(I, out, 10, 100, M_MATH::FilterType::GBR);
    out.convertTo(out, CV_8UC1, 1.0, 0.0);
    cv::imshow("filtered", out);

    cv::waitKey();

    return 0;
}
