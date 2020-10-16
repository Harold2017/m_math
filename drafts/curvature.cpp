//
// Created by Harold on 2020/10/16.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "m_p2d_opencv.h"

int main() {
    cv::Mat I = imread( cv::samples::findFile( "lena.jpg" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }

    // find peaks
    cv::Mat peaks;
    M_MATH::p2d::GetLocalMaxima(I, peaks, 3);
    cv::imshow("peaks", peaks);
    // set all non-zero pixel to 1
    cv::threshold(peaks, peaks, 0, 255, cv::THRESH_BINARY);
    //cv::imshow("peaks_1", res);

    // use laplacian to calculate mean principal curvature
    cv::Mat dst, abs_dst;
    cv::Laplacian(I, dst, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
    // converting back to CV_8U, notice abs here
    cv::convertScaleAbs(dst, abs_dst);
    cv::imshow("laplacian", abs_dst);

    // select only peaks
    cv::bitwise_and(abs_dst, peaks, abs_dst);
    cv::imshow("selected", abs_dst);

    cv::waitKey();

    // abs of peak mean principal curvature
    auto mpc = cv::sum(abs_dst)[0] / 2;
    std::cout << mpc << std::endl;

    return 0;
}
