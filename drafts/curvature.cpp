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
    M_MATH::p2d::GetLocalMaxima(I, peaks, 3, 0);
    cv::imshow("peaks", peaks);
    // set all non-zero pixel to 1
    cv::threshold(peaks, peaks, 0, 255, cv::THRESH_BINARY);
    //cv::imshow("peaks_1", res);

    // use laplacian to calculate mean principal curvature
    cv::Mat dst;
    cv::Laplacian(I, dst, I.type(), 1, 1, 0, cv::BORDER_DEFAULT);
    cv::imshow("laplacian", dst);

    // select only peaks
    cv::bitwise_and(dst, peaks, dst);
    cv::imshow("selected", dst);

    cv::waitKey();

    // peak mean principal curvature
    std::vector<cv::Point> peak_pts;
    cv::findNonZero(peaks, peak_pts);
    auto mpc = cv::sum(dst)[0] / double(2 * peak_pts.size());
    std::cout << mpc << std::endl;

    return 0;
}
