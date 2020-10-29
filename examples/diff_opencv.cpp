//
// Created by Harold on 2020/10/29.
//

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "m_diff_opencv.h"

using namespace M_MATH;

int main() {
    cv::Mat I = imread( cv::samples::findFile( "lena.jpg" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }

    cv::Mat dx;
    diff_x(I, dx);
    cv::convertScaleAbs(dx, dx);
    cv::imshow("dx", dx);

    cv::Mat dy;
    diff_y(I, dy);
    cv::convertScaleAbs(dy, dy);
    cv::imshow("dy", dy);

    cv::Mat gp;
    M_MATH::gradient_phase(I, gp, true);
    // std::cout << gp << std::endl;
    cv::convertScaleAbs(gp, gp);
    cv::imshow("gp", gp);

    cv::waitKey();

    return 0;
}
