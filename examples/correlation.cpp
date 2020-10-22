//
// Created by Harold on 2020/10/19.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "m_correlation.h"
#include "m_opencv_utils.h"

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
    //std::cout << corr << std::endl;

    // use threshold to filter the bright ellipse area
    cv::threshold(corr, corr, 0.5, 1, cv::THRESH_BINARY);
    cv::imshow("corr_thr", corr);
    auto corr_8U = M_MATH::To8U(corr);
    //std::cout << corr_8U << std::endl;
    //cv::imshow("corr_8U", corr_8U);

    // use canny to find ellipse edge
    cv::Mat canny_out;
    cv::Canny(corr_8U, canny_out, 0, 30);
    cv::imshow("canny", canny_out);

    // find contour of this edge
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_out, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    cv::Mat drawing = cv::Mat::zeros(corr_8U.size(), CV_8UC1);
    cv::drawContours(drawing, contours, -1, 255, 1, cv::LINE_8, hierarchy, 0);
    cv::imshow("Contours", drawing);

    // fit ellipse
    auto rect = cv::fitEllipse(contours[contours.size()-1]);
    cv::ellipse(I, rect, cv::Scalar(255,255,0), 2, 8);
    cv::imshow("rect", I);

    cv::waitKey();

    std::cout << "ellipse center: " << rect.center << '\n'
              << "ellipse theta (in radians): " << rect.angle  << '\n'
              << "ellipse semi-axis 1: " << rect.size.width / 2 << '\n'
              << "ellipse semi-axis 2: " << rect.size.height / 2 << '\n'
              << std::endl;

    return 0;
}
