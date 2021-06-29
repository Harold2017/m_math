//
// Created by Harold on 2020/10/22.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "m_correlation.h"
#include "m_opencv_utils.h"

int main() {
    cv::Mat I = imread( cv::samples::findFile( "test.png" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }

    cv::imshow("original", I);

    auto auto_corr = M_MATH::XCorrelation(I, I);
    cv::normalize(auto_corr, auto_corr, 1, 0, cv::NORM_MINMAX);
    cv::imshow("auto_corr", auto_corr);
    //std::cout << auto_corr << std::endl;

    // use threshold s to filter the bright ellipse area
    cv::threshold(auto_corr, auto_corr, 0.5, 1, cv::THRESH_BINARY);
    cv::imshow("corr_thr", auto_corr);

    // find contour of this edge
    auto auto_corr_8U = M_MATH::To8U(auto_corr);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(auto_corr_8U, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::cout << "contours size: " << contours.size() << std::endl;
    cv::Mat drawing = cv::Mat::zeros(auto_corr_8U.size(), CV_8UC1);
    cv::drawContours(drawing, contours, -1, 255, 1);
    cv::imshow("Contours", drawing);

    // no contour
    if (contours.empty()) {
        std::cout << "auto-correlation length: " << 0 << '\n'
                  << "texture aspect ratio: " << 0
                  << std::endl;
        return 0;
    }

    // center is always at image center
    cv::Point center{I.cols/2, I.rows/2};

    // find contour which surrond center
    int target_contour_idx = -1;
    for (auto i = 0; i < contours.size(); i++)
        if (cv::pointPolygonTest(contours[i], center, false) >= 0)
            target_contour_idx = i;
    if (target_contour_idx == -1) {
        std::cout << "auto-correlation length: " << 0 << '\n'
                  << "texture aspect ratio: " << 0
                  << std::endl;
        return 0;
    }

    // find shortest radius and longest radius
    double max = std::numeric_limits<double>::min();
    double min = std::numeric_limits<double>::max();
    cv::Point max_r, min_r;
    for (auto &pt : contours[target_contour_idx]) {
        if (cv::norm(pt - center) > max) {
            max = cv::norm(pt - center);
            max_r = pt;
        }
        if (cv::norm(pt - center) < min) {
            min = cv::norm(pt - center);
            min_r = pt;
        }
    }
    std::cout << "max: " << max << ", at: " << max_r << '\n'
              << "min: " << min << ", at: " << min_r
              << std::endl;

    // draw max, min radius
    drawing = 0;
    cv::drawContours(drawing, contours, target_contour_idx, 255, 1);
    cv::line(drawing, center, max_r, 255);
    cv::line(drawing, center, min_r, 255);
    cv::imshow("max_min", drawing);

    // get auto-correlation length
    auto Sal = min;
    // get texture aspect ratio
    auto Str = min/max;
    std::cout << "auto-correlation length: " << Sal << '\n'
              << "texture aspect ratio: " << Str
              << std::endl;

    cv::waitKey();

    return 0;
}
