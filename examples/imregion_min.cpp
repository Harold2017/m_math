//
// Created by Harold on 2021/7/6.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

#include "m_imregion_min.h"

int main(int argc, char* argv[]) {
    cv::Mat I = imread(cv::samples::findFile(argv[1]), cv::IMREAD_GRAYSCALE);
    if(I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }

    cv::Mat src_32FC1;
    I.convertTo(src_32FC1, CV_32FC1);

    cv::Mat dst;
    M_MATH::imregionalmin(src_32FC1, dst);
    cv::Mat mins = dst == 0;
    
    imshow("mins", mins);
    cv::waitKey();

    return 0;
}