//
// Created by Harold on 2021/7/5.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "m_p2d_opencv.h"
#include "m_curvature_opencv.h"

int main(int argc, char* argv[]) {
    cv::Mat I = imread(cv::samples::findFile(argv[1]), cv::IMREAD_GRAYSCALE);
    if(I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }

    auto peaks_mat = M_MATH::p2d::GetLocalMaxima(I, 5, 0., 0.05);

    //cv::imshow("src img", I);
    //cv::imshow("peaks", peaks_mat);
    //cv::waitKey();

    std::vector<cv::Point> peaks;
    cv::findNonZero(peaks_mat, peaks);
    auto peaks_num = peaks.size();
    double area = I.rows * I.cols;
    std::cout << "peaks density: " << double(peaks_num) / area << std::endl;

    cv::Mat src_32FC1;
    I.convertTo(src_32FC1, CV_32FC1);
    auto spc = M_MATH::ArithmeticMeanSummitCurvature(src_32FC1, peaks);
    std::cout << "arithmetic mean summit curvature: " << spc << std::endl;

    // use Laplacian [0 1 0; 1 -4 1; 0 1 0]
    cv::Mat dst;
    cv::Laplacian(src_32FC1, dst, src_32FC1.type(), 1, 1, 0, cv::BORDER_DEFAULT);
    cv::Mat peaks_mat_32FC1;
    peaks_mat.convertTo(peaks_mat_32FC1, CV_32FC1);
    cv::bitwise_and(dst, peaks_mat_32FC1, dst);
    std::cout << "laplacian: " << double(cv::sum(dst)[0]) / double(2 * peaks_num) << std::endl;

    return 0;
}