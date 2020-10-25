//
// Created by Harold on 2020/10/10.
//

#include "m_fft_opencv.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <utility>
#include "m_opencv_utils.h"

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

int main()
{
    /*
    cv::Mat I = imread( cv::samples::findFile( "lena.jpg" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }
    */


    // 2d matrix as pts
    size_t M_ = 10, N_ = 10;
    std::vector<std::vector<float>> pts(N_, std::vector<float>(M_, 0));
    //for (auto i = 0; i < M; ++i) {pts[1][i] = 1; pts[3][i] = 1; pts[5][i] = 1; pts[7][i] = 1; pts[9][i] = 1;}
    //for (auto i = 0; i < M; ++i) {pts[i][1] = 1; pts[i][3] = 1; pts[i][5] = 1; pts[i][7] = 1; pts[i][9] = 1;}
    for (auto i = 0; i < M_; ++i) pts[i][i] = 1;
    //for (auto i = 0; i < M; ++i)
    //    for (auto j = 0; j < N; ++j)
    //        pts[i][j] = 1;
    //for (auto i = 0; i < M; ++i)
    //    for (auto j = 0; j < N; ++j)
    //        pts[i][j] = normal_gen<float>();

    std::vector<float> pts1(M_*N_);
    for (auto i = 0; i < M_; ++i)
        for (auto j = 0; j < N_; ++j)
            pts1[j * M_ + i] = pts[i][j];

    cv::Mat I = M_MATH::ToMat(M_, N_, pts1.data());


    cv::Mat mag, phase;
    //M_MATH::Spectrum_Mag_Phase(I, mag, phase);
    //M_MATH::Spectrum_MagInLog_Phase(I, mag, phase);
    //M_MATH::Spectrum_Mag(I, mag);
    M_MATH::Spectrum_MagInLog(I, mag);

    //cv::imshow("Input Image", I);
    //cv::imshow("spectrum magnitude", h_mag);
    //cv::imshow("phase", phase);
    //cv::waitKey();

    auto m = mag.rows;
    auto n = mag.cols;
    std::cout << m << ", " << n << std::endl;
    auto N = m < n ? m : n;
    std::cout << N << std::endl;

    // only need the upper half
    // shallow copy, for deep copy, use `.copyTo()` or `.clone()`
    auto roi = cv::Rect(0, 0, m, n/2 + 1);
    auto h_mag = mag(roi);

    std::vector<float> Psum(181);
    // theta: [0:1:180]
    double theta;
    cv::Mat mask = cv::Mat::zeros(h_mag.rows, h_mag.cols, CV_32F);
    auto center = cv::Point(m/2, n/2);

    for (auto i = 0; i < 181; i++) {
        // theta in radians
        theta = double(i) / 180.0 * M_PI;
        // draw mask
        mask.setTo(cv::Scalar(0));
        cv::line(mask, center,
             cv::Point2f(float(m)/2 + float(N)/2 * std::cos(theta), float(n)/2-float(N)/2 * std::sin(theta)),
             cv::Scalar_<float>(1));
        //cv::imshow("mask", mask);
        //cv::waitKey();

        // apply mask
        cv::bitwise_and(h_mag, mask, mask);
        //cv::imshow("out", mask);
        //cv::waitKey();
        //std::cout << cv::sum(mask)[0] << std::endl;
        Psum[i] = cv::sum(mask)[0];
    }

    print_v(Psum);

    auto max_it = std::max_element(Psum.begin(), Psum.end());
    std::cout << std::distance(Psum.begin(), max_it) << std::endl;

    return 0;
}