//
// Created by Harold on 2020/10/10.
//

// code from: https://docs.opencv.org/4.4.0/d8/d01/tutorial_discrete_fourier_transform.html

#include "m_fft_opencv.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

template<typename T>
Mat ToMat(size_t xdim, size_t ydim, T* data) {
    return Mat(xdim, ydim, CV_32FC1, data);
}

int main()
{
    //*
    Mat I = imread( samples::findFile( "lena.jpg" ), IMREAD_GRAYSCALE);
    if( I.empty()){
        cout << "Error opening image" << endl;
        return EXIT_FAILURE;
    }
    // */

    /*
    // 2d matrix as pts
    size_t M_ = 10, N_ = 10;
    std::vector<std::vector<double>> pts(N_, std::vector<double>(M_, 0));
    //for (auto i = 0; i < M; ++i) {pts[1][i] = 1; pts[3][i] = 1; pts[5][i] = 1; pts[7][i] = 1; pts[9][i] = 1;}
    //for (auto i = 0; i < M; ++i) {pts[i][1] = 1; pts[i][3] = 1; pts[i][5] = 1; pts[i][7] = 1; pts[i][9] = 1;}
    for (auto i = 0; i < M_; ++i) pts[i][i] = 1;
    //for (auto i = 0; i < M; ++i)
    //    for (auto j = 0; j < N; ++j)
    //        pts[i][j] = 1;
    //for (auto i = 0; i < M; ++i)
    //    for (auto j = 0; j < N; ++j)
    //        pts[i][j] = normal_gen<double>();

    std::vector<double> pts1(M_*N_);
    for (auto i = 0; i < M_; ++i)
        for (auto j = 0; j < N_; ++j)
            pts1[j * M_ + i] = pts[i][j];

    Mat I = ToMat(M_, N_, pts1.data());
     */

    Mat mag, phase;
    //M_MATH::Spectrum_Mag_Phase(I, mag, phase);
    //M_MATH::Spectrum_MagInLog_Phase(I, mag, phase);
    //M_MATH::Spectrum_Mag(I, mag);
    M_MATH::Spectrum_MagInLog(I, mag);

    auto m = mag.rows;
    auto n = mag.cols;
    std::cout << m << ", " << n << std::endl;
    auto N = m < n ? m : n;
    std::cout << N << std::endl;



    imshow("Input Image", I);
    imshow("spectrum magnitude", mag);
    //imshow("phase", phase);
    waitKey();

    return 0;
}