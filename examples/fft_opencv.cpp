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

Mat GetMag(Mat const& I) {
    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize( I.rows );
    int n = getOptimalDFTSize( I.cols ); // on the border add zero values
    copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
    dft(complexI, complexI);            // this way the result may fit in the source matrix
    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];
    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);
    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));
    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;
    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right
    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);
    normalize(magI, magI, 0, 1, NORM_MINMAX); // Transform the matrix with float values into a

    // viewable image form (float between values 0 and 1).
    imshow("Input Image"       , I   );    // Show the result
    imshow("spectrum magnitude", magI);
    waitKey();
    return magI;
}

int main()
{
    //*
    Mat I = imread( samples::findFile( "lena.jpg" ), IMREAD_GRAYSCALE);
    if( I.empty()){
        cout << "Error opening image" << endl;
        return EXIT_FAILURE;
    }
    //GetMag(I);
     //*/

    // 2d matrix as pts
    size_t M = 10, N = 10;
    std::vector<std::vector<double>> pts(N, std::vector<double>(M, 0));
    //for (auto i = 0; i < M; ++i) {pts[1][i] = 1; pts[3][i] = 1; pts[5][i] = 1; pts[7][i] = 1; pts[9][i] = 1;}
    //for (auto i = 0; i < M; ++i) {pts[i][1] = 1; pts[i][3] = 1; pts[i][5] = 1; pts[i][7] = 1; pts[i][9] = 1;}
    for (auto i = 0; i < M; ++i) pts[i][i] = 1;
    //for (auto i = 0; i < M; ++i)
    //    for (auto j = 0; j < N; ++j)
    //        pts[i][j] = 1;
    //for (auto i = 0; i < M; ++i)
    //    for (auto j = 0; j < N; ++j)
    //        pts[i][j] = normal_gen<double>();

    std::vector<double> pts1(M*N);
    for (auto i = 0; i < M; ++i)
        for (auto j = 0; j < N; ++j)
            pts1[j * M + i] = pts[i][j];

    //Mat I = ToMat(M, N, pts1.data());
    //GetMag(I);

    Mat mag, phase;
    M_MATH::Spectrum_MagInLog_Phase(I, mag, phase);

    imshow("Input Image", I);
    imshow("spectrum magnitude", mag);
    imshow("phase", phase);
    waitKey();

    return 0;
}