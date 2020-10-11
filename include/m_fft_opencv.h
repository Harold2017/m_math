//
// Created by Harold on 2020/10/10.
//

#ifndef M_MATH_M_FFT_OPENCV_H
#define M_MATH_M_FFT_OPENCV_H

#include <opencv2/core.hpp>

namespace M_MATH {
    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    // same with fftshift in matlab
    void Rearrange(cv::Mat &src, cv::Mat &dst)
    {
        int cx = src.cols / 2;
        int cy = src.rows / 2;
        cv::Mat tmp;
        tmp.create(src.size(), src.type());
        src(cv::Rect(0, 0, cx, cy)).copyTo(tmp(cv::Rect(cx, cy, cx, cy)));
        src(cv::Rect(cx, cy, cx, cy)).copyTo(tmp(cv::Rect(0, 0, cx, cy)));
        src(cv::Rect(cx, 0, cx, cy)).copyTo(tmp(cv::Rect(0, cy, cx, cy)));
        src(cv::Rect(0, cy, cx, cy)).copyTo(tmp(cv::Rect(cx, 0, cx, cy)));
        dst = tmp;
    }

    // Forward FFT of src
    void ForwardFFT(cv::Mat const& Src, cv::Mat *FImg)
    {
        int M = cv::getOptimalDFTSize( Src.rows );
        int N = cv::getOptimalDFTSize( Src.cols );
        cv::Mat padded;
        copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
        cv::Mat complexImg;
        merge(planes, 2, complexImg);
        dft(complexImg, complexImg);
        split(complexImg, planes);

        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

        Rearrange(planes[0], planes[0]);
        Rearrange(planes[1], planes[1]);

        planes[0]/=float(M*N);
        planes[1]/=float(M*N);
        FImg[0]=planes[0].clone();
        FImg[1]=planes[1].clone();
    }

    void Spectrum_Mag_Phase(cv::Mat const& src, cv::Mat &Mag, cv::Mat &Phase)
    {
        cv::Mat planes[2];
        ForwardFFT(src, planes);
        Mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        Phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], Mag, Phase);
    }

    void Spectrum_MagInLog_Phase(cv::Mat &src, cv::Mat &MagInLog, cv::Mat &Phase)
    {
        cv::Mat planes[2];
        ForwardFFT(src,planes);
        MagInLog = cv::Mat::zeros(planes[0].rows, planes[0].cols, CV_32F);
        MagInLog += cv::Scalar::all(1);
        cv::log(MagInLog, MagInLog);
        Phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], MagInLog, Phase);
    }
}

#endif //M_MATH_M_FFT_OPENCV_H
