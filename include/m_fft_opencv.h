//
// Created by Harold on 2020/10/10.
//

#ifndef M_MATH_M_FFT_OPENCV_H
#define M_MATH_M_FFT_OPENCV_H

#include <opencv2/core.hpp>

namespace M_MATH {
    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    // same with fftshift in matlab
    void Rearrange(cv::Mat& src, cv::Mat& dst)
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
    void ForwardFFT(cv::Mat const& Src, cv::Mat& complexImg)
    {
        int M = cv::getOptimalDFTSize( Src.rows );
        int N = cv::getOptimalDFTSize( Src.cols );
        cv::Mat padded;
        cv::copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
        cv::merge(planes, 2, complexImg);
        cv::dft(complexImg, complexImg);
    }

    void Spectrum_Mag(cv::Mat const& src, cv::Mat& Mag) {
        cv::Mat complexImg;
        ForwardFFT(src, complexImg);

        cv::Mat planes[2];
        cv::split(complexImg, planes);

        cv::magnitude(planes[0], planes[1], planes[0]);
        Mag = planes[0];

        Mag = Mag(cv::Rect(0, 0, Mag.cols & -2, Mag.rows & -2));
        Rearrange(Mag, Mag);

        // scaling
        int M = complexImg.rows;
        int N = complexImg.cols;
        Mag /= float(M*N);
    }

    void Spectrum_MagInLog(cv::Mat const& src, cv::Mat& Mag) {
        cv::Mat complexImg;
        ForwardFFT(src, complexImg);

        cv::Mat planes[2];
        cv::split(complexImg, planes);

        cv::magnitude(planes[0], planes[1], planes[0]);
        Mag = planes[0];


        Mag = Mag(cv::Rect(0, 0, Mag.cols & -2, Mag.rows & -2));
        Rearrange(Mag, Mag);

        // scale in log
        Mag += cv::Scalar::all(1);
        cv::log(Mag, Mag);
        cv::normalize(Mag, Mag, 0, 1, cv::NORM_MINMAX);
    }

    void Spectrum_Mag_Phase(cv::Mat const& src, cv::Mat& Mag, cv::Mat& Phase)
    {
        cv::Mat complexImg;
        ForwardFFT(src, complexImg);

        cv::Mat planes[2];
        cv::split(complexImg, planes);

        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

        Rearrange(planes[0], planes[0]);
        Rearrange(planes[1], planes[1]);

        // scaling
        int M = complexImg.rows;
        int N = complexImg.cols;
        planes[0]/=float(M*N);
        planes[1]/=float(M*N);

        Mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        Phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], Mag, Phase);
    }

    void Spectrum_MagInLog_Phase(cv::Mat& src, cv::Mat& MagInLog, cv::Mat& Phase)
    {
        cv::Mat complexImg;
        ForwardFFT(src, complexImg);

        cv::Mat planes[2];
        cv::split(complexImg, planes);

        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

        Rearrange(planes[0], planes[0]);
        Rearrange(planes[1], planes[1]);

        // scale in log
        MagInLog = cv::Mat::zeros(planes[0].rows, planes[0].cols, CV_32F);
        Phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], MagInLog, Phase);
        MagInLog += cv::Scalar::all(1);
        cv::log(MagInLog, MagInLog);
        cv::normalize(MagInLog, MagInLog, 0, 1, cv::NORM_MINMAX);
    }

    void CalcPSD(const cv::Mat& src, cv::Mat& PSD, bool logflag = false)
    {
        // not use padding here
        // FIXME: padding introduce white lines on ouputImg's x, y axis
        cv::Mat planes[2] = { cv::Mat_<float>(src.clone()), cv::Mat::zeros(src.size(), CV_32F) };
        cv::Mat complexI;
        cv::merge(planes, 2, complexI);
        cv::dft(complexI, complexI);
        cv::split(complexI, planes);

        // remove constant components for better visualization
        planes[0].at<float>(0) = 0;
        planes[1].at<float>(0) = 0;

        // PSD calculation
        cv::Mat imgPSD;
        cv::magnitude(planes[0], planes[1], imgPSD);
        cv::pow(imgPSD, 2, imgPSD);
        PSD = imgPSD;

        // switch PSD to logarithmic scale
        // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
        cv::Mat imglogPSD;
        if (logflag)
        {
            imglogPSD = imgPSD + cv::Scalar::all(1);
            cv::log(imglogPSD, imglogPSD);
            PSD = imglogPSD;
        }
        // crop the spectrum, if it has an odd number of rows or columns
        PSD = PSD(cv::Rect(0, 0, PSD.cols & -2, PSD.rows & -2));
        Rearrange(PSD, PSD);
        cv::normalize(PSD, PSD, 0, 1, cv::NORM_MINMAX);
    }
}

#endif //M_MATH_M_FFT_OPENCV_H
