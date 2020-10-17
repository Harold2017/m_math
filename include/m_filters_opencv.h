//
// Created by Harold on 2020/10/15.
//

#ifndef M_MATH_M_FILTERS_OPENCV_H
#define M_MATH_M_FILTERS_OPENCV_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "m_fft_opencv.h"

namespace M_MATH {
    // in, out has the same size
    void Gaussian1D(cv::Mat const& in, cv::Mat & out, int window_len) {
        cv::blur(in, out, cv::Size(1, window_len), cv::Point(-1,-1), cv::BORDER_CONSTANT);
    }

    void Gaussian2D(cv::Mat const& in, cv::Mat & out, int window_len_x, int window_len_y, double sigma_x, double sigma_y) {
        cv::GaussianBlur(in, out, cv::Size(window_len_x, window_len_y), sigma_x, sigma_y, cv::BORDER_CONSTANT);
    }

    // High pass
    // for cutoff_feq, 0 is center
    void HighPass(cv::Mat const& in, cv::Mat & out, int cutoff_feq) {
        int M = cv::getOptimalDFTSize( in.rows );
        int N = cv::getOptimalDFTSize( in.cols );
        cv::Mat padded;
        cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
        cv::Mat complexImg;
        cv::merge(planes, 2, complexImg);
        cv::dft(complexImg, complexImg);
        cv::split(complexImg, planes);

        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

        Rearrange(planes[0], planes[0]);
        Rearrange(planes[1], planes[1]);

        cv::Mat mag, phase;
        mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], mag, phase);

        // circle mask
        cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
        auto center = cv::Point(mag.rows/2, mag.cols/2);
        cv::circle(mask, center, cutoff_feq, CV_RGB(255, 255, 255), -1);
        cv::bitwise_not(mask, mask);
        mask.convertTo(mask, CV_32FC1);
        // apply mask
        cv::bitwise_and(mag, mask, mag);
        cv::bitwise_and(phase, mask, phase);

        cv::Mat filtered;
        cv::polarToCart(mag, phase, planes[0], planes[1]);
        cv::merge(planes, 2, filtered);
        Rearrange(filtered, filtered);

        cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);

        // normalize to original scale
        double min, max;
        cv::minMaxLoc(in, &min, &max);
        cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);
    }

    // Low pass
    // for cutoff_feq, 0 is center
    void LowPass(cv::Mat const& in, cv::Mat & out, int cutoff_feq) {
        int M = cv::getOptimalDFTSize( in.rows );
        int N = cv::getOptimalDFTSize( in.cols );
        cv::Mat padded;
        cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
        cv::Mat complexImg;
        cv::merge(planes, 2, complexImg);
        cv::dft(complexImg, complexImg);
        cv::split(complexImg, planes);

        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

        Rearrange(planes[0], planes[0]);
        Rearrange(planes[1], planes[1]);

        cv::Mat mag, phase;
        mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], mag, phase);

        // circle mask
        cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
        auto center = cv::Point(mag.rows/2, mag.cols/2);
        cv::circle(mask, center, cutoff_feq, CV_RGB(255, 255, 255), -1);
        mask.convertTo(mask, CV_32FC1);
        // apply mask
        cv::bitwise_and(mag, mask, mag);
        cv::bitwise_and(phase, mask, phase);

        cv::Mat filtered;
        cv::polarToCart(mag, phase, planes[0], planes[1]);
        cv::merge(planes, 2, filtered);
        Rearrange(filtered, filtered);

        cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);

        // normalize to original scale
        double min, max;
        cv::minMaxLoc(in, &min, &max);
        cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);
    }

    // Band pass
    // for (cutoff_feq_l, cutoff_feq_h), 0 is center
    void BandPass(cv::Mat const& in, cv::Mat & out, int cutoff_feq_l, int cutoff_feq_h) {
        int M = cv::getOptimalDFTSize( in.rows );
        int N = cv::getOptimalDFTSize( in.cols );
        cv::Mat padded;
        cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
        cv::Mat complexImg;
        cv::merge(planes, 2, complexImg);
        cv::dft(complexImg, complexImg);
        cv::split(complexImg, planes);

        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

        Rearrange(planes[0], planes[0]);
        Rearrange(planes[1], planes[1]);

        cv::Mat mag, phase;
        mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], mag, phase);

        // ring mask
        cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
        auto center = cv::Point(mag.rows/2, mag.cols/2);
        cv::circle(mask, center, cutoff_feq_h, CV_RGB(255, 255, 255), -1);
        cv::circle(mask, center, cutoff_feq_l, CV_RGB(0, 0, 0), -1);
        mask.convertTo(mask, CV_32FC1);
        // apply mask
        cv::bitwise_and(mag, mask, mag);
        cv::bitwise_and(phase, mask, phase);

        cv::Mat filtered;
        cv::polarToCart(mag, phase, planes[0], planes[1]);
        cv::merge(planes, 2, filtered);
        Rearrange(filtered, filtered);

        cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);

        // normalize to original scale
        double min, max;
        cv::minMaxLoc(in, &min, &max);
        cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);
    }

    // Band reject
    // for [0, cutoff_feq_l] && [cutoff_feq_h, infinite], 0 is center
    void BandReject(cv::Mat const& in, cv::Mat & out, int cutoff_feq_l, int cutoff_feq_h) {
        int M = cv::getOptimalDFTSize( in.rows );
        int N = cv::getOptimalDFTSize( in.cols );
        cv::Mat padded;
        cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
        cv::Mat complexImg;
        cv::merge(planes, 2, complexImg);
        cv::dft(complexImg, complexImg);
        cv::split(complexImg, planes);

        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

        Rearrange(planes[0], planes[0]);
        Rearrange(planes[1], planes[1]);

        cv::Mat mag, phase;
        mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
        cv::cartToPolar(planes[0], planes[1], mag, phase);

        // ring mask
        cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
        auto center = cv::Point(mag.rows/2, mag.cols/2);
        cv::circle(mask, center, cutoff_feq_h, CV_RGB(255, 255, 255), -1);
        cv::circle(mask, center, cutoff_feq_l, CV_RGB(0, 0, 0), -1);
        cv::bitwise_not(mask, mask);
        mask.convertTo(mask, CV_32FC1);
        // apply mask
        cv::bitwise_and(mag, mask, mag);
        cv::bitwise_and(phase, mask, phase);

        cv::Mat filtered;
        cv::polarToCart(mag, phase, planes[0], planes[1]);
        cv::merge(planes, 2, filtered);
        Rearrange(filtered, filtered);

        cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);

        // normalize to original scale
        double min, max;
        cv::minMaxLoc(in, &min, &max);
        cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);
    }
}

#endif //M_MATH_M_FILTERS_OPENCV_H
