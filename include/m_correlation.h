//
// Created by Harold on 2020/10/19.
//

#ifndef M_MATH_M_CORRELATION_H
#define M_MATH_M_CORRELATION_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "m_fft_opencv.h"

namespace M_MATH {
    cv::Mat Correlation(cv::Mat const& I, cv::Mat const& I1) {
        int width = cv::getOptimalDFTSize(std::max(I.cols,I1.cols));
        int height = cv::getOptimalDFTSize(std::max(I.rows,I1.rows));
        cv::Mat fft1;
        cv::Mat fft2;

        cv::copyMakeBorder(I, fft1, 0, height - I.rows, 0, width - I.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::copyMakeBorder(I1, fft2, 0, height - I.rows, 0, width - I.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

        fft1.convertTo(fft1, CV_32F);
        fft2.convertTo(fft2, CV_32F);

        cv::dft(fft1,fft1,0,I.rows);
        cv::dft(fft2,fft2,0,I1.rows);

        cv::mulSpectrums(fft1,fft2,fft1,0,true);
        fft1 = fft1/cv::abs(fft1);
        cv::idft(fft1,fft1);
        //cv::idft(fft1,fft1,cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
        Rearrange(fft1, fft1);
        return fft1;
    }

    cv::Mat translateImg(cv::Mat &img, int offsetx, int offsety) {
        cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
        cv::warpAffine(img, img, trans_mat, img.size());
        return img;
    }

    enum class Direction {
        ShiftUp=1, ShiftRight, ShiftDown, ShiftLeft
    };

    cv::Mat shiftImg(cv::Mat const& img, int pixels, Direction direction) {
        cv::Mat temp = cv::Mat::zeros(img.size(), img.type());

        switch (direction) {
            case(Direction::ShiftUp) :
                img(cv::Rect(0, pixels, img.cols, img.rows - pixels)).copyTo(temp(cv::Rect(0, 0, temp.cols, temp.rows - pixels)));
                break;
            case(Direction::ShiftRight) :
                img(cv::Rect(0, 0, img.cols - pixels, img.rows)).copyTo(temp(cv::Rect(pixels, 0, img.cols - pixels, img.rows)));
                break;
            case(Direction::ShiftDown) :
                img(cv::Rect(0, 0, img.cols, img.rows - pixels)).copyTo(temp(cv::Rect(0, pixels, img.cols, img.rows - pixels)));
                break;
            case(Direction::ShiftLeft) :
                img(cv::Rect(pixels, 0, img.cols - pixels, img.rows)).copyTo(temp(cv::Rect(0, 0, img.cols - pixels, img.rows)));
                break;
            default:
                img.copyTo(temp);
        }

        return temp;
    }
}

#endif //M_MATH_M_CORRELATION_H
