//
// Created by Harold on 2020/10/15.
//

#ifndef M_MATH_M_FILTERS_OPENCV_H
#define M_MATH_M_FILTERS_OPENCV_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace M_MATH {
    // in, out has the same size
    void Gaussian1D(cv::Mat const& in, cv::Mat & out, int window_len) {
        cv::blur(in, out, cv::Size(1, window_len), cv::Point(-1,-1), cv::BORDER_CONSTANT);
    }

    void Gaussian2D(cv::Mat const& in, cv::Mat & out, int window_len_x, int window_len_y, double sigma_x, double sigma_y) {
        cv::GaussianBlur(in, out, cv::Size(window_len_x, window_len_y), sigma_x, sigma_y, cv::BORDER_CONSTANT);
    }
}

#endif //M_MATH_M_FILTERS_OPENCV_H
