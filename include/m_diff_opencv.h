//
// Created by Harold on 2020/10/29.
//

#ifndef M_MATH_M_DIFF_OPENCV_H
#define M_MATH_M_DIFF_OPENCV_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace M_MATH {
    void diff_x(cv::Mat const& in, cv::Mat &out, int order = 1, int ksize = 3) {
        // ksize must be in [1, 3, 5, 7]
        cv::Sobel(in, out, CV_32F, order, 0, ksize);
    }

    void diff_y(cv::Mat const& in, cv::Mat &out, int order = 1, int ksize = 3) {
        // ksize must be in [1, 3, 5, 7]
        cv::Sobel(in, out, CV_32F, 0, order, ksize);
    }

    void gradient_phase(cv::Mat const& in, cv::Mat &out, bool angleInDegrees = false, int order = 1, int ksize = 3) {
        cv::Mat gx, gy;
        cv::Sobel(in, gx, CV_32F, order, 0, ksize);
        cv::Sobel(in, gy, CV_32F, 0, order, ksize);
        cv::phase(gx, gy, out, angleInDegrees);
    }
}

#endif //M_MATH_M_DIFF_OPENCV_H
