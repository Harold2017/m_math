//
// Created by Harold on 2020/10/16.
//

#ifndef M_MATH_M_CURVATURE_OPENCV_H
#define M_MATH_M_CURVATURE_OPENCV_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace M_MATH {
    double MeanPrincipalCurvature(cv::Mat const& in, int kSize = 1) {
        // use laplacian to calculate mean principal curvature
        cv::Mat dst;
        cv::Laplacian(in, dst, in.type(), kSize, 1, 0, cv::BORDER_DEFAULT);
        return cv::sum(dst)[0] / double(2 * dst.total());
    }
}

#endif //M_MATH_M_CURVATURE_OPENCV_H
