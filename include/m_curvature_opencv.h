//
// Created by Harold on 2020/10/16.
//

#ifndef M_MATH_M_CURVATURE_OPENCV_H
#define M_MATH_M_CURVATURE_OPENCV_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace M_MATH {
    // formula from SURFSTAND report (Blunt and Jiang 2003)
    // least squares polynormial surface: f(x, y) = a00 + a10 * x + a01 * y + a20 * x^2 + a11 * x * y + a02 * y^2
    // dx = dy => dxy
    // spc = - 1/N * sum(a20 + a02)
    // src: CV_32FC1
    double ArithmeticMeanSummitCurvature(cv::Mat const& src, std::vector<cv::Point> const& summits, double dxy = 1.) {
        if (summits.empty()) return 0.;

        double spc = 0.;
        double const alpha = 1. / (1636. * dxy * dxy * double(summits.size()));
        int i, j;
        int imax = src.rows - 2;
        int jmax = src.cols - 2;
        for (auto const& p : summits) {
            i = p.x;
            j = p.y;
            if (i < 2 || j < 2 || i > imax || j > jmax) continue;
            spc += 65 * ( src.at<float>(i - 2, j)     + src.at<float>(i + 2, j)     + src.at<float>(i, j - 2)     + src.at<float>(i, j + 2) )
                  -71 * ( src.at<float>(i - 1, j)     + src.at<float>(i + 1, j)     + src.at<float>(i, j - 1)     + src.at<float>(i, j + 1) )
                  -88 * ( src.at<float>(i, j) )
                  +48 * ( src.at<float>(i - 2, j - 2) + src.at<float>(i - 2, j + 2) + src.at<float>(i + 2, j - 2) + src.at<float>(i + 2, j + 2) );
        }

        spc *= alpha;
        return spc;
    }
}

#endif //M_MATH_M_CURVATURE_OPENCV_H
