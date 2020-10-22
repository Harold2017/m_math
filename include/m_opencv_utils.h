//
// Created by Harold on 2020/10/19.
//

#ifndef M_MATH_M_OPENCV_UTILS_H
#define M_MATH_M_OPENCV_UTILS_H

#include <opencv2/core.hpp>
#include <vector>

namespace M_MATH {
    // all use float inside
    inline cv::Mat ToMat(size_t rows, size_t cols, float *data) {
        return cv::Mat(rows, cols, CV_32FC1, data);
    }

    inline cv::Mat ToMat(size_t rows, size_t cols, double *data) {
        cv::Mat out;
        cv::Mat(rows, cols, CV_64FC1, data).convertTo(out, CV_32FC1);
        return out;
    }

    inline void ToVec(cv::Mat const& in, std::vector<float> & out) {
        if (in.isContinuous()) {
            // array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
            out.assign((float*)in.data, (float*)in.data + in.total()*in.channels());
        } else {
            for (int i = 0; i < in.rows; ++i) {
                out.insert(out.end(), in.ptr<float>(i), in.ptr<float>(i)+in.cols*in.channels());
            }
        }
    }

    // in should in [0, 1]
    inline cv::Mat To8U(cv::Mat const& in) {
        double min, max;
        cv::minMaxLoc(in, &min, &max);
        cv::Mat out;
        if (min != max)
            in.convertTo(out, CV_8U, 255.0/(max-min),-255.0*min/(max-min));
        else
            if (min == 0)
                out = cv::Mat::zeros(in.rows, in.cols, CV_8U);
            else
                out = cv::Mat::ones(in.rows, in.cols, CV_8U);
        return out;
    }
}

#endif //M_MATH_M_OPENCV_UTILS_H
