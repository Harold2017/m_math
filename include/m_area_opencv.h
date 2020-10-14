//
// Created by Harold on 2020/10/14.
//

#ifndef M_MATH_M_AREA_OPENCV_H
#define M_MATH_M_AREA_OPENCV_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace M_MATH {
    template<typename T>
    cv::Mat ToMat(size_t rows, size_t cols, T* data) {
        auto ret = cv::Mat(rows, cols, CV_32FC1, data);
        // normalize to make it in [0, 1]
        cv::normalize(ret, ret, 0, 1, cv::NORM_MINMAX, CV_32FC1);
        return ret;
    }

    // in should in [0, 1]
    cv::Mat To8U(cv::Mat const& in) {
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

    template<typename T>
    T Area(T* pPts, size_t rows, size_t cols) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        auto I = ToMat(rows, cols, pPts);
        I = To8U(I);
        cv::findContours(I, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        //cv::Mat drawing = cv::Mat::zeros(I.size(), CV_8UC1);
        // -1 for outermost
        //drawContours( drawing, contours, -1, 255, 1, cv::LINE_8, hierarchy, 0 );
        //cv::imshow( "Contours", drawing );
        //cv::waitKey();

        return cv::contourArea(contours[contours.size()-1]);
    }
}

#endif //M_MATH_M_AREA_OPENCV_H
