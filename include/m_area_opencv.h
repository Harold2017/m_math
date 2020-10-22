//
// Created by Harold on 2020/10/14.
//

#ifndef M_MATH_M_AREA_OPENCV_H
#define M_MATH_M_AREA_OPENCV_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "m_opencv_utils.h"

namespace M_MATH {
    template<typename T>
    T Area(T* pPts, size_t rows, size_t cols) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        auto I = ToMat(rows, cols, pPts);
        I = To8U(I);
        cv::findContours(I, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        //cv::Mat drawing = cv::Mat::zeros(I.size(), CV_8UC1);
        //drawContours( drawing, contours, -1, 255, 1, cv::LINE_8, hierarchy, 0 );
        //cv::imshow( "Contours", drawing );
        //cv::waitKey();

        // outermost contour
        return cv::contourArea(contours[contours.size()-1]);
    }
}

#endif //M_MATH_M_AREA_OPENCV_H
