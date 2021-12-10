//
// Created by Harold on 2021/12/10.
//

#ifndef M_MATH_M_P2D_H
#define M_MATH_M_P2D_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace M_MATH
{
    namespace p2d
    {
        /**
         * GetLocalMaxima
         * @param src input mat (single channel)
         * @param squareSize >= 3
         * @param threshold [0, 1] factor of maxVal
         */
        cv::Mat GetLocalMaxima(cv::Mat const &src, double threshold = 0.1)
        {
            cv::Mat m0;
            cv::Mat img = src.clone();
            cv::Mat dst(src.size(), CV_8UC1, cv::Scalar(0));

            if (threshold != 0.) {
                double minVal = 0, maxVal = 0;
                cv::minMaxLoc(img, &minVal, &maxVal, nullptr, nullptr);
                double maxHeight = maxVal - minVal;
                //Find the threshold value to threshold the image
                // default: use 1/10 max as threshold
                cv::threshold(img, m0, maxVal * threshold, 1, cv::THRESH_BINARY);

                //Now delete all thresholded values from picture
                img = img.mul(m0);
            }

            auto find_neighbors = [](int rows, int cols, int i, int j, std::vector<cv::Point>& neighbors)
            {
                auto valid_pos = [=](int i, int j) {
                    return (0 <= i && i < rows) && (0 <= j && j < cols);
                };
                // eight directions
                if (valid_pos(i - 1, j)) neighbors.push_back(cv::Point(i - 1, j));
                if (valid_pos(i + 1, j)) neighbors.push_back(cv::Point(i + 1, j));
                if (valid_pos(i, j - 1)) neighbors.push_back(cv::Point(i, j - 1));
                if (valid_pos(i, j + 1)) neighbors.push_back(cv::Point(i, j + 1));
                if (valid_pos(i - 1, j - 1)) neighbors.push_back(cv::Point(i - 1, j - 1));
                if (valid_pos(i - 1, j + 1)) neighbors.push_back(cv::Point(i - 1, j + 1));
                if (valid_pos(i + 1, j - 1)) neighbors.push_back(cv::Point(i + 1, j - 1));
                if (valid_pos(i + 1, j + 1)) neighbors.push_back(cv::Point(i + 1, j + 1));
            };

            auto is_local_maximum = [&](int i, int j)
            {
                std::vector<cv::Point> neighbors;
                neighbors.reserve(8);
                find_neighbors(img.rows, img.cols, i, j, neighbors);
                for (auto n : neighbors)
                    if (img.at<uchar>(n.x, n.y) >= img.at<uchar>(i, j))  // not include plateau
                        return false;

                return true;
            };


            for (auto i = 0; i < img.rows; i++)
                for (auto j = 0; j < img.cols; j++)
                    if (is_local_maximum(i, j))
                        dst.at<uchar>(i, j) = 255;
            return dst;
        }
    }
}

#endif //M_MATH_M_P2D_H