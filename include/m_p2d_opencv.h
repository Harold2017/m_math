//
// Created by Harold on 2020/10/13.
//

#ifndef M_MATH_M_P2D_OPENCV_H
#define M_MATH_M_P2D_OPENCV_H

// code from: https://stackoverflow.com/questions/5550290/find-local-maxima-in-grayscale-image-using-opencv

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
        cv::Mat GetLocalMaxima(cv::Mat const &src, int squareSize = 5, double threshold = 0.1, double pruning = 0.05)
        {
            if (squareSize == 0)
            {
                return src.clone();
            }

            cv::Mat m0;
            cv::Mat img = src.clone();
            cv::Mat dst(src.size(), CV_8UC1);
            cv::Point maxLoc(0, 0);

            //1.Be sure to have at least 5x5 for at least looking at 1 pixel close neighbours
            //  Also the window must be <odd>x<odd>
            assert(squareSize > 2);
            int sqrCenter = (squareSize - 1) / 2;

            //2.Create the localWindow mask to get things done faster
            //  When we find a local maxima we will multiply the subwindow with this MASK
            //  So that we will not search for those 0 values again and again
            cv::Mat localWindowMask = cv::Mat::zeros(cv::Size(squareSize, squareSize), CV_8U); //boolean
            localWindowMask.at<unsigned char>(sqrCenter, sqrCenter) = 1;

            double minVal = 0, maxVal = 0;
            cv::minMaxLoc(img, &minVal, &maxVal, nullptr, nullptr);
            double maxHeight = maxVal - minVal;

            if (threshold != 0.) {
                //3.Find the threshold value to threshold the image
                // default: use 1/10 max as threshold
                cv::threshold(img, m0, maxVal * threshold, 1, cv::THRESH_BINARY);

                //4.Now delete all thresholded values from picture
                img = img.mul(m0);
            }

            double minV, maxV;
            //put the src in the middle of the big array
            for (int row = sqrCenter; row < img.size().height - sqrCenter; row++)
                for (int col = sqrCenter; col < img.size().width - sqrCenter; col++)
                {
                    //1.if the value is zero it can not be a local maxima
                    if (img.at<unsigned char>(row, col) == 0)
                        continue;
                    //2.the value at (row,col) is not 0 so it can be a local maxima point
                    m0 = img.colRange(col - sqrCenter, col + sqrCenter + 1).rowRange(row - sqrCenter, row + sqrCenter + 1);
                    cv::minMaxLoc(m0, &minV, &maxV, nullptr, &maxLoc);
                    //if the maximum location of this subWindow is at center
                    //it means we found the local maxima
                    //so we should delete the surrounding values which lies in the subWindow area
                    //hence we will not try to find if a point is at localMaxima when already found a neighbour was
                    if ((maxLoc.x == sqrCenter) && (maxLoc.y == sqrCenter) && ((maxV - minV) > pruning * maxHeight))  // peak height > pruning * max height
                    {
                        m0 = m0.mul(localWindowMask);
                        //we can skip the values that we already made 0 by the above function
                        col += sqrCenter;
                        dst.at<uchar>(row - sqrCenter + maxLoc.x, col - sqrCenter + maxLoc.y) = 255;
                    }
                }
            return dst;
        }
    }
}

#endif //M_MATH_M_P2D_OPENCV_H
