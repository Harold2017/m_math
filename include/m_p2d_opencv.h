//
// Created by Harold on 2020/10/13.
//

#ifndef M_MATH_M_P2D_OPENCV_H
#define M_MATH_M_P2D_OPENCV_H

// code from: https://stackoverflow.com/questions/5550290/find-local-maxima-in-grayscale-image-using-opencv

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace M_MATH {
    namespace p2d {
        /**
         * GetLocalMaxima
         * @param src input mat (single channel)
         * @param dst result mat
         * @param squareSize >= 3
         * @param threshold [0, 1] factor of maxVal
         */
        void GetLocalMaxima(cv::Mat const& src,cv::Mat &dst,int squareSize, double threshold = 0.1)
        {
            if (squareSize==0)
            {
                dst = src.clone();
                return;
            }

            cv::Mat m0;
            dst = src.clone();
            cv::Point maxLoc(0,0);

            //1.Be sure to have at least 3x3 for at least looking at 1 pixel close neighbours
            //  Also the window must be <odd>x<odd>
            assert(squareSize > 2);
            int sqrCenter = (squareSize-1)/2;

            //2.Create the localWindow mask to get things done faster
            //  When we find a local maxima we will multiply the subwindow with this MASK
            //  So that we will not search for those 0 values again and again
            cv::Mat localWindowMask = cv::Mat::zeros(cv::Size(squareSize,squareSize),CV_8U);//boolean
            localWindowMask.at<unsigned char>(sqrCenter,sqrCenter)=1;

            //3.Find the threshold value to threshold the image
            double maxVal=0;
            minMaxLoc(dst, nullptr, &maxVal, nullptr, nullptr);
            // default: use 1/10 max as threshold
            cv::threshold(dst, m0, maxVal * threshold,1,cv::THRESH_BINARY);

            //4.Now delete all thresholded values from picture
            dst = dst.mul(m0);

            //put the src in the middle of the big array
            for (int row=sqrCenter;row<dst.size().height-sqrCenter;row++)
                for (int col=sqrCenter;col<dst.size().width-sqrCenter;col++)
                {
                    //1.if the value is zero it can not be a local maxima
                    if (dst.at<unsigned char>(row,col)==0)
                        continue;
                    //2.the value at (row,col) is not 0 so it can be a local maxima point
                    m0 =  dst.colRange(col-sqrCenter,col+sqrCenter+1).rowRange(row-sqrCenter,row+sqrCenter+1);
                    minMaxLoc(m0, nullptr, nullptr, nullptr, &maxLoc);
                    //if the maximum location of this subWindow is at center
                    //it means we found the local maxima
                    //so we should delete the surrounding values which lies in the subWindow area
                    //hence we will not try to find if a point is at localMaxima when already found a neighbour was
                    if ((maxLoc.x==sqrCenter)&&(maxLoc.y==sqrCenter))
                    {
                        m0 = m0.mul(localWindowMask);
                        //we can skip the values that we already made 0 by the above function
                        col+=sqrCenter;
                    }
                }
        }
    }
}

#endif //M_MATH_M_P2D_OPENCV_H
