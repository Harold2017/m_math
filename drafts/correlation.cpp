//
// Created by Harold on 2020/10/18.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "m_fft_opencv.h"

// https://stackoverflow.com/questions/19068085/shift-image-content-with-opencv
cv::Mat translateImg(cv::Mat &img, int offsetx, int offsety){
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    cv::warpAffine(img, img, trans_mat, img.size());
    return img;
}

enum Direction{
    ShiftUp=1, ShiftRight, ShiftDown, ShiftLeft
};

cv::Mat shiftFrame(cv::Mat const& frame, int pixels, Direction direction)
{
    //create a same sized temporary Mat with all the pixels flagged as invalid (-1)
    cv::Mat temp = cv::Mat::zeros(frame.size(), frame.type());

    switch (direction)
    {
        case(ShiftUp) :
            frame(cv::Rect(0, pixels, frame.cols, frame.rows - pixels)).copyTo(temp(cv::Rect(0, 0, temp.cols, temp.rows - pixels)));
            break;
        case(ShiftRight) :
            frame(cv::Rect(0, 0, frame.cols - pixels, frame.rows)).copyTo(temp(cv::Rect(pixels, 0, frame.cols - pixels, frame.rows)));
            break;
        case(ShiftDown) :
            frame(cv::Rect(0, 0, frame.cols, frame.rows - pixels)).copyTo(temp(cv::Rect(0, pixels, frame.cols, frame.rows - pixels)));
            break;
        case(ShiftLeft) :
            frame(cv::Rect(pixels, 0, frame.cols - pixels, frame.rows)).copyTo(temp(cv::Rect(0, 0, frame.cols - pixels, frame.rows)));
            break;
        default:
            std::cout << "Shift direction is not set properly" << std::endl;
    }

    return temp;
}

int main() {
    cv::Mat I = imread( cv::samples::findFile( "lena.jpg" ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }
    cv::imshow("original", I);


    cv::Mat I1;
    I.copyTo(I1);
    translateImg(I1, 30, 20);
    cv::imshow("transferred", I1);



    /*
    // sobel
    cv::Mat sobel_x = (cv::Mat_<float>(3,3) << -1,0,1,-2,0,2,-1,0,1);
    std::cout << sobel_x << std::endl;
    cv::Mat sobX;

    // correlation
    cv::filter2D(I, sobX, -1, sobel_x);
    cv::imshow("sobel x", sobX);

    // convolution
    cv::flip(sobel_x, sobel_x, -1);
    std::cout << sobel_x << std::endl;
    cv::filter2D(I, sobX, -1, sobel_x);
    imshow("sobel x2", sobX);

    cv::waitKey();

    std::vector<cv::Point> v;
    cv::findNonZero(sobX, v);
    std::cout << v.size() << std::endl;
     */


    /*
    // https://stackoverflow.com/questions/31561513/strange-result-of-2d-cross-correlation-using-opencvs-matchtemplate-method-in-py
    // https://stackoverflow.com/questions/28506665/explaing-cross-correlation-and-normalization-for-opencvs-match-template
    cv::Mat mt;
    //cv::matchTemplate(I, I1, mt, cv::TM_CCORR_NORMED);
    //std::cout << mt << std::endl;

    cv::Mat I2;
    I.copyTo(I2);
    std::vector<cv::Point> pairs;
    // too slow
    for (auto i = 0; i < I.rows; ++i)
        for (auto j = 0; j < I.cols; ++j) {
            translateImg(I2, i, j);
            cv::matchTemplate(I, I2, mt, cv::TM_CCORR_NORMED);
            if (cv::sum(mt)[0] < 0.2)
                pairs.emplace_back(i, j);
        }

    for (auto & e : pairs)
        std::cout << e << '\n';
    std::cout << std::endl;
     */


    // https://stackoverflow.com/questions/51347829/c-cross-correlation-of-2-shifted-images-with-opencv
    // https://scikit-image.org/docs/0.11.x/auto_examples/plot_register_translation.html
    int width = cv::getOptimalDFTSize(std::max(I.cols,I1.cols));
    int height = cv::getOptimalDFTSize(std::max(I.rows,I1.rows));
    cv::Mat fft1;
    cv::Mat fft2;

    cv::copyMakeBorder(I, fft1, 0, height - I.rows, 0, width - I.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::copyMakeBorder(I1, fft2, 0, height - I.rows, 0, width - I.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    fft1.convertTo(fft1, CV_32F);
    fft2.convertTo(fft2, CV_32F);

    cv::dft(fft1,fft1,0,I.rows);
    cv::dft(fft2,fft2,0,I1.rows);
    cv::imshow("fft1_o", fft1);
    cv::imshow("fft2", fft2);

    cv::mulSpectrums(fft1,fft2,fft1,0,true);
    fft1 = fft1/cv::abs(fft1);
    cv::idft(fft1,fft1);
    //cv::idft(fft1,fft1,cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
    double maxVal;
    cv::Point maxLoc;
    minMaxLoc(fft1,nullptr,&maxVal,nullptr,&maxLoc);
    int resX = (maxLoc.x<width/2) ? (maxLoc.x) : (maxLoc.x-width);
    int resY = (maxLoc.y<height/2) ? (maxLoc.y) : (maxLoc.y-height);

    std::cout << resX << ", " << resY << std::endl;

    M_MATH::Rearrange(fft1, fft1);

    cv::imshow("fft1_r", fft1);
    cv::waitKey();

    return 0;
}
