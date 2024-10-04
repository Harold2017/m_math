//
// Created by Harold on 2024/10/4.
//
// code from https://github.com/Smorodov/LogPolarFFTTemplateMatcher

#include "m_fft_opencv.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>

#include "stopwatch.h"

#define USE_M_MATH
#ifndef USE_M_MATH
// Recombinate image quaters
void Recomb(cv::Mat& src, cv::Mat& dst)
{
    int cx = src.cols >> 1;
    int cy = src.rows >> 1;
    cv::Mat tmp;
    tmp.create(src.size(), src.type());
    src(cv::Rect(0, 0, cx, cy)).copyTo(tmp(cv::Rect(cx, cy, cx, cy)));
    src(cv::Rect(cx, cy, cx, cy)).copyTo(tmp(cv::Rect(0, 0, cx, cy)));
    src(cv::Rect(cx, 0, cx, cy)).copyTo(tmp(cv::Rect(0, cy, cx, cy)));
    src(cv::Rect(0, cy, cx, cy)).copyTo(tmp(cv::Rect(cx, 0, cx, cy)));
    dst = tmp;
}

// 2D Forward FFT
void ForwardFFT(cv::Mat& Src, cv::Mat* FImg, bool do_recomb = true)
{
    int M = cv::getOptimalDFTSize(Src.rows);
    int N = cv::getOptimalDFTSize(Src.cols);
    cv::Mat padded;
    cv::copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = { cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F) };
    cv::Mat complexImg;
    cv::merge(planes, 2, complexImg);
    cv::dft(complexImg, complexImg);
    cv::split(complexImg, planes);
    planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));
    if (do_recomb)
    {
        Recomb(planes[0], planes[0]);
        Recomb(planes[1], planes[1]);
    }
    planes[0] /= float(M * N);
    planes[1] /= float(M * N);
    FImg[0] = planes[0].clone();
    FImg[1] = planes[1].clone();
}

// 2D inverse FFT
void InverseFFT(cv::Mat* FImg, cv::Mat& Dst, bool do_recomb = true)
{
    if (do_recomb)
    {
        Recomb(FImg[0], FImg[0]);
        Recomb(FImg[1], FImg[1]);
    }
    cv::Mat complexImg;
    cv::merge(FImg, 2, complexImg);
    cv::idft(complexImg, complexImg);
    cv::split(complexImg, FImg);
    Dst = FImg[0].clone();
}
#endif

void highpass(cv::Size sz, cv::Mat& dst)
{
    cv::Mat a = cv::Mat(sz.height, 1, CV_32FC1);
    cv::Mat b = cv::Mat(1, sz.width, CV_32FC1);

    float step_y = CV_PI / sz.height;
    float val = -CV_PI*0.5;

    for (int i = 0; i < sz.height; ++i)
    {
        a.at<float>(i) = cos(val);
        val += step_y;
    }

    val = -CV_PI*0.5;
    float step_x = CV_PI / sz.width;
    for (int i = 0; i < sz.width; ++i)
    {
        b.at<float>(i) = cos(val);
        val += step_x;
    }

    cv::Mat tmp = a*b;
    dst = (1.0 - tmp).mul(2.0 - tmp);
}

float logpolar(cv::Mat& src, cv::Mat& dst)
{
    float radii = src.cols;
    float angles = src.rows;
    cv::Point2f center(src.cols / 2, src.rows / 2);
    float d = cv::norm(cv::Vec2f(src.cols - center.x, src.rows - center.y));
    float log_base = pow(10.0, log10(d) / radii);
    float d_theta = CV_PI / (float)angles;
    float theta = CV_PI / 2.0;
    float radius = 0;
    cv::Mat map_x(src.size(), CV_32FC1);
    cv::Mat map_y(src.size(), CV_32FC1);
    for (int i = 0; i < angles; ++i)
    {
        for (int j = 0; j < radii; ++j)
        {
            radius = cv::pow(log_base, float(j));
            float x = radius * sin(theta) + center.x;
            float y = radius * cos(theta) + center.y;
            map_x.at<float>(i, j) = x;
            map_y.at<float>(i, j) = y;
        }
        theta += d_theta;
    }
    cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    return log_base;
}

//-----------------------------------------------------------------------------------------------------
// As input we need equal sized images, with the same aspect ratio,
// scale difference should not exceed 1.8 times.
//-----------------------------------------------------------------------------------------------------
cv::RotatedRect LogPolarFFTTemplateMatch(cv::Mat& im0, cv::Mat& im1, double canny_threshold1, double canny_threshold2)
{
    // Accept 1 or 3 channel CV_8U, CV_32F or CV_64F images.
    CV_Assert((im0.type() == CV_8UC1) || (im0.type() == CV_8UC3) ||
        (im0.type() == CV_32FC1) || (im0.type() == CV_32FC3) ||
        (im0.type() == CV_64FC1) || (im0.type() == CV_64FC3));

    CV_Assert(im0.rows == im1.rows && im0.cols == im1.cols);

    CV_Assert(im0.channels() == 1 || im0.channels() == 3 || im0.channels() == 4);

    CV_Assert(im1.channels() == 1 || im1.channels() == 3 || im1.channels() == 4);

    cv::Mat im0_tmp = im0.clone();
    cv::Mat im1_tmp = im1.clone();
    if (im0.channels() == 3)
    {
        cv::cvtColor(im0, im0, cv::COLOR_BGR2GRAY);
    }

    if (im0.channels() == 4)
    {
        cv::cvtColor(im0, im0, cv::COLOR_BGRA2GRAY);
    }

    if (im1.channels() == 3)
    {
        cv::cvtColor(im1, im1, cv::COLOR_BGR2GRAY);
    }

    if (im1.channels() == 4)
    {
        cv::cvtColor(im1, im1, cv::COLOR_BGRA2GRAY);
    }

    if (im0.type() == CV_32FC1)
    {
       im0.convertTo(im0, CV_8UC1, 255.0);
    }

    if (im1.type() == CV_32FC1)
    {
       im1.convertTo(im1, CV_8UC1, 255.0);
    }

    if (im0.type() == CV_64FC1)
    {
        im0.convertTo(im0, CV_8UC1, 255.0);
    }

    if (im1.type() == CV_64FC1)
    {
        im1.convertTo(im1, CV_8UC1, 255.0);
    }


    // Canny(im0, im0, canny_threshold1, canny_threshold2); // you can change this
    // Canny(im1, im1, canny_threshold1, canny_threshold2);
    
    // Ensure both images are of CV_32FC1 type
    im0.convertTo(im0, CV_32FC1, 1.0 / 255.0);
    im1.convertTo(im1, CV_32FC1, 1.0 / 255.0);

    cv::Mat f0, f1;
#ifndef USE_M_MATH
    cv::Mat F0[2], F1[2];
    ForwardFFT(im0, F0);
    ForwardFFT(im1, F1);
    cv::magnitude(F0[0], F0[1], f0);
    cv::magnitude(F1[0], F1[1], f1);
#else
    M_MATH::Spectrum_Mag(im0, f0);
    M_MATH::Spectrum_Mag(im1, f1);
#endif
    // Create filter 
    cv::Mat h;
    highpass(f0.size(), h);

    // Apply it in freq domain
    f0 = f0.mul(h);
    f1 = f1.mul(h);
/*
    cv::Mat mask = cv::Mat::zeros(f0.size(), CV_8UC1);
    cv::rectangle(mask, cv::Point(f0.cols / 2, f0.rows / 2), cv::Point(f0.cols, 0), Scalar::all(255), -1);
    mask = 255 - mask;
    f0.setTo(0, mask);
    f1.setTo(0, mask);
*/    
    cv::normalize(f0, f0, 0, 1, cv::NORM_MINMAX);
    cv::normalize(f1, f1, 0, 1, cv::NORM_MINMAX);
    //cv::imshow("f0", f0);
    //cv::imshow("f1", f1);
    
    float log_base;
    cv::Mat f0lp, f1lp;

    log_base = logpolar(f0, f0lp);
    log_base = logpolar(f1, f1lp);

    cv::Mat dbgImg= cv::Mat::zeros(f0lp.size(), CV_32FC3);
    cv::Mat Z = cv::Mat::zeros(f0lp.size(), CV_32FC1);

    //std::vector<cv::Mat> ch = {f0lp,f1lp,Z};
    //cv::merge(ch, dbgImg);
    //cv::imshow("dbgImg", dbgImg);


    // Find rotation and scale
    cv::Point2d rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);

    float angle = 180.0 * rotation_and_scale.y / f0lp.rows;
    float scale = pow(log_base, rotation_and_scale.x);
    // --------------
    if (scale > 1.8)
    {
        rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);
        angle = -180.0 * rotation_and_scale.y / f0lp.rows;
        scale = 1.0 / pow(log_base, rotation_and_scale.x);
        if (scale > 1.8)
        {
            std::cout << "Images are not compatible. Scale change > 1.8" << std::endl;
            return cv::RotatedRect();
        }
    }
    // --------------
    if (angle < -90.0)
    {
        angle += 180.0;
    }
    else if (angle > 90.0)
    {
        angle -= 180.0;
    }

    // Now rotate and scale fragment back, then find translation
    cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point(im1.cols / 2, im1.rows / 2), angle, 1.0 / scale);

    // rotate and scale
    cv::Mat im1_rs;
    cv::warpAffine(im1, im1_rs, rot_mat, im1.size());

    // find translation
    cv::Point2d tr = cv::phaseCorrelate(im1_rs, im0);

    // compute rotated rectangle parameters
    cv::RotatedRect rr;
    rr.center = tr + cv::Point2d(im0.cols / 2, im0.rows / 2);
    rr.angle = -angle;
    rr.size.width = im1.cols / scale;
    rr.size.height = im1.rows / scale;

    im0 = im0_tmp.clone();
    im1 = im1_tmp.clone();

    return rr;
}

// LogPolarFFTTemplateMatch is much faster than ECC
int main(int argc, char* argv[])
{
    cv::Mat im0 = cv::imread(argv[1]);
    cv::Mat im1 = cv::imread(argv[2]);

    cv::Mat im0g = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat im1g = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    //cv::imshow("im1", im1);
    //cv::imshow("im0", im0);

    // As input we need equal sized images, with the same aspect ratio,
    // scale difference should not exceed 1.8 times.
    cv::RotatedRect rr;

    {
        // 92ms
        TIME_BLOCK("LogPolarFFTTemplateMatch");
        rr = LogPolarFFTTemplateMatch(im0, im1, 200, 100);
    }

    cv::Mat warped_image = cv::Mat(im0g.rows, im0g.cols, CV_32FC1);
    {
        // 775ms
        TIME_BLOCK("CV_ECC");
        cv::Mat warp_matrix = cv::Mat::eye(2, 3, CV_32F);
        cv::findTransformECC(im0g, im1g, warp_matrix, cv::MOTION_AFFINE, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.001));
        cv::warpAffine(im1g, warped_image, warp_matrix, warped_image.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    }

    // Plot rotated rectangle, to check result correctness
    cv::Point2f rect_points[4];
    rr.points(rect_points);
    for (int j = 0; j < 4; j++)
    {
        cv::line(im0, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    }

    cv::imshow("result", im0);
    cv::imshow("result1", warped_image);

    cv::waitKey();

    return 0;
}