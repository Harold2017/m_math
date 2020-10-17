//
// Created by Harold on 2020/10/15.
//

#include <random>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "m_fft_opencv.h"

using namespace M_MATH;

// all use float inside
template<typename T>
inline cv::Mat ToMat(size_t rows, size_t cols, T *data) {
    return cv::Mat(rows, cols, CV_32FC1, data);
}

template<typename T>
void ToVec(cv::Mat const& in, std::vector<T> & out) {
    if (in.isContinuous()) {
        // array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
        out.assign((float*)in.data, (float*)in.data + in.total()*in.channels());
    } else {
        for (int i = 0; i < in.rows; ++i) {
            out.insert(out.end(), in.ptr<float>(i), in.ptr<float>(i)+in.cols*in.channels());
        }
    }
}

template<typename T>
void print_v(std::string const& name, std::vector<T> const& v) {
    std::cout << name << ": \n";
    for (auto & e : v)
        std::cout << e <<' ';
    std::cout << '\n';
}

// High pass
// for cutoff_feq, 0 is center
void HighPass(cv::Mat const& in, cv::Mat & out, int cutoff_feq) {
    int M = cv::getOptimalDFTSize( in.rows );
    int N = cv::getOptimalDFTSize( in.cols );
    cv::Mat padded;
    cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexImg;
    cv::merge(planes, 2, complexImg);
    cv::dft(complexImg, complexImg);
    cv::split(complexImg, planes);

    planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

    Rearrange(planes[0], planes[0]);
    Rearrange(planes[1], planes[1]);

    cv::Mat mag, phase;
    mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    cv::cartToPolar(planes[0], planes[1], mag, phase);
    //cv::imshow("mag", mag);
    //cv::imshow("phase", phase);

    // circle mask
    cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
    auto center = cv::Point(mag.rows/2, mag.cols/2);
    cv::circle(mask, center, cutoff_feq, CV_RGB(255, 255, 255), -1);
    cv::bitwise_not(mask, mask);
    mask.convertTo(mask, CV_32FC1);
    //cv::imshow("mask", mask);

    // apply mask
    cv::bitwise_and(mag, mask, mag);
    cv::bitwise_and(phase, mask, phase);

    //cv::imshow("mag_filtered", mag);
    //cv::imshow("phase_filtered", phase);

    cv::Mat filtered;
    cv::polarToCart(mag, phase, planes[0], planes[1]);
    cv::merge(planes, 2, filtered);
    Rearrange(filtered, filtered);

    cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
    //cv::dft(complexImg, inverseTransform, cv::DFT_INVERSE|cv::DFT_REAL_OUTPUT);

    // normalize to original scale
    double min, max;
    cv::minMaxLoc(in, &min, &max);
    cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);

    cv::imshow("original", in);
    cv::imshow("filtered", out);
    cv::waitKey();
}

// Low pass
// for cutoff_feq, 0 is center
void LowPass(cv::Mat const& in, cv::Mat & out, int cutoff_feq) {
    int M = cv::getOptimalDFTSize( in.rows );
    int N = cv::getOptimalDFTSize( in.cols );
    cv::Mat padded;
    cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexImg;
    cv::merge(planes, 2, complexImg);
    cv::dft(complexImg, complexImg);
    cv::split(complexImg, planes);

    planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

    Rearrange(planes[0], planes[0]);
    Rearrange(planes[1], planes[1]);

    cv::Mat mag, phase;
    mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    cv::cartToPolar(planes[0], planes[1], mag, phase);
    //cv::imshow("mag", mag);
    //cv::imshow("phase", phase);

    // circle mask
    cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
    auto center = cv::Point(mag.rows/2, mag.cols/2);
    cv::circle(mask, center, cutoff_feq, CV_RGB(255, 255, 255), -1);
    mask.convertTo(mask, CV_32FC1);
    //cv::imshow("mask", mask);

    // apply mask
    cv::bitwise_and(mag, mask, mag);
    cv::bitwise_and(phase, mask, phase);

    //cv::imshow("mag_filtered", mag);
    //cv::imshow("phase_filtered", phase);

    cv::Mat filtered;
    cv::polarToCart(mag, phase, planes[0], planes[1]);
    cv::merge(planes, 2, filtered);
    Rearrange(filtered, filtered);

    cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
    //cv::dft(complexImg, inverseTransform, cv::DFT_INVERSE|cv::DFT_REAL_OUTPUT);

    // normalize to original scale
    double min, max;
    cv::minMaxLoc(in, &min, &max);
    cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);

    cv::imshow("original", in);
    cv::imshow("filtered", out);
    cv::waitKey();
}

// Band pass
// for (cutoff_feq_l, cutoff_feq_h), 0 is center
void BandPass(cv::Mat const& in, cv::Mat & out, int cutoff_feq_l, int cutoff_feq_h) {
    int M = cv::getOptimalDFTSize( in.rows );
    int N = cv::getOptimalDFTSize( in.cols );
    cv::Mat padded;
    cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexImg;
    cv::merge(planes, 2, complexImg);
    cv::dft(complexImg, complexImg);
    cv::split(complexImg, planes);

    planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

    Rearrange(planes[0], planes[0]);
    Rearrange(planes[1], planes[1]);

    cv::Mat mag, phase;
    mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    cv::cartToPolar(planes[0], planes[1], mag, phase);
    cv::imshow("mag", mag);
    cv::imshow("phase", phase);

    // ring mask
    cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
    auto center = cv::Point(mag.rows/2, mag.cols/2);
    cv::circle(mask, center, cutoff_feq_h, CV_RGB(255, 255, 255), -1);
    cv::circle(mask, center, cutoff_feq_l, CV_RGB(0, 0, 0), -1);
    mask.convertTo(mask, CV_32FC1);
    cv::imshow("mask", mask);
    // apply mask
    cv::bitwise_and(mag, mask, mag);
    cv::bitwise_and(phase, mask, phase);
    cv::imshow("mag_filtered", mag);
    cv::imshow("phase_filtered", phase);

    cv::Mat filtered;
    cv::polarToCart(mag, phase, planes[0], planes[1]);
    cv::merge(planes, 2, filtered);
    Rearrange(filtered, filtered);

    cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);

    // normalize to original scale
    double min, max;
    cv::minMaxLoc(in, &min, &max);
    cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);

    cv::imshow("original", in);
    cv::imshow("filtered", out);
    cv::waitKey();
}

// Band reject
// for [0, cutoff_feq_l] && [cutoff_feq_h, infinite], 0 is center
void BandReject(cv::Mat const& in, cv::Mat & out, int cutoff_feq_l, int cutoff_feq_h) {
    int M = cv::getOptimalDFTSize( in.rows );
    int N = cv::getOptimalDFTSize( in.cols );
    cv::Mat padded;
    cv::copyMakeBorder(in, padded, 0, M - in.rows, 0, N - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexImg;
    cv::merge(planes, 2, complexImg);
    cv::dft(complexImg, complexImg);
    cv::split(complexImg, planes);

    planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));

    Rearrange(planes[0], planes[0]);
    Rearrange(planes[1], planes[1]);

    cv::Mat mag, phase;
    mag = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    phase = cv::Mat::zeros(planes[0].rows, planes[0].cols,CV_32F);
    cv::cartToPolar(planes[0], planes[1], mag, phase);
    cv::imshow("mag", mag);
    cv::imshow("phase", phase);

    // ring mask
    cv::Mat mask = cv::Mat::zeros(mag.rows, mag.cols, CV_8UC1);
    auto center = cv::Point(mag.rows/2, mag.cols/2);
    cv::circle(mask, center, cutoff_feq_h, CV_RGB(255, 255, 255), -1);
    cv::circle(mask, center, cutoff_feq_l, CV_RGB(0, 0, 0), -1);
    cv::bitwise_not(mask, mask);
    mask.convertTo(mask, CV_32FC1);
    cv::imshow("mask", mask);
    // apply mask
    cv::bitwise_and(mag, mask, mag);
    cv::bitwise_and(phase, mask, phase);

    cv::imshow("mag_filtered", mag);
    cv::imshow("phase_filtered", phase);

    cv::Mat filtered;
    cv::polarToCart(mag, phase, planes[0], planes[1]);
    cv::merge(planes, 2, filtered);
    Rearrange(filtered, filtered);

    cv::idft(filtered, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);

    // normalize to original scale
    double min, max;
    cv::minMaxLoc(in, &min, &max);
    cv::normalize(out, out, min, max, cv::NORM_MINMAX, CV_32FC1);

    cv::imshow("original", in);
    cv::imshow("filtered", out);
    cv::waitKey();
}

int main() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0, 1);
    auto gen = [&]() -> float {
        return dist(rng);
    };

    size_t const N = 100;
    size_t const res_size = N;
    std::vector<float> x;
    std::vector<float> n_x;
    x.reserve(N);
    n_x.resize(res_size);
    for (auto i = 0; i < N; ++i)
        x.emplace_back(gen());

    print_v("original", x);

    auto I = ToMat(10, 10, x.data());

    // high pass
    cv::Mat out;
    HighPass(I, out, 3);
    std::cout << out << std::endl;

    //ToVec(out, n_x);
    //print_v("filtered", n_x);

    // low pass
    cv::Mat out1;
    LowPass(I, out1, 3);
    std::cout << out1 << std::endl;

    // band pass
    cv::Mat out2;
    BandPass(I, out2, 3, 5);
    std::cout << out2 << std::endl;

    // band reject
    cv::Mat out3;
    BandReject(I, out3, 3, 5);
    std::cout << out3 << std::endl;

    return 0;
}
