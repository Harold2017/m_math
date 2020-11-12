//
// Created by Harold on 2020/11/12.
//

#include <iostream>
#include "m_fft_opencv.h"
#include "m_opencv_utils.h"

void HighPass(cv::Mat const& in, cv::Mat &out, int cut_off_freq) {
    M_MATH::ForwardFFT(in, out);
    for (auto i = 0; i < cut_off_freq * 2; i++)
        out.at<float>(i) = 0;
    cv::idft(out, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
}

void LowPass(cv::Mat const& in, cv::Mat &out, int cut_off_freq) {
    M_MATH::ForwardFFT(in, out);
    for (auto i = cut_off_freq * 2; i < out.cols * 2; i++)
        out.at<float>(i) = 0;
    cv::idft(out, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
}

void BandPass(cv::Mat const& in, cv::Mat &out, int cut_off_freq_l, int cut_off_freq_r) {
    M_MATH::ForwardFFT(in, out);
    for (auto i = 0; i < cut_off_freq_l * 2; i++)
        out.at<float>(i) = 0;
    for (auto i = cut_off_freq_r * 2; i < out.cols * 2; i++)
        out.at<float>(i) = 0;
    cv::idft(out, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
}

void BandReject(cv::Mat const& in, cv::Mat &out, int cut_off_freq_l, int cut_off_freq_r) {
    M_MATH::ForwardFFT(in, out);
    for (auto i = cut_off_freq_l * 2; i < cut_off_freq_r * 2; i++)
        out.at<float>(i) = 0;
    cv::idft(out, out, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
}

int main() {
    std::vector<float> v = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    auto o = M_MATH::ToMat(1, v.size(), v.data());
    cv::Mat out;
    M_MATH::ForwardFFT(o, out);
    std::cout << out << std::endl;

    cv::Mat hp, lp, bp, bj;
    HighPass2D(o, hp, 3);
    std::cout << "hp: " << hp << std::endl;
    LowPass2D(o, lp, 3);
    std::cout << "lp: " << lp << std::endl;
    BandPass2D(o, bp, 3, 5);
    std::cout << "bp: " << bp << std::endl;
    BandReject2D(o, bj, 3, 5);
    std::cout << "bj: " << bj << std::endl;

    return 0;
}