//
// Created by Harold on 2020/12/15.
//

#ifndef M_MATH_M_FILTERS_OPENCV_1_H
#define M_MATH_M_FILTERS_OPENCV_1_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//#define DEBUG_PLOT

namespace M_MATH {
    enum FilterType {
        ILP,  ///< ideal low pass filter
        BLP,  ///< Butterworth low pass filter
        GLP,  ///< Gaussian low pass filter

        IHP,  ///< ideal high pass filter
        BHP,  ///< Butterworth high pass filter
        GHP,  ///< Gaussian high pass filter

        IBP,  ///< ideal band pass filter
        BBP,  ///< Butterworth band pass filter
        GBP,  ///< Gaussian band pass filter

        IBR,  ///< ideal band reject filter
        BBR,  ///< Butterworth band reject filter
        GBR,  ///< Gaussian band reject filter
    };

    ///\note if band pass or band reject, radius_1 should > radius
    cv::Mat createFilter(cv::Size const& size,
                         cv::Point const& center,
                         float radius,
                         float radius_1,
                         FilterType type,
                         int BLP_Order = 2) {
        cv::Mat lpFilter = cv::Mat::zeros(size, CV_32FC1);
        int rows = size.height;
        int cols = size.width;
        if (radius <= 0)
            return lpFilter;
        switch (type) {
            // low pass
            case ILP: {
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        auto D = cv::sqrt(cv::pow(r - center.y, 2) +
                                          cv::pow(c - center.x, 2));
                        if (D < radius)
                            lpFilter.at<float>(r, c) = 1;
                        else
                            lpFilter.at<float>(r, c) = 0;
                    }
                break;
            }
            case BLP: {
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        lpFilter.at<float>(r, c) = float(1.0 / (1.0 +
                                cv::pow(cv::sqrt(cv::pow(r - center.y, 2) +
                                                 cv::pow(c - center.x, 2)) /
                                        radius, 2.0 * BLP_Order)));
                    }
                break;
            }
            case GLP: {
                for (auto r = 0; r < rows; r++)
                    for (auto c = 0; c < cols; ++c)
                        lpFilter.at<float>(r, c) = float(cv::exp(-(cv::pow(r - center.y, 2) +
                                                                   cv::pow(c - center.x, 2)) /
                                                                 (2 * cv::pow(radius, 2))));
                break;
            }

            // high pass
            case IHP: {
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        auto D = cv::sqrt(cv::pow(r - center.y, 2) +
                                          cv::pow(c - center.x, 2));
                        if (D <= radius)
                            lpFilter.at<float>(r, c) = 0;
                        else
                            lpFilter.at<float>(r, c) = 1;
                    }
                break;
            }
            case BHP: {
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        lpFilter.at<float>(r, c) = 1 - float(1.0 / (1.0 +
                                                                    cv::pow(cv::sqrt(cv::pow(r - center.y, 2) +
                                                                                     cv::pow(c - center.x, 2)) /
                                                                            radius, 2.0 * BLP_Order)));
                    }
                break;
            }
            case GHP: {
                for (auto r = 0; r < rows; r++)
                    for (auto c = 0; c < cols; ++c)
                        lpFilter.at<float>(r, c) = 1 - float(cv::exp(-(cv::pow(r - center.y, 2) +
                                                                       cv::pow(c - center.x, 2)) /
                                                             (2 * cv::pow(radius, 2))));
                break;
            }

            // band pass
            case IBP: {
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        auto D = cv::sqrt(cv::pow(r - center.y, 2) +
                                          cv::pow(c - center.x, 2));
                        if (D >= radius && D <= radius_1)
                            lpFilter.at<float>(r, c) = 1;
                        else
                            lpFilter.at<float>(r, c) = 0;
                    }
                break;
            }
            case BBP: {
                auto BW = radius_1 - radius;
                auto D0 = (radius + radius_1) / 2;
                auto D02 = cv::pow(D0, 2);
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        auto D2 = cv::pow(r - center.y, 2) +
                                  cv::pow(c - center.x, 2);
                        auto D = cv::sqrt(D2);
                        lpFilter.at<float>(r, c) = 1 - float(1.0 / (1.0 +
                                                                    cv::pow(D * BW / (D2 - D02),
                                                                            2.0 * BLP_Order)));
                    }
                break;
            }
            case GBP: {
                auto BW = radius_1 - radius;
                auto D0 = (radius + radius_1) / 2;
                auto D02 = cv::pow(D0, 2);
                for (auto r = 0; r < rows; r++)
                    for (auto c = 0; c < cols; ++c) {
                        auto D2 = cv::pow(r - center.y, 2) +
                                  cv::pow(c - center.x, 2);
                        auto D = cv::sqrt(D2);
                        lpFilter.at<float>(r, c) = float(cv::exp(-cv::pow((D2 - D02) / (D * BW), 2)));
                    }
                break;
            }

            // band reject
            case IBR: {
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        auto D = cv::sqrt(cv::pow(r - center.y, 2) +
                                          cv::pow(c - center.x, 2));
                        if (D >= radius && D <= radius_1)
                            lpFilter.at<float>(r, c) = 0;
                        else
                            lpFilter.at<float>(r, c) = 1;
                    }
                break;
            }
            case BBR: {
                auto BW = radius_1 - radius;
                auto D0 = (radius + radius_1) / 2;
                auto D02 = cv::pow(D0, 2);
                for (auto r = 0; r < rows; ++r)
                    for (auto c = 0; c < cols; ++c) {
                        auto D2 = cv::pow(r - center.y, 2) +
                                  cv::pow(c - center.x, 2);
                        auto D = cv::sqrt(D2);
                        lpFilter.at<float>(r, c) = float(1.0 / (1.0 +
                                                                cv::pow(D * BW / (D2 - D02),
                                                                        2.0 * BLP_Order)));
                    }
                break;
            }
            case GBR: {
                auto BW = radius_1 - radius;
                auto D0 = (radius + radius_1) / 2;
                auto D02 = cv::pow(D0, 2);
                for (auto r = 0; r < rows; r++)
                    for (auto c = 0; c < cols; ++c) {
                        auto D2 = cv::pow(r - center.y, 2) +
                                  cv::pow(c - center.x, 2);
                        auto D = cv::sqrt(D2);
                        lpFilter.at<float>(r, c) = 1 - float(cv::exp(-cv::pow((D2 - D02) / (D * BW), 2)));
                    }
                break;
            }
        }
        return lpFilter;
    }

    /// pre-process input image, multiply every element with (-1) ^ (r + c)
    void preprocess(cv::Mat const& in, cv::Mat &out) {
        in.convertTo(out, CV_32FC1, 1.0, 0.0);
        auto rows = in.rows;
        auto cols = in.cols;
        for (auto r = 0; r < rows; ++r)
            for (auto c = 0; c < cols; ++c)
                if ((r + c) % 2)
                    out.at<float>(r, c) *= -1;
    }

    void fft2Img(cv::InputArray in, cv::OutputArray out) {
        auto I = in.getMat();
        auto rows = I.rows;
        auto cols = I.cols;
        auto rPad = cv::getOptimalDFTSize(rows);
        auto cPad = cv::getOptimalDFTSize(cols);
        // pad 0 at left and bottom side
        cv::Mat II;
        cv::copyMakeBorder(I, II, 0, rPad - rows, 0, cPad - cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::dft(II, out, cv::DFT_COMPLEX_OUTPUT);
        // inverse and only obtain real part: cv::dft(out, iout, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
        // crop after inverse: cv::Mat i = iout(cv::Rect(0, 0, ILP.rows, ILP.cols)).clone()
    }

    void amplitudeSpectrum(cv::InputArray _srcFFT, cv::OutputArray _dstSpectrum) {
        // input fft is complex matrix, which has two channels
        CV_Assert(_srcFFT.channels() == 2);
        std::vector<cv::Mat> FFT2Channel(2);
        cv::split(_srcFFT, FFT2Channel);
        cv::magnitude(FFT2Channel[0], FFT2Channel[1], _dstSpectrum);
    }

    void phaseSpectrum(cv::InputArray _srcFFT, cv::OutputArray _dstPhase) {
        // input fft is complex matrix, which has two channels
        CV_Assert(_srcFFT.channels() == 2);
        std::vector<cv::Mat> FFT2Channel(2);
        cv::split(_srcFFT, FFT2Channel);
        cv::phase(FFT2Channel[0], FFT2Channel[1], _dstPhase);
    }

    cv::Mat graySpectrum(cv::Mat const& spectrum) {
        cv::Mat dst;
        cv::log(spectrum + 1, dst);
        cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
        dst.convertTo(dst, CV_8UC1, 255, 0);
        return dst;
    }

    void Filter(cv::Mat const& in, cv::Mat &out, float radius, float radius_1, FilterType type, int BLP_Order = 2) {
        // convert type to CV_32FC1
        cv::Mat I;
        in.convertTo(I, CV_32FC1, 1.0, 0.0);
        // preprocess
        cv::Mat inp;
        preprocess(I, inp);
        // fft
        cv::Mat F;
        fft2Img(inp, F);
        cv::Mat amp;
        amplitudeSpectrum(F, amp);
        auto spectrum = graySpectrum(amp);
#ifdef DEBUG_PLOT
        cv::imshow("spectrum", spectrum);
#endif
        cv::Point maxLoc;  // maxLoc is center
        cv::minMaxLoc(spectrum, nullptr, nullptr, nullptr, &maxLoc);
        auto lpFilter = createFilter(spectrum.size(), maxLoc, radius, radius_1, type, BLP_Order);
        // create filtered spectrum
        cv::Mat FlpFilter;
        FlpFilter.create(F.size(), F.type());
        // apply low pass filter
        for (auto r = 0; r < lpFilter.rows; ++r)
            for (auto c = 0; c < lpFilter.cols; ++c) {
                auto F_rc = F.at<cv::Vec2f>(r, c);
                auto lpFilter_rc = lpFilter.at<float>(r, c);
                FlpFilter.at<cv::Vec2f>(r, c) = F_rc * lpFilter_rc;
            }
#ifdef DEBUG_PLOT
        cv::Mat FlpSpectrum;
        amplitudeSpectrum(FlpFilter, FlpSpectrum);
        FlpSpectrum = graySpectrum(FlpSpectrum);
        cv::imshow("filtered spectrum", FlpSpectrum);
#endif
        // ifft
        cv::dft(FlpFilter, out, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
        // pre-process again
        preprocess(out, out);
        out = out(cv::Rect(0, 0, in.rows, in.cols)).clone();
    }
}

#endif //M_MATH_M_FILTERS_OPENCV_1_H
