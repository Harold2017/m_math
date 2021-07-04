//
// Created by Harold on 2020/10/10.
//

#include "m_fft_opencv.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "m_opencv_utils.h"
#include <fstream>
#include <numeric>

#define THETA_NUM 720

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

template<typename T>
void save_v(std::vector<T> const& v, std::string const& filename) {
    std::ofstream outfile(filename);
    for (auto const& e : v)
        outfile << e << ' ';
    outfile.close();
}

void getLinePointinImageBorder(const cv::Point& p1_in, const cv::Point& p2_in,
                               cv::Point& p1_out, cv::Point& p2_out, 
                               int rows, int cols) {
    double m = (double) (p1_in.y - p2_in.y) / (double) (p1_in.x - p2_in.x + std::numeric_limits<double>::epsilon());
    double b = p1_in.y - (m * p1_in.x);

    std::vector<cv::Point> border_point;
    double x,y;
    //test for the line y = 0
    y = 0;
    x = (y-b)/m;
    if(x > 0 && x < cols)
        border_point.push_back(cv::Point(x,y));

    //test for the line y = img.rows
    y = rows;
    x = (y-b)/m;
    if(x > 0 && x < cols)
        border_point.push_back(cv::Point(x,y));

    //check intersection with horizontal lines x = 0
    x = 0;
    y = m * x + b;
    if(y > 0 && y < rows)
        border_point.push_back(cv::Point(x,y));

    x = cols;
    y = m * x + b;
    if(y > 0 && y < rows)
        border_point.push_back(cv::Point(x,y));

    p1_out = border_point[0];
    p2_out = border_point[1];
}

// too slow
void radial_sum_integral(cv::Mat const& src) {
    auto rows = src.rows;
    auto cols = src.cols;
    cv::Point2f center{ float(cols/2), float(rows/2) };
    std::vector<float> Psum(THETA_NUM);
    // Pi/THETA_NUM radian as step
    for (unsigned s = 0; s < THETA_NUM + 1; s++) {
        float angle = s * CV_PI / THETA_NUM;
        for (auto i = 0; i < rows; i++)
            for (auto j = 0; j < cols; j++) {
                auto theta = std::atan2(float(i) - center.y, float(j) - center.x);  // radians: [0, 2Pi]
                if (theta > CV_PI) theta -= CV_PI;
                if (std::abs(angle - theta) < 0.01 * angle) Psum[s] += src.at<float>(i, j);
            }
    }
    //print_v(Psum);
    save_v(Psum, "./psum.txt");
    auto angle_in_degrees = double(std::distance(Psum.begin(), std::max_element(Psum.begin(), Psum.end()))) / THETA_NUM * 180.;  // cc angle from +x
    std::cout << "max at angle: " << angle_in_degrees << std::endl;
}

void radial_sum_integral_1(cv::Mat const& src) {
    auto rows = src.rows;
    auto cols = src.cols;
    auto N = rows < cols ? rows : cols;
    std::vector<float> Psum(THETA_NUM);

    double theta;
    cv::Point center{ cols/2, rows/2 };

    // [0, Pi]
    for (unsigned i = 0; i < THETA_NUM + 1; i++) {
        theta = double(i) / THETA_NUM * CV_PI;
        auto p = cv::Point2f(float(cols)/2 + float(N)/2 * std::cos(theta), float(rows)/2-float(N)/2 * std::sin(theta));
        cv::Point p1, p2;
        getLinePointinImageBorder(center, p, p1, p2, rows, cols);

        // iterate all pixels on the line and src
        cv::LineIterator it(src, p1, p2, 8, true);  // left-to-right
        std::vector<float> buf(it.count);
        for(int j = 0; j < it.count; j++, ++it)
            buf[j] = *(const float*)*it;
        Psum[i] = cv::sum(buf)[0];
    }
    auto angle_in_degrees = double(std::distance(Psum.begin(), std::max_element(Psum.begin(), Psum.end()))) / THETA_NUM * 180.;
    save_v(Psum, "./psum.txt");
    std::cout << "max at angle: " << angle_in_degrees << std::endl;
}

void radial_sum_integral_2(cv::Mat const& src, size_t rmin = 20, size_t rmax = 250) {
    auto rows = src.rows;
    auto cols = src.cols;
    std::vector<float> Psum(THETA_NUM);

    double theta;

    // [0, Pi]
    for (unsigned i = 0; i < THETA_NUM + 1; i++) {
        theta = double(i) / THETA_NUM * CV_PI;
        auto p1 = cv::Point2f(float(cols)/2 + float(rmin) * std::cos(theta), float(rows)/2-float(rmin) * std::sin(theta));
        auto p2 = cv::Point2f(float(cols)/2 + float(rmax) * std::cos(theta), float(rows)/2-float(rmax) * std::sin(theta));

        // iterate all pixels on the line and src
        cv::LineIterator it(src, p1, p2, 8, true);  // left-to-right
        std::vector<float> buf(it.count);
        for(int j = 0; j < it.count; j++, ++it)
            buf[j] = *(const float*)*it;
        Psum[i] = cv::sum(buf)[0];
    }
    auto angle_in_degrees = double(std::distance(Psum.begin(), std::max_element(Psum.begin(), Psum.end()))) / THETA_NUM * 180.;
    save_v(Psum, "./psum.txt");
    std::cout << "max at angle: " << angle_in_degrees << std::endl;
}

void radial_sum_integral_2_averaged(cv::Mat const& src, size_t rmin = 20, size_t rmax = 250) {
    auto rows = src.rows;
    auto cols = src.cols;
    std::vector<float> Psum(THETA_NUM);

    double theta;

    // [0, Pi]
    for (unsigned i = 0; i < THETA_NUM + 1; i++) {
        theta = double(i) / THETA_NUM * CV_PI;
        auto p1 = cv::Point2f(float(cols)/2 + float(rmin) * std::cos(theta), float(rows)/2-float(rmin) * std::sin(theta));
        auto p2 = cv::Point2f(float(cols)/2 + float(rmax) * std::cos(theta), float(rows)/2-float(rmax) * std::sin(theta));

        // iterate all pixels on the line and src
        cv::LineIterator it(src, p1, p2, 8, false);  // left-to-right or not
        std::vector<float> buf(it.count);
        for(int j = 0; j < it.count; j++, ++it)
            buf[j] = *(const float*)*it;
        Psum[i] = cv::mean(buf)[0];
    }
    auto angle_in_degrees = double(std::distance(Psum.begin(), std::max_element(Psum.begin(), Psum.end()))) / THETA_NUM * 180.;
    save_v(Psum, "./psum.txt");
    std::cout << "max at angle: " << angle_in_degrees << std::endl;
}

int main(int argc, char* argv[])
{
    /*
    std::vector<float> A(49);
    std::iota(A.begin(), A.end(), 0);
    auto AA = M_MATH::ToMat(7, 7, A.data());
    cv::Mat dst, dst1;
    M_MATH::ForwardFFT(AA, dst);
    std::cout << "fft with optimal size: \n" << dst << std::endl;
    M_MATH::ForwardFFT_Not_Optimized_Size(AA, dst1);
    std::cout << "\nfft without optimal size: \n" << dst1 << std::endl;
    cv::Mat iAA;
    M_MATH::InverseFFT(dst, iAA);
    std::cout << "\nifft of fft with optimal size: \n" << iAA << std::endl;
    M_MATH::InverseFFT(dst1, iAA);
    std::cout << "\nifft of fft without optimal size: \n" << iAA << std::endl;

    cv::Mat planes[2], mag, mag1;
    cv::split(dst, planes);
    cv::magnitude(planes[0], planes[1], mag);
    mag = mag(cv::Rect(0, 0, mag.cols & -2, mag.rows & -2));
    M_MATH::Rearrange(mag, mag);
    cv::normalize(mag, mag, 0, 1, cv::NORM_MINMAX);
    std::cout << "\nmagnitude of fft with optimal size: \n" << mag << std::endl;
    cv::imshow("mag of fft with optimal size", mag);
    cv::split(dst1, planes);
    cv::magnitude(planes[0], planes[1], mag1);
    mag1 = mag1(cv::Rect(0, 0, mag1.cols & -2, mag1.rows & -2));
    M_MATH::Rearrange(mag1, mag1);
    cv::normalize(mag1, mag1, 0, 1, cv::NORM_MINMAX);
    std::cout << "\nmagnitude of fft without optimal size: \n" << mag1 << std::endl;
    cv::imshow("mag of fft without optimal size", mag1);
    cv::waitKey();
    */

    cv::Mat I = imread( cv::samples::findFile( argv[1] ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }


    cv::Mat mag, phase;  // phase in [0, 2Pi]
    //M_MATH::Spectrum_Mag_Phase(I, mag, phase);
    //M_MATH::Spectrum_MagInLog_Phase(I, mag, phase);
    M_MATH::Spectrum_Mag(I, mag);
    //M_MATH::Spectrum_MagInLog(I, mag);

    //cv::Mat mask;
    //cv::threshold(mag, mask, 0.8, 1, cv::THRESH_BINARY_INV);
    //cv::bitwise_and(mag, mask, mag);

    //cv::Point max_mag_loc;
    //cv::minMaxLoc(mag, nullptr, nullptr, nullptr, &max_mag_loc);
    //auto center = cv::Point{ mag.cols/2, mag.rows/2 };
    //std::cout << "max mag with phase: " << phase.at<float>(max_mag_loc) * 180.0 / CV_PI << " at angle: " << std::atan2(max_mag_loc.y - center.y, max_mag_loc.x - center.x) * 180.0 / CV_PI  << std::endl;

    //cv::imshow("Input Image", I);
    cv::imshow("spectrum magnitude", mag);
    //cv::imshow("phase", phase);

    /*
    auto rows = mag.rows;
    auto cols = mag.cols;
    auto center = cv::Point2f{ float(cols)/2.f, float(rows)/2.f };
    double max_radius = std::sqrt(rows * rows + cols * cols) / 2;
    cv::Mat mag_polar, phase_polar;
    cv::linearPolar(mag, mag_polar, center, max_radius, cv::WARP_FILL_OUTLIERS);
    cv::linearPolar(phase, phase_polar, center, max_radius, cv::WARP_FILL_OUTLIERS);
    cv::imshow("mag polar", mag_polar);
    cv::imshow("phase polar", phase_polar);
    */

    cv::Mat PSD;
    //cv::pow(mag, 2, PSD);
    M_MATH::CalcPSD(I, PSD);
    cv::imshow("PSD", PSD);

    cv::waitKey();

    //radial_sum_integral(PSD);
    //radial_sum_integral_1(PSD);


    radial_sum_integral_2(PSD, 20, 250);
    //radial_sum_integral_2_averaged(PSD, 20, 250);

    return 0;
}