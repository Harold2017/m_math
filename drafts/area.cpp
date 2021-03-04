//
// Created by Harold on 2021/03/04.
//

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

float BrightnessArea(const cv::Mat &img,
                     float brightness_lower,
                     float brightness_upper) {
    assert(img.channels() == 3);
    cv::Mat imgCopy, sum, tmp;
    img.convertTo(imgCopy, CV_32FC3);  // ensure RGB sum not exceed type range
    cv::transform(imgCopy, sum, cv::Matx13f(1,1,1));
    cv::inRange(sum, cv::Scalar(brightness_lower, 0, 0), cv::Scalar(brightness_upper, 0, 0), tmp);
    cv::imshow("brightness", tmp);
    cv::waitKey();
    return cv::countNonZero(tmp);
}

float ColorArea(const cv::Mat &img,
                const cv::Scalar &color_lower,
                const cv::Scalar &color_upper) {
    assert(img.channels() == 3);
    cv::Mat tmp;
    cv::inRange(img, color_lower, color_upper, tmp);
    cv::imshow("color", tmp);
    cv::waitKey();
    return cv::countNonZero(tmp);
}

int main(int argc, char* argv[]) {
    assert(argc == 2);
    auto srcImg = cv::imread(argv[1], cv::IMREAD_COLOR);
    auto b = BrightnessArea(srcImg, 100, 255 * 2);
    std::cout << b << std::endl;
    auto c = ColorArea(srcImg, { 100, 100, 100 }, { 255, 255, 255 });
    std::cout << c << std::endl;

    return 0;
}