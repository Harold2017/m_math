//
// Created by Harold on 2021/7/5.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include<iostream>

#include "m_opencv_utils.h"

void ShowConnectedComponents(cv::Mat const &src, double threshold = 0.1, bool fully_connected = true, double small_area_ignored = 100.) {
    auto src_bin = M_MATH::To8U(src);
    cv::threshold(src_bin, src_bin, threshold * 255., 255., cv::THRESH_BINARY);
    cv::Mat labels, stats, centroids;
    int connectivity = fully_connected ? 8 : 4;
    int nccomps = cv::connectedComponentsWithStats(src_bin, labels, stats, centroids, connectivity);

    // show stats
    char title[1024];
    sprintf(title, "connected components number in src img: %d\n", nccomps);
    cv::String num_connect(title);
    cv::imshow(num_connect, src_bin);

    // remove components with too small area and initialize colors
    std::vector<cv::Vec3b> colors(nccomps);
    colors[0] = cv::Vec3b(0, 0, 0); // background pixels remain black.
    for (int i = 1; i < nccomps; i++)
    {
        colors[i] = cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
        // remove componentes with area < 100 (painted with black)
        if (stats.at<int>(i, cv::CC_STAT_AREA) < 100)
            colors[i] = cv::Vec3b(0, 0, 0);
    }
    // paint components with color according to labels
    cv::Mat img_color = cv::Mat::zeros(src.size(), CV_8UC3);
    for (int i = 0; i < img_color.rows; i++)
        for (int j = 0; j < img_color.cols; j++)
            img_color.at<cv::Vec3b>(i, j) = colors[labels.at<int>(i, j)];

    // show stats after small region removal
    cv::cvtColor(img_color, src_bin, cv::COLOR_BGR2GRAY);
    cv::threshold(src_bin, src_bin, 1, 255, cv::THRESH_BINARY);
    nccomps = cv::connectedComponentsWithStats(src_bin, labels, stats, centroids);
    sprintf(title, "connected components number after small region removal: %d\n", nccomps);
    num_connect = title;
    cv::imshow(num_connect, img_color);
    cv::waitKey();
}

int main(int argc, char* argv[]) {
    cv::Mat I = imread(cv::samples::findFile( argv[1] ), cv::IMREAD_GRAYSCALE);
    if(I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }

    ShowConnectedComponents(I);

    return 0;
}