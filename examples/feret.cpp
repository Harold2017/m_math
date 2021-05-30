//
// Created by Harold on 2021/5/30.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

// https://blogs.mathworks.com/steve/2018/04/17/feret-properties-wrapping-up/

int main(int argc, char* argv[]) {
    // ./examples/m_feret ../files/feret.png
    auto I = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(I, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    cv::Mat drawing = cv::Mat::zeros(I.size(), CV_8UC1);
    //drawContours(drawing, contours, -1, 255, 1, cv::LINE_8, hierarchy, 0);
    //cv::imshow("contours", drawing);

    auto contour = contours[contours.size()-1];

    // perimeter
    std::cout << cv::arcLength(contour, true) << std::endl;

    // axis aligned feret
    auto rect = cv::boundingRect(contour);
    cv::rectangle(I, rect, cv::Scalar( 255, 255, 255 ));

    // min area rect
    auto min_rect = cv::minAreaRect(contour);
    cv::Point2f vtx[4];
    min_rect.points(vtx);
    for(auto i = 0; i < 4; i++)
        cv::line(I, vtx[i], vtx[(i+1)%4], cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // min area circle
    cv::Point2f center;
    float radius = 0;
    minEnclosingCircle(contour, center, radius);
    cv::circle(I, center, cvRound(radius), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // equivalent circle
    auto area = cv::contourArea(contour);
    auto r = sqrt(area / CV_PI);
    auto moments = cv::moments(contour);
    auto c = cv::Point2d{moments.m10/moments.m00, moments.m01/moments.m00};
    cv::circle(I, c, cvRound(r), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
    cv::imshow("img", I);
    cv::waitKey();

    return 0;
}
