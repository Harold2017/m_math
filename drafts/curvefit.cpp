//
// Created by Harold on 2020/10/15.
//

#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
    //Number of key points
    int N = key_point.size();

    // construct matrix X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_32FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<float>(i, j) = X.at<float>(i, j) +
                                     std::pow(key_point[k].x, i + j);
            }
        }
    }

    // Y matrix structure
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_32FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<float>(i, 0) = Y.at<float>(i, 0) +
                                 std::pow(key_point[k].x, i) * key_point[k].y;
        }
    }

    A = cv::Mat::zeros(n + 1, 1, CV_32FC1);
    // solving matrix A
    cv::solve(X, Y, A, cv::DECOMP_NORMAL|cv::DECOMP_SVD);
    return true;
}

int main() {
    // Create a deep blue background image for drawing
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    image.setTo(cv::Scalar(100, 0, 0));

    // Enter a fit point
    std::vector<cv::Point> points;
    points.emplace_back(100., 58.);
    points.emplace_back(150., 70.);
    points.emplace_back(200., 90.);
    points.emplace_back(252., 140.);
    points.emplace_back(300., 220.);
    points.emplace_back(350., 400.);

    // fitting the plotted points on the blank of FIG.
    for (auto & point : points)
    {
        cv::circle(image, point, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    }

    // Draw polyline
    cv::polylines(image, points, false, cv::Scalar(0, 255, 0), 1, 8, 0);

    cv::Mat A;

    polynomial_curve_fit(points, 3, A);
    std::cout << "A = " << A << std::endl;

    std::vector<cv::Point> points_fitted;

    for (int x = 0; x < 400; x++)
    {
        float y = A.at<float>(0, 0) + A.at<float>(1, 0) * x +
                   A.at<float>(2, 0)*std::pow(x, 2) + A.at<float>(3, 0)*std::pow(x, 3);

        points_fitted.emplace_back(x, y);
    }
    cv::polylines(image, points_fitted, false, cv::Scalar(0, 255, 255), 1, 8, 0);

    cv::imshow("image", image);

    cv::waitKey();

    return 0;
}
