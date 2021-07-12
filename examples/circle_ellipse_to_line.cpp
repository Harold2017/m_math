//
// Created by Harold on 2021/7/12.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <string>

// if need sub-pixel resolution, use resize to interpolate img first
template<typename T>
std::vector<cv::Point_<T>> CircleToLine(cv::Mat const& I, cv::Point_<T> const& center, T radius, size_t N = 360) {
    std::vector<cv::Point_<T>> points(N);
    for (auto i = 0; i < N; i++) {
        T theta = T(i) / T(N) * T(CV_2PI);
        points[i].x = theta * radius;
        points[i].y = I.at<T>(cvRound(center.x + radius * cos(theta)), cvRound(center.y + radius * sin(theta)));
    }
    return points;
}

template<typename T>
std::vector<cv::Point_<T>> EllipseToLine(cv::Mat const& I, cv::Point_<T> const& center, cv::Point_<T> const& radius, size_t N = 360) {
    N = N > 360 ? 360 : N;  // 1 degree as min theta
    std::vector<cv::Point2d> points;
    cv::ellipse2Poly(cv::Point2d(center.x, center.y), cv::Size2d(radius.x, radius.y), 0, 0, 360, int(360 / N), points);
    std::vector<cv::Point_<T>>  pts(N-1);
    pts[0].x = 0;
    pts[0].y = I.at<T>(cvRound(points[0].x), cvRound(points[0].y));
    T x = 0;
    for (auto i = 1; i < N-1; i++) {
        x += static_cast<T>(cv::norm(points[i] - points[i-1]));
        pts[i].x = x;
        pts[i].y = I.at<T>(cvRound(points[i].x), cvRound(points[i].y));
    }
    return pts;
}

template<typename T>
void print_pt_vec(std::vector<cv::Point_<T>> const& pts, std::string const& name) {
    std::cout << name << ":\n";
    for (auto const& p : pts)
        std::cout << p << ' ';
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    cv::Mat I = imread( cv::samples::findFile( argv[1] ), cv::IMREAD_GRAYSCALE);
    if( I.empty()){
        std::cout << "Error opening image" << std::endl;
        return EXIT_FAILURE;
    }

    I.convertTo(I, CV_32FC1);
    auto circle_line_pts = CircleToLine(I, cv::Point2f(256, 256), 50.f);
    print_pt_vec(circle_line_pts, "circle to line pts");

    auto ellipse_line_pts = EllipseToLine(I, cv::Point2f(256, 256), cv::Point2f(30, 50));
    print_pt_vec(ellipse_line_pts, "ellipse to line pts");

    return 0;
}
