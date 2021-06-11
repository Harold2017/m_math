//
// Created by Harold on 2021/6/11.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <map>
#include <iostream>

int main() {
    cv::Mat img(600, 600, CV_8UC1, cv::Scalar());
    cv::rectangle(img, { 100, 100 }, {500, 500}, CV_RGB(255, 255, 255), 1);
    cv::rectangle(img, { 200, 200 }, { 300, 300 }, CV_RGB(255, 255, 255), 1);
    cv::rectangle(img, { 350, 350 }, { 400, 400 }, CV_RGB(255, 255, 255), 1);
    cv::circle(img, { 250, 250 }, 20, CV_RGB(255, 255, 255), 1);
    cv::circle(img, { 275, 275 }, 4, CV_RGB(255, 255, 255), 1);
    cv::circle(img, { 250, 250 }, 10, CV_RGB(255, 255, 255), 1);

    cv::namedWindow("img");
    cv::imshow("img", img);
    cv::waitKey();

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    printf("total contours number: %zu\n", contours.size());
    std::copy(hierarchy.begin(), hierarchy.end(), std::ostream_iterator<cv::Vec4i>(std::cout));
    std::cout << std::endl;

    cv::Mat img2(600, 600, CV_8UC1, cv::Scalar());
    cv::drawContours(img2, contours, -1, CV_RGB(255, 255, 255), 3);
    cv::namedWindow("contours");
    cv::imshow("contours", img2);
    cv::waitKey();

    std::map<size_t, std::vector<size_t>> filtered_contours_indices;  // contour idx : holes...  (map -> index from outer-most(0) to inner-most(x))
    for (auto i = 0; i < contours.size(); i++)
        // filter out closed contours
        if (hierarchy[i][2] < 0) continue;  // open contours (has no child)
        else {  // closed contours
            if (hierarchy[hierarchy[i][2]][2] >= 0) {  // closed contour and has grand closed contours
                filtered_contours_indices[i] = std::vector<size_t>();
                auto grandson_index = hierarchy[hierarchy[i][2]][2];
                filtered_contours_indices[i].push_back(grandson_index);
                auto next_grandson_index = hierarchy[hierarchy[hierarchy[i][2]][2]][0];
                while (next_grandson_index >= 0) {
                    filtered_contours_indices[i].push_back(next_grandson_index);
                    next_grandson_index = hierarchy[next_grandson_index][0];
                }
            }
            // skip inner contour (first child)
            i += 1;
        }
    for (auto const& e : filtered_contours_indices) {
        std::cout << e.first << ": ";
        for (auto x : e.second)
            std::cout << x << " ";
        std::cout << '\n';
    }
    std::cout << std::endl;

    // compute area between contours
    double area = cv::contourArea(contours[filtered_contours_indices.begin()->first]);
    int sign = -1;
    for (auto const& e : filtered_contours_indices) {
        for (auto x : e.second) area += sign * cv::contourArea(contours[x]);
        sign *= -1;
    }
    std::cout << "area: " << area << std::endl;

    return 0;
}