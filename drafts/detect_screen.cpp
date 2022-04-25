//
// Created by Harold on 2022/4/20.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <cstdlib>
#include <iostream>

// https://docs.opencv.org/4.5.3/db/d00/samples_2cpp_2squares_8cpp-example.html#a20

void find_contours(cv::Mat const& img_gray, std::vector<std::vector<cv::Point>>& contours)
{
    // apply Canny
    // set the lower threshold to 0 (which forces edges merging)
    cv::Mat edges;
    cv::Canny(img_gray, edges, 100, 255, 5);  // 0 50
    // dilate canny output to remove potential
    // holes between edge segments
    // cv::dilate(edges, edges, cv::Mat(), cv::Point(-1,-1));

    // cv::imshow("edges", edges);

    // find hough line segments
    // std::vector<cv::Vec4i> lines;
    // cv::HoughLinesP(edges, lines, 1, CV_PI/360.0, 100, 50, 20);  // https://docs.opencv.org/4.5.3/d9/db0/tutorial_hough_lines.html
    // cv::Mat img_lines = cv::Mat::zeros(img_gray.size(), CV_8UC1);
    // for (auto const& line : lines)
    //     cv::line(img_lines, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 1);

    // cv::imshow("hough lines", img_lines);

    // invert edges
    // v::bitwise_not(edges, edges);

    // cv::imshow("inverted edges", edges);

    // find contours
    // std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // for (int i = 0; i < contours.size(); i++)
    //     cv::drawContours(img_bgr, contours, i, cv::Scalar(0, 255, 0));
}

// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/std::sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void find_rectangle_with_max_area_polydp(std::vector<std::vector<cv::Point>> const& contours, std::vector<cv::Point>& max_contour, std::vector<cv::Point>& contour_rect)
{
    // find rectangle contour with max area
    double max_area = 0, area = 0;
    std::vector<cv::Point> potential_rect;
    for (auto const& c : contours)
    {
        cv::approxPolyDP(c, potential_rect, cv::arcLength(c, true) * 0.02, true);
        // should have 4 vertices and is convex
        if (potential_rect.size() == 4 && cv::isContourConvex(potential_rect))
        {
            double maxCosine = 0;
            for(int j = 2; j < 5; j++)
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = std::abs(angle(potential_rect[j % 4], potential_rect[j - 2], potential_rect[j - 1]));
                maxCosine = std::max(maxCosine, cosine);
            }
            // if cosines of all angles are small
            // (all angles are ~90 degree)
            if( maxCosine >= 0.3 )
                continue;

            area = cv::contourArea(c);
            if (area > max_area)
            {
                max_area = area;
                max_contour = c;
                contour_rect = potential_rect;
            }
        }
    }
}

void rotate_and_crop_screen_area(cv::Mat const& img_bgr, std::vector<cv::Point> const& rect_contour, cv::Mat& img_screen)
{
    // https://stackoverflow.com/questions/11627362/how-to-straighten-a-rotated-rectangle-area-of-an-image-using-opencv-in-python
    // TODO: handle potential minAreaRect vertices outside the image
    auto rect = cv::minAreaRect(rect_contour);

    auto center = rect.center;
    auto size = rect.size;
    auto angle = rect.angle;

    auto W = size.width;
    auto H = size.height;
    if (W < H)
    {
        angle -= 90;
        std::swap(W, H);
    }

    auto rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(img_bgr, img_screen, rotation_matrix, img_bgr.size());
    cv::getRectSubPix(img_screen, cv::Size(W, H), center, img_screen);
}

void order_vertices(std::vector<cv::Point> const& four_vertices, std::vector<cv::Point>& ordered_four_vertices)
{
    assert(four_vertices.size() == 4);

    // top-left, top-right, bottom-right, bottom-left
    // Point(x, y) == column x, row y -> image top-left is 0, 0
    // top-left has smallest sum, bottom-right has largest sum
    // top-right has smallest difference, bottom-left has largest difference
    std::vector<int> sum = { four_vertices[0].x + four_vertices[0].y, four_vertices[1].x + four_vertices[1].y, four_vertices[2].x + four_vertices[2].y, four_vertices[3].x + four_vertices[3].y };
    std::vector<int> diff = { four_vertices[0].y - four_vertices[0].x, four_vertices[1].y - four_vertices[1].x, four_vertices[2].y - four_vertices[2].x, four_vertices[3].y - four_vertices[3].x };
    ordered_four_vertices.clear();
    ordered_four_vertices.reserve(4);
    auto s_min_max = std::minmax_element(sum.begin(), sum.end());
    auto d_min_max = std::minmax_element(diff.begin(), diff.end());
    // top-left
    ordered_four_vertices.push_back(four_vertices[std::distance(sum.begin(), s_min_max.first)]);
    // top-right
    ordered_four_vertices.push_back(four_vertices[std::distance(diff.begin(), d_min_max.first)]);
    // bottom-right
    ordered_four_vertices.push_back(four_vertices[std::distance(sum.begin(), s_min_max.second)]);
    // bottom-left
    ordered_four_vertices.push_back(four_vertices[std::distance(diff.begin(), d_min_max.second)]);
}

double get_visible_aspect_ratio(std::vector<cv::Point> const& ordered_four_vertices, int image_width, int image_height)
{
    // center
    double u0 = image_width / 2.0;
    double v0 = image_height / 2.0;
    // widths
    auto w1 = cv::norm(ordered_four_vertices[0] - ordered_four_vertices[1]);
    auto w2 = cv::norm(ordered_four_vertices[2] - ordered_four_vertices[3]);
    // heights
    auto h1 = cv::norm(ordered_four_vertices[0] - ordered_four_vertices[2]);
    auto h2 = cv::norm(ordered_four_vertices[1] - ordered_four_vertices[3]);
    // visible max width and height
    auto w = std::max(w1, w2);
    auto h = std::max(h1, h2);
    // visible aspect ratio
    auto ar_vis = double(w) / double(h);
    return ar_vis;
}

double get_real_aspect_ratio(std::vector<cv::Point> const& ordered_four_vertices, int image_width, int image_height)
{
    // center
    double u0 = image_width / 2.0;
    double v0 = image_height / 2.0;
    // real aspect ratio
    auto m1x = ordered_four_vertices[0].x - u0;
    auto m1y = ordered_four_vertices[0].y - v0;
    auto m2x = ordered_four_vertices[1].x - u0;
    auto m2y = ordered_four_vertices[1].y - v0;
    auto m3x = ordered_four_vertices[2].x - u0;
    auto m3y = ordered_four_vertices[2].y - v0;
    auto m4x = ordered_four_vertices[3].x - u0;
    auto m4y = ordered_four_vertices[3].y - v0;
    auto k2 = ((m1y - m4y)*m3x - (m1x - m4x)*m3y + m1x*m4y - m1y*m4x) / ((m2y - m4y)*m3x - (m2x - m4x)*m3y + m2x*m4y - m2y*m4x);
    auto k3 = ((m1y - m4y)*m2x - (m1x - m4x)*m2y + m1x*m4y - m1y*m4x) / ((m3y - m4y)*m2x - (m3x - m4x)*m2y + m3x*m4y - m3y*m4x);
    auto f_squared = abs( ((k3*m3y - m1y)*(k2*m2y - m1y) + (k3*m3x - m1x)*(k2*m2x - m1x)) / ((k3 - 1)*(k2 - 1)) );
    auto ar_real = sqrt( (pow((k2 - 1),2) + pow((k2*m2y - m1y),2)/f_squared + pow((k2*m2x - m1x),2)/f_squared) / (pow((k3 - 1),2) + pow((k3*m3y - m1y),2)/f_squared + pow((k3*m3x - m1x),2)/f_squared) );
    if (k2==1 && k3==1 ) {
        ar_real = sqrt( (pow((m2y-m1y),2) + pow((m2x-m1x),2)) / (pow((m3y-m1y),2) + pow((m3x-m1x),2)) );
    }
    return ar_real;
}

// notice: will fail if two sides parallel and other two not
void unwarp_rect(cv::Mat const& img, std::vector<cv::Point> const& four_vertices, cv::Mat& img_unwarp)
{
    assert(four_vertices.size() == 4);

    std::vector<cv::Point> ordered_four_vertices;
    order_vertices(four_vertices, ordered_four_vertices);
    // for (auto const& p : four_vertices)
    //     std::cout << p << "; ";
    // std::cout << "\n";
    for (auto const& p : ordered_four_vertices)
        std::cout << p << "; ";
    std::cout << "\n";

    // width / height ratios
    // https://stackoverflow.com/questions/38285229/calculating-aspect-ratio-of-perspective-transform-destination-image

    // widths
    auto w1 = cv::norm(ordered_four_vertices[0] - ordered_four_vertices[1]);
    auto w2 = cv::norm(ordered_four_vertices[2] - ordered_four_vertices[3]);
    // heights
    auto h1 = cv::norm(ordered_four_vertices[1] - ordered_four_vertices[2]);
    auto h2 = cv::norm(ordered_four_vertices[0] - ordered_four_vertices[3]);
    // visible max width and height
    auto w = std::max(w1, w2);
    auto h = std::max(h1, h2);
    // visible aspect ratio
    auto ar_vis = double(w) / double(h);

    // real aspect ratio
    // center
    double u0 = img.cols / 2.0;
    double v0 = img.rows / 2.0;
    auto m1x = ordered_four_vertices[0].x - u0;  // top-left
    auto m1y = ordered_four_vertices[0].y - v0;
    auto m2x = ordered_four_vertices[1].x - u0;  // top-right
    auto m2y = ordered_four_vertices[1].y - v0;
    auto m3x = ordered_four_vertices[3].x - u0;  // bottom-left
    auto m3y = ordered_four_vertices[3].y - v0;
    auto m4x = ordered_four_vertices[2].x - u0;  // bottom-right
    auto m4y = ordered_four_vertices[2].y - v0;
    auto k2 = ((m1y - m4y)*m3x - (m1x - m4x)*m3y + m1x*m4y - m1y*m4x) / ((m2y - m4y)*m3x - (m2x - m4x)*m3y + m2x*m4y - m2y*m4x);
    auto k3 = ((m1y - m4y)*m2x - (m1x - m4x)*m2y + m1x*m4y - m1y*m4x) / ((m3y - m4y)*m2x - (m3x - m4x)*m2y + m3x*m4y - m3y*m4x);
    auto f_squared = abs( ((k3*m3y - m1y)*(k2*m2y - m1y) + (k3*m3x - m1x)*(k2*m2x - m1x)) / ((k3 - 1)*(k2 - 1)) );
    auto ar_real = sqrt( (pow((k2 - 1),2) + pow((k2*m2y - m1y),2)/f_squared + pow((k2*m2x - m1x),2)/f_squared) / (pow((k3 - 1),2) + pow((k3*m3y - m1y),2)/f_squared + pow((k3*m3x - m1x),2)/f_squared) );
    if (k2 == 1 && k3 == 1) {
        ar_real = sqrt( (pow((m2y-m1y),2) + pow((m2x-m1x),2)) / (pow((m3y-m1y),2) + pow((m3x-m1x),2)) );
    }
    std::cout << ar_vis << ", " << ar_real << '\n';

    // projected width and height
    int W = 0, H = 0;
    if (ar_real < ar_vis)
    {
        W = int(w);
        H = int(W / ar_real);
    }
    else
    {
        H = int(h);
        W = int(ar_real * H);
    }
    std::cout << W << ", " << H << '\n';

    std::vector<cv::Point2f> projected = { ordered_four_vertices[0], ordered_four_vertices[1], ordered_four_vertices[2], ordered_four_vertices[3] };
    std::vector<cv::Point2f> rect = { cv::Point2f(0, 0), cv::Point2f(W, 0), cv::Point2f(W, H), cv::Point2f(0, H) };
    auto M = cv::getPerspectiveTransform(projected, rect);
    cv::warpPerspective(img, img_unwarp, M, cv::Size(W, H));
}

int main(int argc, char* argv[])
{
    auto img_bgr = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_gray;
    cv::cvtColor(img_bgr, img_gray, cv::COLOR_BGR2GRAY);

    // cv::imshow("gray img", img_gray);

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    find_contours(img_gray, contours);

    // find rectangle contour with max area
    std::vector<cv::Point> & max_contour = contours[0];
    std::vector<cv::Point> contour_rect;
    find_rectangle_with_max_area_polydp(contours, max_contour, contour_rect);

    std::vector<cv::Point> rectangles{ max_contour };
    cv::polylines(img_bgr, rectangles, true, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    cv::imshow("detect screen", img_bgr);

    cv::Mat screen;
    // rotate_and_crop_screen_area(img_bgr, max_contour, screen);
    // cv::imshow("cropped screen", screen);

    unwarp_rect(img_bgr, contour_rect, screen);
    cv::imshow("unwarp screen", screen);

    cv::waitKey(0);

    // write image to file
    cv::imwrite("screen.jpg", screen);

    // call ocr to detect texts on screen and output to stdout
    // make sure tesseract folder is in your PATH and set env_var TESSDATA_PREFIX='your tessdata folder'
    // use `tesseract --help-extra` to get more info on its arguments
    std::system("tesseract screen.jpg stdout -l eng --oem 1 --psm 3");  // "> screen_text.txt"

    return 0;
}
