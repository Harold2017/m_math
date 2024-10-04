//
// Created by Harold on 2022/6/13.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

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

    printf("%f\n", angle);

    // FIXME: angle handling
    auto W = size.width;
    auto H = size.height;
    if (angle > 0)
    {
        angle -= 90;
        std::swap(W, H);
    }

    auto rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(img_bgr, img_screen, rotation_matrix, img_bgr.size());
    cv::getRectSubPix(img_screen, cv::Size(W, H), center, img_screen);
}

int main(int argc, char* argv[])
{
    auto img_bgr = cv::imread(argv[1], cv::IMREAD_COLOR);

    // transform to hsv
    cv::Mat img_hsv;
    cv::cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);
    // yellow range
    auto lb = cv::Scalar(20, 100, 100);
    auto ub = cv::Scalar(30, 255, 255);

    // mask
    cv::Mat yellow_mask;
    cv::inRange(img_hsv, lb, ub, yellow_mask);
    //cv::imshow("yellow mask", yellow_mask);

    // dilate to eliminate internal gap
    cv::dilate(yellow_mask, yellow_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3}));
    cv::imshow("dilated mask", yellow_mask);

    // mask bounding box
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // for (int i = 0; i < contours.size(); i++)
    //     cv::drawContours(img_bgr, contours, i, cv::Scalar(0, 0, 255), 3);
    // cv::imshow("contours", img_bgr);

     // find rectangle contour with max area
    std::vector<cv::Point> & max_contour = contours[0];
    std::vector<cv::Point> contour_rect;
    find_rectangle_with_max_area_polydp(contours, max_contour, contour_rect);

    // crop this colored area
    cv::Mat screen;
    rotate_and_crop_screen_area(img_bgr, contour_rect, screen);
    cv::imshow("cropped screen", screen);
    cv::waitKey(0);

    // write image to file
    cv::imwrite("screen.jpg", screen);

    // call ocr to detect texts on screen and output to stdout
    // make sure tesseract folder is in your PATH and set env_var TESSDATA_PREFIX='your tessdata folder'
    // use `tesseract --help-extra` to get more info on its arguments
    std::system("tesseract screen.jpg stdout -l eng --oem 1 --psm 11");  // "> screen_text.txt"

    return 0;
}