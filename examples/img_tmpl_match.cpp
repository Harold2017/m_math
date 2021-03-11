//
// Created by Harold on 2021/02/26.
//

#include <opencv2/highgui.hpp>
#include "m_img_tmpl_match.h"
#include <iostream>

using namespace M_MATH;

cv::Point pt(0, 0);
static void onMouse(int event, int x, int y, int, void* params);

int main(int argc, char* argv[]) {
    assert(argc == 3);
    auto tm = TmplMatching(argv[1], argv[2]);  // srcImg, tmplImg
    tm.Match(cv::TM_CCORR_NORMED);
    tm.DetectEdge();

    cv::namedWindow("tmplImg", cv::WINDOW_AUTOSIZE);
    cv::imshow("tmplImg", tm._tmplImg);

    cv::namedWindow("srcImg", cv::WINDOW_AUTOSIZE);
    auto rect = cv::Rect(tm._matchLoc.x, tm._matchLoc.y, tm._tmplImg.cols, tm._tmplImg.rows);
    cv::rectangle(tm._srcImg, rect, cv::Scalar(0, 0, 255));
    cv::imshow("srcImg", tm._srcImg);

    cv::setMouseCallback("tmplImg", onMouse, &tm);

    cv::waitKey();
    cv::destroyAllWindows();
    
    return 0;
}

static void onMouse(int event, int x, int y, int, void* params)
{
    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    auto* tm = (TmplMatching*)params;
    cv::Mat* img = &tm->_tmplImg;
    pt = cv::Point(x, y);
    cv::circle(*img, pt, 1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
    auto pt_m = tm->Locate(pt);
    cv::circle(tm->_srcImg, pt_m, 1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
    cv::imshow("tmplImg", *img);
    cv::imshow("srcImg", tm->_srcImg);
}