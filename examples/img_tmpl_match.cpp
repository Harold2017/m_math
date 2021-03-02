//
// Created by Harold on 2021/02/26.
//

#include <opencv2/highgui.hpp>
#include "m_img_tmpl_match.h"
#include <iostream>

using namespace M_MATH;

int main(int argc, char* argv[]) {
    assert(argc == 3);
    auto pt = cv::Point{ 300, 50 };
    auto tm = TmplMatching(argv[1], argv[2]);  // srcImg, tmplImg
    tm.Match(cv::TM_CCORR_NORMED);
    tm.DetectEdge();
    auto pt_m = tm.Locate(pt);
    std::cout << pt_m << std::endl;

    cv::circle(tm._tmplImg, pt, 1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
    cv::imshow("tmplImg", tm._tmplImg);

    auto rect = cv::Rect(tm._matchLoc.x, tm._matchLoc.y, tm._tmplImg.cols, tm._tmplImg.rows);
    cv::rectangle(tm._srcImg, rect, cv::Scalar(0, 0, 255));
    cv::circle(tm._srcImg, pt_m, 1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
    cv::imshow("srcImg", tm._srcImg);

    cv::waitKey();
    
    return 0;
}