//
// Created by Harold on 2021/02/26.
//

#ifndef M_MATH_M_IMG_TMPL_MATCH_H
#define M_MATH_M_IMG_TMPL_MATCH_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace M_MATH {
    class TmplMatching {
    public:
        TmplMatching(std::string const& srcImgPath, std::string const& tmplImgPath) {
            _srcImg = cv::imread(srcImgPath, cv::IMREAD_COLOR);
            _tmplImg = cv::imread(tmplImgPath, cv::IMREAD_COLOR);
        }
        // no mask
        void Match(cv::TemplateMatchModes const match_method) {
            if (_srcImg.empty() || _tmplImg.empty())
                throw;
            cv::Mat result(_srcImg.cols - _tmplImg.cols + 1, _srcImg.rows - _tmplImg.rows + 1, CV_32FC1);
            cv::matchTemplate(_srcImg, _tmplImg, result, match_method);
            cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
            double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
            if (match_method == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED)
                _matchLoc = minLoc;
            else
                _matchLoc = maxLoc;
        }
        void DetectEdge(double canny_threshold_1 = 0, double canny_threshold_2 = 255) {
            cv::Mat src, edges;
            cv::cvtColor(_srcImg(cv::Rect(_matchLoc.x, _matchLoc.y, _tmplImg.cols, _tmplImg.rows)), src, cv::COLOR_BGR2GRAY);
            // reduce noise
            cv::blur(src, src, cv::Size(3, 3));
            cv::Canny(src, edges, canny_threshold_1, canny_threshold_2, 3, false);
            cv::findNonZero(edges, _edges_pts);
            _detect_edge = true;
        }
        cv::Point Locate(cv::Point const& label_pt_on_tmpl) {
            // convert tmpl label pts pos to src pts pos
            cv::Point label_pt_on_src = label_pt_on_tmpl;
            // edge detect
            // TODO: need ensure input pt are on tmpl img edges
            if (_detect_edge) {
                auto min_dist = std::numeric_limits<double>::max();
                cv::Point min_pt;
                for (auto const& pt : _edges_pts) {
                    auto dist = cv::norm(pt - label_pt_on_tmpl);
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_pt = pt;
                    }
                }
                label_pt_on_src = min_pt;
            }
            return label_pt_on_src + _matchLoc;
        }

    public:
        cv::Mat _srcImg, _tmplImg;  // CV_8U
        cv::Point _matchLoc;
        bool _detect_edge = false;
        std::vector<cv::Point> _edges_pts;
    };
}

#endif //M_MATH_M_IMG_TMPL_MATCH_H