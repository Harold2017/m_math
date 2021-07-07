//
// Created by Harold on 2021/7/7.
//

#include <opencv2/core.hpp>
#include <string>
#include <fstream>
#include <iostream>

bool Read2DPtWithDelimiter(std::string const& file_name, std::string const& delimiter, std::vector<cv::Point2f>& pts);

// ignore peaks at both end
// one peak has two zero-crossing points and it may have sub-peaks inbetween
// ignore small peaks, which length < 1% * total_length and height < 10% * peak_valley_diff_max
unsigned HighSpotCounts(std::vector<cv::Point2f> const& pts, float cutting_level) 
{
    auto N = pts.size();
    auto less = [](cv::Point2f const& p1, cv::Point2f const& p2) { return p1.y < p2.y; };
    auto minmax = std::minmax_element(pts.begin(), pts.end(), less);
    auto peak_valley_diff_max = minmax.second->y - minmax.first->y;
    auto height_threshold = cutting_level * peak_valley_diff_max + minmax.first->y;
    auto max = minmax.second->y - height_threshold;
    auto peak_left_it = std::find_if(pts.begin(), pts.end(), [=](cv::Point2f const& p) { return p.y > height_threshold; });
    auto peak_right_it = pts.begin();
    unsigned cnt = 0;
    for (auto it = peak_left_it + 1; it != pts.end(); it++) {
        if (it->y < height_threshold) {
            peak_right_it = it;
        }
        else if (it->y > height_threshold && float(std::distance(peak_left_it, peak_right_it)) > 0.005f * float(N)) {
            //std::cout << peak_left_it->x << ", " << peak_right_it->x << std::endl;
            if (std::max_element(peak_left_it, peak_right_it, less)->y - height_threshold > 0.1f * max) {
                cnt++;
                peak_left_it = it;
            }
        }
    }
    return cnt;
}

// peak first cross the upper_threshold and then fall back to cross lower_threshold
unsigned PeakCount(std::vector<cv::Point2f> const& pts, float lower_threshold, float upper_threshold) {
    auto N = pts.size();
    auto less = [](cv::Point2f const& p1, cv::Point2f const& p2) { return p1.y < p2.y; };
    auto max = std::max_element(pts.begin(), pts.end(), less)->y - upper_threshold;
    auto peak_left_it = std::find_if(pts.begin(), pts.end(), [=](cv::Point2f const& p) { return p.y > upper_threshold; });
    auto peak_right_it = pts.begin();
    unsigned cnt = 0;
    for (auto it = peak_left_it + 1; it != pts.end(); it++) {
        if (it->y < lower_threshold) {
            peak_right_it = it;
        }
        else if (it->y > upper_threshold && std::distance(peak_left_it, peak_right_it) > 0) {
            if (std::max_element(peak_left_it, peak_right_it, less)->y - upper_threshold > 0.1f * max) {
                peak_left_it = it;
                //std::cout << peak_left_it->x << ", " << peak_right_it->x << std::endl;
                cnt++;
            }
        }
    }
    return cnt;
}

// total length
float UnpackingLength(std::vector<cv::Point2f> const& pts) {
    float sum = 0.f;
    for (auto i = 1; i < pts.size(); i++)
        sum += cv::norm(pts[i] - pts[i-1]);
    return sum;
}

// mean slope angle
float ArithmeticMeanSlopeAngle(std::vector<cv::Point2f> const& pts) {
    float sum = 0.f;
    for (auto i = 1; i < pts.size(); i++) {
        auto p = pts[i] - pts[i-1];
        sum += std::abs(std::atan2(p.y, p.x));
    }
    sum /= float(pts.size()-1);
    return sum / CV_PI * 180;
}

// sqrt slope angle
float SqrtSlopeAngle(std::vector<cv::Point2f> const& pts) {
    float sum = 0.f;
    for (auto i = 1; i < pts.size(); i++) {
        auto p = pts[i] - pts[i-1];
        auto a = std::atan2(p.y, p.x);
        sum += a * a;
    }
    sum /= float(pts.size()-1);
    return std::sqrt(sum) / CV_PI * 180;
}

int main(int argc, char* argv[]) {
    std::vector<cv::Point2f> pts;
    Read2DPtWithDelimiter(argv[1], " ", pts);
    std::cout << "points number: " << pts.size() << std::endl;

    auto mean = cv::mean(pts)[1];
    auto minmax = std::minmax_element(pts.begin(), pts.end(), [](cv::Point2f const& p1, cv::Point2f const& p2) { return p1.y < p2.y; });
    auto max = minmax.second->y;
    auto min = minmax.first->y;
    auto peak_valley_diff_max = max - min;
    std::cout << "mean: " << mean << ", max: " << max << ", min: " << min << std::endl;

    auto hsc = HighSpotCounts(pts, 0.5);
    std::cout << "high spot counts: " << hsc << std::endl;

    auto pc = PeakCount(pts, mean - 0.025 * peak_valley_diff_max, mean + 0.025 * peak_valley_diff_max);
    std::cout << "peak counts: " << pc << std::endl;

    auto length = UnpackingLength(pts);
    std::cout << "unpacking length: " << length << std::endl;

    auto mean_angle = ArithmeticMeanSlopeAngle(pts);
    std::cout << "mean slope angle: " << mean_angle << std::endl;

    auto sqrt_angle = SqrtSlopeAngle(pts);
    std::cout << "sqrt angle: " << sqrt_angle << std::endl;

    return 0;
}

bool Read2DPtWithDelimiter(std::string const &file_name, std::string const &delimiter, std::vector<cv::Point2f> &pts)
{
    std::ifstream ifs;
    ifs.open(file_name, std::ios::in);
    if (!ifs.is_open())
    {
        printf("Read2DPtWithDelimiter: cannot open file: %s\n", file_name.c_str());
        return false;
    }

    while (!ifs.eof())
    {
        std::string line;
        std::getline(ifs, line);
        size_t pos = 0;
        size_t searching_begin_idx = 0;
        if ((pos = line.find(delimiter, searching_begin_idx)) != std::string::npos)
        {
            size_t begin_pos = 0;
            cv::Vec2f line_vec;
            for (int i = 0; i < 2; i++)
            {
                pos = (i < 1) ? line.find(delimiter, searching_begin_idx) : line.size();
                auto single_num = line.substr(begin_pos, pos - begin_pos);
                line_vec[i] = stof(single_num);
                begin_pos = pos + 1;
                searching_begin_idx = pos + 1;
            }
            pts.push_back(line_vec);
        }
    }
    ifs.close();

    return true;
}