//
// Created by Harold on 2020/9/16.
//

#include <iostream>
#include <random>
#include <vector>
#include <algorithm>
#include <string>
#include "point.h"
#include "m_geo.h"
#include "utils.h"

using namespace M_MATH::GEO;

template<typename T>
void print_v(std::string name, std::vector<T> const& v) {
    std::cout << name << ": ";
    for (auto const& e : v)
        std::cout << e << " ";
    std::cout << std::endl;
}

template<typename T>
bool SaveLine(std::string const& filename, T x_start, T x_end, unsigned num, M_MATH::CurveFit const& cf) {
    auto X = linspace(x_start, x_end, num);
    std::vector<Point2D<T>> pts;
    pts.reserve(num);
    for (auto e : X)
        pts.emplace_back(e, cf.getY(e));
    return SaveIter(filename, pts.begin(), pts.end());
}

int main() {
    std::vector<Point2D<float>> vs;
    vs.reserve(100);
    for (auto i = 0; i < 10; i++)
        for (auto j = 0; j < 10; j++)
            vs.emplace_back(i, j);

    auto a = Area<float>(vs.begin(), vs.end());
    std::cout << "area: " << a << std::endl;

    /*******************************************************/

    std::random_device dev;
    std::mt19937 rng(dev());
    // normal distribution for S shape height distribution curve
    std::normal_distribution<float> dist(-0.05,0.05);
    auto gen = [&]() -> float {
        return dist(rng);
    };

    std::vector<Point3D<float>> pts;
    pts.reserve(100);
    for (auto i = 0; i < 10; ++i)
        for (auto j = 0; j < 10; ++j)
            pts.emplace_back(i, j, gen());

    auto less = [](Point3D<float> const& pt1, Point3D<float> const& pt2){ return pt1.z < pt2.z; };

    std::sort(pts.begin(), pts.end(), less);
    print_v("sorted pts", pts);

    auto minmax = std::minmax_element(pts.begin(), pts.end(), less);
    auto min = minmax.first->z;
    auto max = minmax.second->z;

    std::cout << "min: " << min << ", max: " << max << std::endl;

    auto hist = HistOfHeight(pts.begin(), pts.end(), max, min);
    print_v("height histogram", hist);

    auto sum = std::accumulate(hist.begin(), hist.end(), 0u);
    assert(sum == 100u);

    auto cf = HeightDistCurve(pts.begin(), pts.end(), max, min);
    auto X = linspace(0.0, 1.0, 100);
    std::vector<Point2D<float>> vp;
    vp.reserve(X.size());
    for (auto e : X)
        if (e == 0.0) {
            vp.emplace_back(e, max);
        }
        else if (e == 1.0) {
            vp.emplace_back(e, min);
        }
        else {
            vp.emplace_back(e, cf.getY(e));
        }

    SaveIter("height_dist_curve.txt", vp.begin(), vp.end());

    // use min gradient point to fit the middle part straight line, gradient is 2nd diff
    auto min_gradient = std::numeric_limits<double>::max();
    // points used range in fitting: 20% here
    auto deltaX = 2 * vp.size() / 10;
    auto it = vp.begin();
    auto k = vp.begin();
    for (auto i = vp.begin(); i != vp.begin() + (vp.size() - deltaX); ++i)
        for (int j = 0; j != 2; j++)
            for (k = i; k != i + deltaX - 2; ++k)
                if (std::abs((*(k+1)).y - (*k).y) < min_gradient) {
                    min_gradient = std::abs((*(k+1)).y - (*k).y);
                    it = k;
                }
    std::cout << "min gradient at point: " << it - vp.begin() << std::endl;
    std::vector<double> xx, yy;
    xx.reserve(deltaX);
    yy.reserve(deltaX);
    // [min_gradient, min_gradient + deltaX)
    for (auto i = 0; i < deltaX; ++i) {
        xx.emplace_back((*(it + i)).x);
        yy.emplace_back((*(it + i)).y);
    }

    // the middle part straight line
    M_MATH::CurveFit cf2;
    // robust to ignore singular points
    cf2.robustfit(xx.data(), yy.data(), xx.size(), M_MATH::CurveFit::BISQUARE, 2);
    std::vector<Point2D<float>> line;
    line.reserve(X.size());
    for (auto e : X) {
        line.emplace_back(e, cf2.getY(e));
    }
    SaveIter("middle_part_line.txt", line.begin(), line.end());

    return 0;
}
