//
// Created by Harold on 2021/1/22.
//

#include "m_bezier.hpp"

// area bounding by bezier curve and end-points-line
template<size_t N>
float area(Bezier::Bezier<N> const& bezier, size_t intervals) {
    assert(intervals != 0);
    // flat bezier curve to segments
    auto flatten = [&](size_t segmentCnt) -> std::vector<Bezier::Point> {
        std::vector<Bezier::Point> res;
        res.reserve(segmentCnt + 1);
        float step = 1.0f / segmentCnt;
        float t;
        for (auto i = 0; i < segmentCnt + 1; ++i) {
            t = i * step;
            res.template emplace_back(bezier.valueAt(t, 0), bezier.valueAt(t, 1));
        }
        return res;
    };
    auto polygon_pts = flatten(intervals);
    //for (auto const& pt : polygon_pts)
    //    printf("(%f, %f)\n", pt.x, pt.y);
    Bezier::Point p1, p2;
    auto area = 0.f;
    for (auto i = 0; i < intervals + 1; ++i) {
        p1 = polygon_pts[i];
        p2 = polygon_pts[(i+1) % (intervals + 1)];
        area += p1.x * p2.y - p2.x * p1.y;
    }
    return std::abs(area/2.f);
}

int main() {
    Bezier::Bezier<3> b3({ { 120, 160 }, { 35, 200 }, { 220, 260 }, {220, 40} });

    printf("length(100): %f\n", b3.length(100));
    printf("area(10): %f\n", area(b3, 10));
    //printf("area(50): %f\n", area(b3, 50));
    //printf("area(100): %f\n", area(b3, 100));
    //printf("area(500): %f\n", area(b3, 500));
    //printf("area(1000): %f\n", area(b3, 1000));
    //printf("area(10000): %f\n", area(b3, 10000));

    return 0;
}
