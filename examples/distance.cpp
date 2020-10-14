//
// Created by Harold on 2020/9/24.
//

#include <iostream>
#include <cmath>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Plane_3 Plane_3;

void Pts2PlaneDistNormalized(std::vector<Point_3> const& pts, Plane_3 const& pl, std::vector<double>& dist) {
    auto N = pts.size();
    dist.resize(N);
    for (auto i = 0; i < N; ++i)
        dist[i] = std::sqrt(CGAL::squared_distance(pts[i], pl));
    auto max = *(std::max_element(dist.begin(), dist.end()));
    if (max == 0)
        throw;
    for (auto i = 0; i < N; ++i)
        dist[i] /= max;
}

int main() {
    Point_3 p{1, 2, 3};
    Plane_3 pl{Point_3{0,0,0}, Point_3{1, 1, 0}, Point_3{-1, 1, 0}};

    auto d = std::sqrt(CGAL::squared_distance(p, pl));

    std::cout << d << std::endl;

    return 0;
}
