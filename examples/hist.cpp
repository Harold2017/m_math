//
// Created by Harold on 2020/10/7.
//

#include <random>
#include <vector>
#include <iostream>
#include "m_hist.h"

using namespace M_MATH;

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

template<typename T>
void print_v(std::vector<std::pair<T, T>> const& v) {
    for (auto const& e : v)
        std::cout << '(' << e.first << ", " << e.second << ") ";
    std::cout << std::endl;
}

struct point2d {
    float x, y;
    friend std::ostream& operator<<(std::ostream& os, point2d const& p2d) {
        return os << '(' << p2d.x << ", " << p2d.y << ") ";
    }
};

int main() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::normal_distribution<float> dist(-0.05,0.05);
    auto gen = [&]() -> float {
        return dist(rng);
    };

    size_t N = 100;
    std::vector<float> pts(N);
    for (auto i = 0; i < N; ++i)
        pts[i] = gen();

    // print_v(pts);

    auto minmax = std::minmax_element(pts.begin(), pts.end());

    Histogram1D<float> hist1d(pts.begin(), pts.end(), *(minmax.first), (*(minmax.second)) * 1.0001, N/10);
    std::cout << "mean: " << hist1d.mean() << '\n'
              << "sigma: " << hist1d.sigma() << '\n'
              << "sum: " << hist1d.sum() << std::endl;

    auto cnt = hist1d.counts();
    auto bins = hist1d.bins();
    std::cout << "counts: ";
    print_v(cnt);
    std::cout << "bins: ";
    print_v(bins);

    std::cout << "x = " << pts[N/2] << " in [" << bins[hist1d.find(pts[N/2])].first << ", "
              << bins[hist1d.find(pts[N/2])].second << ']' << std::endl;

    std::cout << "pdf sample: x = 0.5 is " << hist1d.pdf_sample(0.5) << std::endl;

    std::cout << std::endl;


    /************************ Histogram2D **************************/

    size_t Nx = 100, Ny = 100;
    std::vector<point2d> pts2d(Nx * Ny);

    for (auto i = 0; i < Nx * Ny; ++i)
        pts2d[i] = point2d{ gen(), gen() };

    auto xminmax = std::minmax_element(pts2d.begin(), pts2d.end(),
                                       [](point2d const& p1, point2d const& p2){
        return p1.x < p2.x;
    });

    auto yminmax = std::minmax_element(pts2d.begin(), pts2d.end(),
                                       [](point2d const& p1, point2d const& p2){
                                           return p1.y < p2.y;
                                       });

    Histogram2D<float> hist2d(pts2d.begin(), pts2d.end(),
                              (*(xminmax.first)).x, (*(xminmax.second)).x * 1.0001,
                              (*(yminmax.first)).y, (*(yminmax.second)).y * 1.0001,
                              Nx/10, Ny/10);

    std::cout << "mean: (" << hist2d.mean().first << ", " << hist2d.mean().second << ")\n"
              << "sigma: (" << hist2d.sigma().first << ", " << hist2d.sigma().second << ")\n"
              << "sum: " << hist2d.sum() << '\n'
              << "cov: " << hist2d.cov()
              << std::endl;

    auto cnt2d = hist2d.counts();
    auto bins2d = hist2d.bins();
    std::cout << "counts: ";
    print_v(cnt2d);
    std::cout << "x bins: ";
    print_v(bins2d.first);
    std::cout << "y bins: ";
    print_v(bins2d.second);

    std::cout << "x = " << pts2d[Nx/2 + Ny/2 * Nx] << " in [("
              << bins2d.first[hist2d.find(pts2d[Nx/2 + Ny/2 * Nx].x, pts2d[Nx/2 + Ny/2 * Nx].y).first].first << ", "
              << bins2d.first[hist2d.find(pts2d[Nx/2 + Ny/2 * Nx].x, pts2d[Nx/2 + Ny/2 * Nx].y).first].second << ") ("
              << bins2d.second[hist2d.find(pts2d[Nx/2 + Ny/2 * Nx].x, pts2d[Nx/2 + Ny/2 * Nx].y).second].first << ", "
              << bins2d.second[hist2d.find(pts2d[Nx/2 + Ny/2 * Nx].x, pts2d[Nx/2 + Ny/2 * Nx].y).second].second
              << ")]" << std::endl;

    std::cout << "pdf sample: (x, y) = (0.5, 0.5) is " << '('
              << hist2d.pdf_sample(0.5, 0.5).first << ", "
              << hist2d.pdf_sample(0.5, 0.5).second << ')'
              << std::endl;

    return 0;
}
