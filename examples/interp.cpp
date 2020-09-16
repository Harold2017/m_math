//
// Created by Harold on 2020/9/16.
//

#include <cmath>
#include <iostream>
#include <vector>
#include "m_interp.h"
#include "utils.h"

using namespace M_MATH;

int main() {
    size_t const N = 10;
    std::vector<double> x(N), y(N), z(N*N), nx(2*N-1), ny(2*N-1), nz((2*N-1)*(2*N-1));

    for (int i = 0; i < N; ++i) {
        x[i] = i;
        y[i] = 2*sin(i);
    }

    nx = linspace<double>(0, N-1, 2*N-1);

    std::cout << "1d original: ";
    for (auto i = 0; i < N; ++i)
        std::cout << '(' << x[i] << ", " << y[i] << ')' << ' ';
    std::cout << std::endl;

    for (int e = Interpolation1D::LINEAR; e != Interpolation1D::STEFFEN + 1; ++e) {
        Interpolation1D::interpolate(x.data(), y.data(), N, static_cast<Interpolation1D::InterpType>(e),
                                     nx.data(), 2*N, ny.data());
        std::cout << e << ": ";
        for (auto i = 0; i < 2*N; ++i)
            std::cout << '(' << nx[i] << ", " << ny[i] << ')' << ' ';
        std::cout << std::endl;
    }

    std::cout << std::endl;

    for (int e = Interpolation1D::LINEAR; e != Interpolation1D::STEFFEN + 1; ++e) {
        Interpolation1D::interpolate(x.begin(), x.end(), y.begin(), y.end(),
                                     static_cast<Interpolation1D::InterpType>(e),
                                     nx.begin(), nx.end(),
                                     ny.begin());
        std::cout << e << ": ";
        for (auto i = 0; i < 2*N; ++i)
            std::cout << '(' << nx[i] << ", " << ny[i] << ')' << ' ';
        std::cout << std::endl;
    }

    std::cout << "\n\n" << std::endl;

    /******************************************************************************/

    x = linspace<double>(0, N-1, N);
    y = linspace<double>(0, N-1, N);
    z = linspace<double>(0, N*N, N*N);
    nx = linspace<double>(0, N-1, 2*N-1);
    ny = linspace<double>(0, N-1, 2*N-1);

    std::cout << "2d original: ";
    for (auto i = 0; i < x.size(); ++i)
        for (auto j = 0; j < y.size(); ++j)
            std::cout << '(' << x[i] << ", " << y[j] << ", " << z[j * N + i] << ')' << ' ';
    std::cout << std::endl;

    for (int e = Interpolation2D::BI_LINEAR; e != Interpolation2D::BI_CUBIC + 1; ++e) {
        Interpolation2D::interpolate(x.data(), y.data(), z.data(), x.size(), y.size(),
                                     static_cast<Interpolation2D::InterpType>(e),
                                     nx.data(), ny.data(), nx.size(), ny.size(),
                                     nz.data());

        std::cout << e << ": ";
        for (auto i = 0; i < nx.size(); ++i)
            for (auto j = 0; j < ny.size(); ++j)
                std::cout << '(' << nx[i] << ", " << ny[j] << ", " << nz[j * N + i] << ')' << ' ';
        std::cout << std::endl;
    }

    std::cout << std::endl;

    for (int e = Interpolation2D::BI_LINEAR; e != Interpolation2D::BI_CUBIC + 1; ++e) {
        Interpolation2D::interpolate(x.begin(), x.end(),
                                     y.begin(), y.end(),
                                     z.begin(), z.end(),
                                     static_cast<Interpolation2D::InterpType>(e),
                                     nx.begin(), nx.end(),
                                     ny.begin(), ny.end(),
                                     nz.begin());

        std::cout << e << ": ";
        for (auto i = 0; i < nx.size(); ++i)
            for (auto j = 0; j < ny.size(); ++j)
                std::cout << '(' << nx[i] << ", " << ny[j] << ", " << nz[j * N + i] << ')' << ' ';
        std::cout << std::endl;
    }

    return 0;
}
