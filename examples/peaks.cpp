//
// Created by Harold on 2020/9/17.
//

#include <iostream>
#include "m_peaks.h"

using namespace M_MATH;

int main()
{
    int a[][8] = {
            { 25, 58, 63, 23, 21, 34, 21, 50 },
            { 32, 45, 43, 40, 41, 32, 30, 27 },
            { 34, 40, 38, 39, 36, 28, 30, 35 },
            { 40, 45, 42, 48, 32, 34, 29, 32 },
            { 39, 39, 40, 42, 47, 49, 27, 30 },
            { 31, 31, 31, 32, 32, 33, 44, 35 }
    };
    auto pv = Peak2D::find_peaks_and_valleys(a, 6, 8, [](int x){ return x; });

    for (auto peak : pv.first)
        std::cout << "peak: " << a[peak.first][peak.second] << " at " << peak.first << ',' << peak.second << '\n';
    for (auto valley : pv.second)
        std::cout << "valley: " << a[valley.first][valley.second] << " at " << valley.first << ',' << valley.second << '\n';

    std::cout << std::endl;

    std::vector<std::vector<float>> b = {
            { 25, 58, 63, 23, 21, 34, 21, 50 },
            { 32, 45, 43, 40, 41, 32, 30, 27 },
            { 34, 40, 38, 39, 36, 28, 30, 35 },
            { 40, 45, 42, 48, 32, 34, 29, 32 },
            { 39, 39, 40, 42, 47, 49, 27, 30 },
            { 31, 31, 31, 32, 32, 33, 44, 35 }
    };

    auto pv_b = Peak2D::find_peaks_and_valleys(b, 6, 8, [](float x){ return x; });

    for (auto peak : pv_b.first)
        std::cout << "peak: " << a[peak.first][peak.second] << " at " << peak.first << ',' << peak.second << '\n';
    for (auto valley : pv_b.second)
        std::cout << "valley: " << a[valley.first][valley.second] << " at " << valley.first << ',' << valley.second << '\n';

    return 0;
}
