//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_UTILS_H
#define M_MATH_UTILS_H

#include <vector>

inline constexpr size_t Odd(size_t N) {
    return N % 2 != 0 ? N : N-1;
}

inline int quick_pow10(int n)
{
    static int pow10[10] = {
            1, 10, 100, 1000, 10000,
            100000, 1000000, 10000000, 100000000, 1000000000
    };
    return pow10[n];
}

inline double ToFixedDecimal(double d, int n) {
    int i;
    if (d >= 0)
        i = static_cast<int>(d * quick_pow10(n) + 0.5);
    else
        i = static_cast<int>(d * quick_pow10(n) - 0.5);
    return (i / 100.0);
}

template<typename T>
inline std::vector<double> linspace(T start, T end, unsigned int num) {
    std::vector<double> ls;
    auto s = static_cast<double>(start);
    auto e = static_cast<double>(end);
    if (num == 0) { return ls; }
    if (num == 1) { ls.push_back(start); return ls; }
    double delta = (e - s) / double(num - 1);
    for (auto i = 0; i < num - 1; ++i)
        ls.push_back(start + delta * i);
    ls.push_back(end);
    return ls;
}

#endif //M_MATH_UTILS_H
