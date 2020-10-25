//
// Created by Harold on 2020/10/25.
//

#ifndef M_MATH_EXAMPLE_UTILS_H
#define M_MATH_EXAMPLE_UTILS_H

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "../utils/traits.h"

template<typename IT>
inline void PrintIter(const std::string& name, IT const begin, IT const end) {
    std::cout << name << ": ";
    for (IT it = begin; it != end; ++it)
        std::cout << *it << " ";
    std::cout << std::endl;
}

template<typename IT, typename = typename std::enable_if<is_iterator<decltype(std::begin(*std::declval<IT>()))>::value>::type>
inline void PrintIter2(const std::string& name, IT const begin, IT const end) {
    std::cout << name << ": ";
    int i = 0;
    int j;
    for (auto it = begin; it != end; ++it) {
        j = 0;
        for (auto it2 = (*it).begin(); it2 != (*it).end(); ++it2) {
            std::cout << '(' << j << ", " << i << ", "<< *it2 << ')';
            ++j;
        }
        ++i;
    }
    std::cout << std::endl;
}

template<typename IT>
inline bool SaveIter(const std::string& filename, IT const begin, IT const end) {
    std::ofstream f(filename);
    if (!f.is_open())
        return false;
    for (IT it = begin; it != end; ++it)
        f << *it << "\n";
    f << std::endl;
    f.close();
    return true;
}

template<typename IT, typename = typename std::enable_if<is_iterator<decltype(std::begin(*std::declval<IT>()))>::value>::type>
inline bool SaveIter2(const std::string& filename, IT const begin, IT const end) {
    std::ofstream f(filename);
    if (!f.is_open())
        return false;
    int i = 0;
    int j;
    for (auto it = begin; it != end; ++it) {
        j = 0;
        for (auto it2 = (*it).begin(); it2 != (*it).end(); ++it2) {
            f << '(' << j << ", " << i << ", "<< *it2 << ")\n";
            ++j;
        }
        ++i;
    }
    f << std::endl;
    f.close();
    return true;
}

template<typename T, size_t N>
inline void PrintArray(const std::string& name, const std::array<T, N>& arr) {
    std::cout << name << ": ";
    for (const auto &e : arr)
        std::cout << e << " ";
    std::cout << std::endl;
}

template<typename T, size_t M, size_t N>
inline void PrintArray(const std::string& name, const std::array<std::array<T, M>, N>& arr) {
    std::cout << name << ": ";
    for (auto i = 0; i < N; i++)
        for (auto j = 0; j < M; j++)
            std::cout << "(" << j << ", " << i << ", "<< arr[j][i] << ")";
    std::cout << std::endl;
}

template<typename T, size_t N>
inline bool SaveArray(const std::string& filename, const std::array<T, N>& arr) {
    std::ofstream f(filename);
    if (!f.is_open())
        return false;
    for (const auto &e : arr)
        f << e << "\n";
    f << std::endl;
    f.close();
    return true;
}

template<typename T, size_t M, size_t N>
inline bool SaveArray(const std::string& filename, const std::array<std::array<T, M>, N>& arr) {
    std::ofstream f(filename);
    if (!f.is_open())
        return false;
    for (auto i = 0; i < N; i++)
        for (auto j = 0; j < M; j++)
            f << "(" << j << ", " << i << ", "<< arr[j][i] << ")\n";
    f << std::endl;
    f.close();
    return true;
}

template<std::size_t OFFSET, std::size_t K, typename T, std::size_t N>
inline std::array<T, K> subarray(const std::array<T, N>& a)
{
    static_assert(OFFSET+K<=N, "invalid sub-array length");

    std::array<T, K> s;
    std::copy(a.begin()+OFFSET, a.begin()+OFFSET+K, s.begin());
    return s;
}

#endif //M_MATH_EXAMPLE_UTILS_H
