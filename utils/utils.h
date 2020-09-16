//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_UTILS_H
#define M_MATH_UTILS_H

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <type_traits>

template<typename T>
using EnableIfFloatingPoint = typename std::enable_if<std::is_floating_point<T>::value>::type;

template<typename ...>
struct make_void { using type = void; };
template<typename ...Ts>
using void_t = typename make_void<Ts ...>::type;

template<typename T, typename = void>
struct is_iterable : std::false_type {};
template<typename T>
struct is_iterable<T,
        void_t<decltype(std::begin(std::declval<T>()) != std::end(std::declval<T>())),
                decltype(++std::declval<decltype(std::begin(std::declval<T>()))>()),
                decltype(*std::begin(std::declval<T>()))
        >>
        : std::true_type {};

template<typename T, typename = void>
struct is_iterator : std::false_type {};
template<typename T>
struct is_iterator<T,
        typename std::enable_if<!std::is_same<typename std::iterator_traits<T>::value_type, void>::value>::type>
        : std::true_type {};


inline constexpr size_t Odd(size_t N) {
    return N % 2 != 0 ? N : N-1;
}

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
