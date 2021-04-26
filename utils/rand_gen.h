//
// Created by Harold on 2021/4/26.
//

#ifndef M_MATH_RAND_GEN_H
#define M_MATH_RAND_GEN_H

#include <random>

/**
 * @brief generate normal distributed random data
 * 
 * @tparam T 
 * @return T 
 */
template<typename T>
T normal_gen(T mean, T sigma) {
    static std::random_device dev;
    static std::mt19937 rng(dev());
    static std::normal_distribution<T> dist(mean, sigma);
    return dist(rng);
}

#endif //M_MATH_RAND_GEN_H