//
// Created by Harold on 2021/5/13.
//

#ifndef M_MATH_M_CONVOLUTION_H
#define M_MATH_M_CONVOLUTION_H

#include <cmath>
#include <vector>

#define USE_FFT

#ifdef USE_FFT
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#endif

namespace M_MATH{
    // full convolution (Cauthy product f * g = g * f)
    // res size = f.size() + g.size() - 1
    template<typename T>
    std::vector<T> Convolution1DFull(std::vector<T> const& f, std::vector<T> const& g) {
        auto const nf = f.size();
        auto const ng = g.size();
        auto const n = nf + ng - 1;
        std::vector<T> res(n, T{});
        for (auto i = 0; i < n; i++) {
            auto const jmin = (i >= ng - 1) ? i - (ng - 1) : 0;
            auto const jmax = (i < nf - 1) ? i : nf - 1;
            for (auto j = jmin; j <= jmax; j++)
                res[i] += (f[j] * g[i - j]);
        }
        return res;
    }

    // same convolution
    // res size = g.size()
    template<typename T>
    std::vector<T> Convolution1DSame(std::vector<T> const& f, std::vector<T> const& g) {
        auto full = Convolution1DFull(f, g);
        std::vector<T> res(full.begin() + f.size() / 2, full.begin() + f.size() / 2 + g.size());
        return res;
    }

    // valid convolution
    // res size = std::max(f.size(), g.size()) - std::min(f.size(), g.size()) + 1
    template<typename T>
    std::vector<T> Convolution1DValid(std::vector<T> const& f, std::vector<T> const& g) {
        auto const nf = f.size();
        auto const ng = g.size();
        std::vector<T> const& min_len_vec = (nf < ng) ? f : g;
        std::vector<T> const& max_len_vec = (nf < ng) ? g : f;
        auto const n = std::max(nf, ng) - std::min(nf, ng) + 1;
        std::vector<T> res(n, T{});
        for (auto i = 0; i < n; i++)
            for (int j = min_len_vec.size() - 1, k = i; j >= 0; --j, ++k)
                res[i] += min_len_vec[j] * max_len_vec[k];
        return res;        
    }

    template<typename T>
    std::vector<T> Convolution1DFullFFT(std::vector<T> const& f, std::vector<T> const& g) {
        auto const nf = f.size();
        auto const ng = g.size();
        auto const n = nf + ng - 1;
        std::vector<T> res(n, T{});
        // zero pad
        std::vector<T> f_(n, T{}), g_(n, T{});
        std::copy(f.begin(), f.end(), f_.begin());
        std::copy(g.begin(), g.end(), g_.begin());
        // fft
        Eigen::FFT<T> fft;
        std::vector<std::complex<T>> tmp(n), tmp_f(n), tmp_g(n);
        fft.fwd(tmp_f, f_);
        fft.fwd(tmp_g, g_);
        // element multiply
        for (auto i = 0; i < n; i++)
            tmp[i] = tmp_f[i] * tmp_g[i];
        // inverse fft
        fft.inv(res, tmp);

        return res;
    }
} // namespace M_MATH

#endif //M_MATH_M_CONVOLUTION_H