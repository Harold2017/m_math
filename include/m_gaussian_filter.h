//
// Created by Harold on 2021/5/12.
//

#ifndef M_MATH_M_GAUSSIAN_FILTER_H
#define M_MATH_M_GAUSSIAN_FILTER_H

#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

namespace M_MATH {
    static const double alpha = sqrt(log(2)/M_PI);

    template<typename T>
    std::vector<T> GaussianKernel(int kernel_size, T lambdac, T dx) {
        std::vector<T> kernel;
        kernel.reserve(kernel_size);
        T const constant = lambdac * alpha;
        int kernel_size_half = kernel_size/2;
        for (int i = -kernel_size_half; i <= kernel_size_half; i++) {
            kernel[i + kernel_size_half] = 1 / constant * exp(-M_PI * pow(i * dx / constant, 2));
        }
        return kernel;
    }

    template<typename T>
    std::vector<T> Convolution1D(std::vector<T> const& kernel, std::vector<T> const& data_in, T dx) {
        int data_size = data_in.size();
        int kernel_size = kernel.size();
        int data_max_idx = data_size - 1;
        std::vector<T> data_out;
        data_out.reserve(data_size);
        for (int i = 0; i < data_size; i++) {
            int ix = i + kernel_size / 2;
            T tmp{};
            for (int j = 0; j < kernel_size; j++) {
                int idx = ix;
                idx = idx > data_max_idx ? data_max_idx : idx;
                idx = idx < 0 ? 0 : idx;
                tmp += dx * data_in[idx] * kernel[j];
                --ix;
            }
            data_out.push_back(tmp);
        }
        return data_out;
    }

    // new kernel for end effect (2nd order)
    template<typename T>
    std::vector<T> _nk(int idx, int kernel_size, T lambdac, T dx) {
        T const constant = lambdac * alpha;
        T u0{}, u1{}, u2{};
        std::vector<T> n_kernel;
        n_kernel.reserve(kernel_size);

        for (int i = 0; i < kernel_size; i++) {
            T xi = (i - idx) * dx;
            T sxi = exp(-M_PI * pow(xi / constant, 2)) / constant;
            u0 += sxi;
            u1 += xi * sxi;
            u2 += xi * xi * sxi;
        }
        u0 *= dx;
        u1 *= dx;
        u2 *= dx;

        T det = u0 * u2 - u1 * u1;
        T b0 = u2 / det;
        T b1 = -u1 / det;

        for (int i = 0; i < kernel_size; i++) {
            T xi = (i - idx) * dx;
            T sxi = exp(-M_PI * pow(xi / constant, 2)) / constant;
            n_kernel.push_back(b0 * sxi + b1 * xi * sxi);
        }
        return n_kernel;
    }

    template<typename T>
    std::vector<T> Convolution1DWithEndEffect(std::vector<T> const& kernel, std::vector<T> const& data_in, T lambdac,T dx) {
        auto data_out = Convolution1D(kernel, data_in, dx);
        int kernel_size = kernel.size();
        int data_size = data_in.size();
        if (kernel_size >= data_size) return data_out;
        // handle end effect
        for (int i = 0; i <= kernel_size/2; i++) {
            auto nk = _nk(i, kernel_size, lambdac, dx);
            // left
            T tmp{};
            for (int j = 0; j < kernel_size; j++)
                tmp += dx * (data_in[j] * nk[j]);
            data_out[i] = tmp;
            // right
            tmp = T{};
            for (int j = 0; j < kernel_size; j++)
                tmp += dx * (data_in[data_size-1-j] * nk[j]);
            data_out[data_size-1-i] = tmp;
        }
        return data_out;
    }
}

#endif //M_MATH_M_GAUSSIAN_FILTER_H