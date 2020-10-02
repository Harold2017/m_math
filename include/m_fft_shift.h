//
// Created by Harold on 2020/10/2.
//

#ifndef M_MATH_M_FFT_SHIFT_H
#define M_MATH_M_FFT_SHIFT_H

// code from: https://stackoverflow.com/questions/5915125/fftshift-ifftshift-c-c-source-code

#include <complex>
#include <algorithm>
#include <vector>

namespace M_MATH {
    // T = std::complex<Real>
    template <typename T>
    static inline
    void fft_shift_1d(T *data, size_t const ydim)
    {
        auto center = ydim / 2;
        if (ydim % 2 != 0) {
            center++;
        }
        // odd: 012 34 changes to 34 012
        std::rotate(data, data + center, data + ydim);
    }

    template <typename T>
    static inline
    void ifft_shift_1d(T *data, size_t const ydim)
    {
        auto center = ydim / 2;
        // odd: 01 234 changes to 234 01
        std::rotate(data, data + center, data + ydim);
    }

    template<typename T>
    static inline
    void fft_shift_2d(T *data, size_t xdim, size_t ydim)
    {
        size_t xshift = xdim / 2;
        size_t yshift = ydim / 2;
        if ((xdim*ydim) % 2 != 0) {
            // temp output array
            std::vector<T> out;
            out.resize(xdim * ydim);
            for (size_t x = 0; x < xdim; x++) {
                size_t outX = (x + xshift) % xdim;
                for (size_t y = 0; y < ydim; y++) {
                    size_t outY = (y + yshift) % ydim;
                    // row-major order
                    out[outX + xdim * outY] = data[x + xdim * y];
                }
            }
            // copy out back to data
            copy(out.begin(), out.end(), &data[0]);
        }
        else {
            // in and output array are the same,
            // values are exchanged using std::swap
            for (size_t x = 0; x < xdim; x++) {
                size_t outX = (x + xshift) % xdim;
                for (size_t y = 0; y < yshift; y++) {
                    size_t outY = (y + yshift) % ydim;
                    // row-major order
                    std::swap(data[outX + xdim * outY], data[x + xdim * y]);
                }
            }
        }
    }

    template<class T>
    static inline
    void ifft_shift_2d(T *data, size_t xdim, size_t ydim)
    {
        size_t xshift = xdim / 2;
        if (xdim % 2 != 0) {
            xshift++;
        }
        size_t yshift = ydim / 2;
        if (ydim % 2 != 0) {
            yshift++;
        }
        if ((xdim*ydim) % 2 != 0) {
            // temp output array
            std::vector<T> out;
            out.resize(xdim * ydim);
            for (size_t x = 0; x < xdim; x++) {
                size_t outX = (x + xshift) % xdim;
                for (size_t y = 0; y < ydim; y++) {
                    size_t outY = (y + yshift) % ydim;
                    // row-major order
                    out[outX + xdim * outY] = data[x + xdim * y];
                }
            }
            // copy out back to data
            copy(out.begin(), out.end(), &data[0]);
        }
        else {
            // in and output array are the same,
            // values are exchanged using std::swap
            for (size_t x = 0; x < xdim; x++) {
                size_t outX = (x + xshift) % xdim;
                for (size_t y = 0; y < yshift; y++) {
                    size_t outY = (y + yshift) % ydim;
                    // row-major order
                    std::swap(data[outX + xdim * outY], data[x + xdim * y]);
                }
            }
        }
    }
}

#endif //M_MATH_M_FFT_SHIFT_H
