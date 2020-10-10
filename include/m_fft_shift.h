//
// Created by Harold on 2020/10/2.
//

#ifndef M_MATH_M_FFT_SHIFT_H
#define M_MATH_M_FFT_SHIFT_H

#include <complex>
#include <algorithm>
#include <vector>

namespace M_MATH {
    // FFT shift
    // Note: in-place change
    class FFT_SHIFT {
    public:
        template<typename T>
        static void fft_shift_1d(T *data, size_t ydim);
        template<typename T>
        static void ifft_shift_1d(T *data, size_t ydim);

        template<typename T>
        static void fft_shift_2d(T *data, size_t xdim, size_t ydim);
        template<typename T>
        static void ifft_shift_2d(T *data, size_t xdim, size_t ydim);

        // helper functions: transfer fftw half spectrum to full spectrum
        // xdim, ydim are original data dims
        template<typename T>
        static void half2full_1d(std::complex<T>* fft_data, std::complex<T>* out_data, size_t ydim);
        template<typename T>
        static void half2full_2d(std::complex<T>* fft_data, std::complex<T>* out_data, size_t xdim, size_t ydim);
    };


    template<typename T>
    void FFT_SHIFT::fft_shift_1d(T *data, size_t ydim) {
        auto center = ydim / 2;
        if (ydim % 2 != 0) {
            center++;
        }
        std::rotate(data, data + center, data + ydim);
    }

    template<typename T>
    void FFT_SHIFT::ifft_shift_1d(T *data, size_t ydim) {
        auto center = ydim / 2;
        std::rotate(data, data + center, data + ydim);
    }

    template<typename T>
    void FFT_SHIFT::fft_shift_2d(T *data, size_t xdim, size_t ydim) {
        size_t xshift = xdim / 2;
        size_t yshift = ydim / 2;
        if ((xdim*ydim) % 2 != 0) {
            std::vector<T> out(xdim * ydim);
            for (size_t x = 0; x < xdim; x++) {
                size_t outX = (x + xshift) % xdim;
                for (size_t y = 0; y < ydim; y++) {
                    size_t outY = (y + yshift) % ydim;
                    // row-major order
                    out[outX + xdim * outY] = data[x + xdim * y];
                }
            }
            std::copy(out.begin(), out.end(), &data[0]);
        }
        else {
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

    template<typename T>
    void FFT_SHIFT::ifft_shift_2d(T *data, size_t xdim, size_t ydim) {
        size_t xshift = xdim / 2;
        if (xdim % 2 != 0) {
            xshift++;
        }
        size_t yshift = ydim / 2;
        if (ydim % 2 != 0) {
            yshift++;
        }
        if ((xdim*ydim) % 2 != 0) {
            std::vector<T> out(xdim * ydim);
            for (size_t x = 0; x < xdim; x++) {
                size_t outX = (x + xshift) % xdim;
                for (size_t y = 0; y < ydim; y++) {
                    size_t outY = (y + yshift) % ydim;
                    // row-major order
                    out[outX + xdim * outY] = data[x + xdim * y];
                }
            }
            std::copy(out.begin(), out.end(), &data[0]);
        }
        else {
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

    template<typename T>
    void FFT_SHIFT::half2full_1d(std::complex<T>* fft_data, std::complex<T>* out_data, size_t ydim){
        auto fft_size = ydim % 2 == 0 ? ydim/2 : ydim/2+1;
        for (auto i = 0; i < fft_size; ++i)
            out_data[i] = fft_data[i];
        for (auto i = fft_size; i < ydim; ++i)
            out_data[i] = std::conj(fft_data[ydim-1 - i]);
    }

    template<typename T>
    void FFT_SHIFT::half2full_2d(std::complex<T> *fft_data, std::complex<T> *out_data, size_t xdim, size_t ydim) {
        // FIXME: not implement yet
        throw;
    }
}

#endif //M_MATH_M_FFT_SHIFT_H
