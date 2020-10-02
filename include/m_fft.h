//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_M_FFT_H
#define M_MATH_M_FFT_H

#include <fftw3.h>
#include <algorithm>
#include <complex>

namespace M_MATH {
    class FFT1D {
    public:
        static bool fft(double *pReal, double *pImg, int length) {
            return _fft(pReal, pImg, length, FFTW_FORWARD);
        }
        // inverse fft
        static bool ifft(double *pReal, double *pImg, int length) {
            return _fft(pReal, pImg, length, FFTW_BACKWARD);
        }
        // real fft, real number -> complex number
        // double[N]
        static bool rfft(double *pOrigin, double* pReal, double* pImg, int length);
        // double[N]
        static bool irfft(double *pOrigin, double const* pReal, double const* pImg, int length);
    private:
        // in-place
        // if img part all == 0, it becomes dft
        // length: array length
        static bool _fft(double *pReal, double *pImg, int length, int direction);
    };

    class FFT2D {
    public:
        // double[Nx * Ny]
        // Nx * Ny -> Nx * (Ny / 2 + 1) complex
        static bool rfft(double* pOrigin, double* pReal, double* pImg, int length_x, int length_y);
        static bool rfft(double* pOrigin, std::complex<double>* pComplex, int length_x, int length_y);
        // double[Nx * Ny]
        static bool irfft(double *pOrigin, double const* pReal, double const* pImg, int length_x, int length_y);
        static bool irfft(double *pOrigin, std::complex<double>* pComplex, int length_x, int length_y);
    };

    bool FFT2D::rfft(double *pOrigin, double *pReal, double *pImg, int length_x, int length_y) {
        auto fft_size = length_x * (length_y/2+1);
        auto pOut = fftw_alloc_complex(fft_size);
        if (pOut == nullptr) {
            return false;
        }
        auto f_plan = fftw_plan_dft_r2c_2d(length_x, length_y, pOrigin, pOut, FFTW_ESTIMATE);
        if (f_plan == nullptr) {
            fftw_free(pOut);
            return false;
        }
        fftw_execute(f_plan);
        for (auto i = 0; i < fft_size; ++i) {
            pReal[i] = pOut[i][0];
            pImg[i] = pOut[i][1];
        }
        fftw_destroy_plan(f_plan);
        fftw_free(pOut);
        return true;
    }

    bool FFT2D::irfft(double *pOrigin, const double *pReal, const double *pImg, int length_x, int length_y) {
        auto fft_size = length_x * (length_y/2+1);
        auto pIn = fftw_alloc_complex(fft_size);
        if (pIn == nullptr) {
            return false;
        }
        auto N = length_x * length_y;
        for (auto i = 0; i < fft_size; ++i) {
            pIn[i][0] = pReal[i] / N;
            pIn[i][1] = pImg[i] / N;
        }
        auto f_plan = fftw_plan_dft_c2r_2d(length_x, length_y, pIn, pOrigin, FFTW_ESTIMATE);
        if (f_plan == nullptr) {
            fftw_free(pIn);
            return false;
        }
        fftw_execute(f_plan);
        fftw_destroy_plan(f_plan);
        fftw_free(pIn);
        return true;
    }

    bool FFT2D::rfft(double *pOrigin, std::complex<double> *pComplex, int length_x, int length_y) {
        auto fft_size = length_x * (length_y/2+1);
        auto pOut = fftw_alloc_complex(fft_size);
        if (pOut == nullptr) {
            return false;
        }
        auto f_plan = fftw_plan_dft_r2c_2d(length_x, length_y, pOrigin, pOut, FFTW_ESTIMATE);
        if (f_plan == nullptr) {
            fftw_free(pOut);
            return false;
        }
        fftw_execute(f_plan);
        memcpy(pComplex, pOut, sizeof(fftw_complex) * fft_size);
        fftw_destroy_plan(f_plan);
        fftw_free(pOut);
        return true;
    }

    bool FFT2D::irfft(double *pOrigin, std::complex<double> *pComplex, int length_x, int length_y) {
        auto fft_size = length_x * (length_y/2+1);
        auto pIn = fftw_alloc_complex(fft_size);
        if (pIn == nullptr) {
            return false;
        }
        auto N = length_x * length_y;
        for (auto i = 0; i < fft_size; ++i) {
            pIn[i][0] = std::real(pComplex[i]) / N;
            pIn[i][1] = std::imag(pComplex[i]) / N;
        }
        auto f_plan = fftw_plan_dft_c2r_2d(length_x, length_y, pIn, pOrigin, FFTW_ESTIMATE);
        if (f_plan == nullptr) {
            fftw_free(pIn);
            return false;
        }
        fftw_execute(f_plan);
        fftw_destroy_plan(f_plan);
        fftw_free(pIn);
        return true;
    }

    inline bool FFT1D::_fft(double *pReal, double *pImg, int length, int direction) {
        auto pIn = fftw_alloc_complex(length);
        if (pIn == nullptr) {
            return false;
        }
        auto pOut = fftw_alloc_complex(length);
        if (pOut == nullptr) {
            fftw_free(pIn);
            return false;
        }
        for (auto i = 0; i < length; i++) {
            pIn[i][0] = pReal[i];
            pIn[i][1] = pImg[i];
        }
        auto f_plan = fftw_plan_dft_1d(length, pIn, pOut, direction, FFTW_ESTIMATE);
        if (f_plan == nullptr) {
            fftw_free(pIn);
            fftw_free(pOut);
            return false;
        }
        fftw_execute(f_plan);
        for (auto i = 0; i < length; i++) {
            pReal[i] = direction == FFTW_FORWARD ? pOut[i][0] : pOut[i][0] / length;
            pImg[i] = direction == FFTW_FORWARD ? pOut[i][1] : pOut[i][1] / length;
        }
        fftw_destroy_plan(f_plan);
        fftw_free(pIn);
        fftw_free(pOut);
        return true;
    }

    bool FFT1D::rfft(double *pOrigin, double* pReal, double* pImg, int length) {
        auto fft_size = length % 2 == 0 ? length/2 : length/2+1;
        auto pOut = fftw_alloc_complex(fft_size);
        if (pOut == nullptr) {
            return false;
        }
        auto f_plan = fftw_plan_dft_r2c_1d(length, pOrigin, pOut, FFTW_ESTIMATE);
        if (f_plan == nullptr) {
            fftw_free(pOut);
            return false;
        }
        fftw_execute(f_plan);
        for (auto i = 0; i < fft_size; ++i) {
            pReal[i] = pOut[i][0];
            pImg[i] = pOut[i][1];
        }
        fftw_destroy_plan(f_plan);
        fftw_free(pOut);
        return true;
    }

    bool FFT1D::irfft(double *pOrigin, const double *pReal, const double *pImg, int length) {
        auto fft_size = length % 2 == 0 ? length/2 : length/2+1;
        auto pIn = fftw_alloc_complex(fft_size);
        if (pIn == nullptr)
            return false;
        for (auto i = 0; i < fft_size; ++i) {
            pIn[i][0] = pReal[i] / length;
            pIn[i][1] = pImg[i] / length;
        }
        auto f_plan = fftw_plan_dft_c2r_1d(length, pIn, pOrigin, FFTW_ESTIMATE);
        if (f_plan == nullptr) {
            fftw_free(pIn);
            return false;
        }
        fftw_execute(f_plan);
        fftw_destroy_plan(f_plan);
        fftw_free(pIn);
        return true;
    }
}

#endif //M_MATH_M_FFT_H
