//
// Created by Harold on 2020/10/2.
//

#ifndef M_MATH_M_AUTOC_H
#define M_MATH_M_AUTOC_H

#include "m_fft.h"
#include "m_fft_shift.h"

namespace M_MATH {
    // ACF = abs(fftshift(ifft2(fft2(H).*conj(fft2(H)))))./(n*m);
    static inline
    void auto_corr_2d(double *pOrigin, double *pAc, int xdim, int ydim) {
        auto fft_size = xdim * (ydim/2+1);
        std::vector<std::complex<double>> vCpx(fft_size);
        FFT2D::rfft(pOrigin, vCpx.data(), xdim, ydim);
        for (auto i = 0; i < fft_size; ++i)
            vCpx[i] = std::abs(vCpx[i]) * std::abs(vCpx[i]);
        FFT2D::irfft(pAc, vCpx.data(), xdim, ydim);
        fft_shift_2d(pAc, xdim, ydim);
        auto N = xdim * ydim;
        for (auto i = 0; i < N; ++i)
            pAc[i] /= N;
    }
}

#endif //M_MATH_M_AUTOC_H
