
//
// Created by Harold on 2021/4/12.
//

#ifndef M_MATH_M_CORRELATION_EIGEN_HPP
#define M_MATH_M_CORRELATION_EIGEN_HPP

#include <type_traits>
#include "m_fft_eigen.hpp"

namespace M_MATH {
    // cross correlation based on FFT
    // FIXME: incorrect compared with the OpenCV one in m_correlation.h
    template<typename DerivedA, typename DerivedB>
    Eigen::Matrix<typename std::common_type<DerivedA, DerivedB>::type::Scalar, Eigen::Dynamic, Eigen::Dynamic>
    XCorrelationFFT(Eigen::MatrixBase<DerivedA> const& A, Eigen::MatrixBase<DerivedB> const& B) {
        /*
        // make copy
        auto a = A.replicate(1, 1).eval();
        auto b = B.replicate(1, 1).eval();
        // compute optimized fft size (power of two)
        auto width = std::pow(2, std::round(0.5 + std::log((double)(std::max(a.cols(), b.cols()))/std::log(2.0))));
        auto height = std::pow(2, std::round(0.5 + std::log((double)(std::max(a.rows(), b.rows()))/std::log(2.0))));
        // zero pad on border
        a.derived().conservativeResizeLike(Eigen::MatrixBase<DerivedA>::Zero(height, width));
        b.derived().conservativeResizeLike(Eigen::MatrixBase<DerivedB>::Zero(height, width));
        // fft
        auto fftA = ForwardFFT(a);
        auto fftB = ForwardFFT(b);
        // multiply complex fourier domain matrices
        auto fd = fftA.cwiseProduct(fftB.conjugate());
        // ifft
        auto xc = BackwardFFT(fd.eval());
        auto xcc = fftshift(xc) / (width * height);
         */

        auto fftA = ForwardFFT(A);
        auto fftB = ForwardFFT(B);
        auto fd = fftA.cwiseProduct(fftB.conjugate());
        auto xc = BackwardFFT(fd.eval());
        return fftshift(xc) / (A.rows() * A.cols());
    }

    // cross correlation based on covariance
    // https://www.wikiwand.com/en/Covariance_and_correlation
    // FIXME: incorrect compared with the OpenCV one in m_correlation.h
    template<typename DerivedA, typename DerivedB>
    Eigen::Matrix<typename std::common_type<DerivedA, DerivedB>::type::Scalar, Eigen::Dynamic, Eigen::Dynamic>
    XCorrelation(Eigen::MatrixBase<DerivedA> const& A, Eigen::MatrixBase<DerivedB> const& B) {
        // make copy
        auto a = A.replicate(1, 1).eval();
        auto b = B.replicate(1, 1).eval();

        // compute degrees of freedom
        // n - 1 is the unbiased estimate
        const int df = a.rows() - 1;

        // center matrices
        b.rowwise() -= b.colwise().mean();
        a.rowwise() -= a.colwise().mean();

        // covariance matrix
        Eigen::Matrix<typename std::common_type<DerivedA, DerivedB>::type::Scalar, Eigen::Dynamic, Eigen::Dynamic>
                cor = a.transpose() * b / df;

        // compute 1 over the standard deviations of X and Y
        Eigen::Matrix<typename std::common_type<DerivedA, DerivedB>::type::Scalar, Eigen::Dynamic, 1>
                inv_sds_X = (a.colwise().norm()/std::sqrt(df)).array().inverse();
        Eigen::Matrix<typename std::common_type<DerivedA, DerivedB>::type::Scalar, Eigen::Dynamic, 1>
                inv_sds_Y = (b.colwise().norm()/std::sqrt(df)).array().inverse();

        // scale the covariance matrix
        cor = cor.cwiseProduct(inv_sds_X * inv_sds_Y.transpose());
        return cor;
    }

    // TODO: study
    //  https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0203434
    //  https://stackoverflow.com/questions/48641648/different-results-of-normxcorr2-and-normxcorr2-mex
}

#endif //M_MATH_M_CORRELATION_EIGEN_HPP
