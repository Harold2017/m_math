//
// Created by Harold on 2021/4/11.
//

#ifndef M_MATH_M_FFT_EIGEN_HPP
#define M_MATH_M_FFT_EIGEN_HPP

#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/FFT>
#include <complex>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "m_circshift_eigen.hpp"

namespace M_MATH {
    // 2D fft, requires fft shift
    template<typename Derived>
    Eigen::Matrix<std::complex<typename Derived::Scalar>, Eigen::Dynamic, Eigen::Dynamic>
    ForwardFFT(Eigen::MatrixBase<Derived> const& mat) {
        auto rows = mat.rows();
        auto cols = mat.cols();
        Eigen::FFT<typename Derived::Scalar> fft;
        Eigen::Matrix<std::complex<typename Derived::Scalar>, Eigen::Dynamic, Eigen::Dynamic> res(rows, cols);
        // need apply 1d fft on both rows and cols
        for (auto i = 0; i < rows; ++i) {
            Eigen::Matrix<std::complex<typename Derived::Scalar>, Eigen::Dynamic, 1> tmp(cols);
            fft.fwd(tmp, mat.row(i).eval());
            res.row(i) = tmp;
        }
        for (auto i = 0; i < cols; ++i) {
            Eigen::Matrix<std::complex<typename Derived::Scalar>, 1, Eigen::Dynamic> tmp(rows);
            fft.fwd(tmp, res.col(i).eval());
            res.col(i) = tmp;
        }
        return res;
    }

    // OMP one is slower according to benchmark result (see bench/fft_bench.cpp)
    template<typename Derived>
    Eigen::Matrix<std::complex<typename Derived::Scalar>, Eigen::Dynamic, Eigen::Dynamic>
    ForwardFFT_OMP(Eigen::MatrixBase<Derived> const& mat) {
        auto rows = mat.rows();
        auto cols = mat.cols();
        Eigen::FFT<typename Derived::Scalar> fft;
        Eigen::Matrix<std::complex<typename Derived::Scalar>, Eigen::Dynamic, Eigen::Dynamic> res(rows, cols);
        // need apply 1d fft on both rows and cols
#pragma omp parallel sections
        {
#pragma omp section
            {
#pragma omp parallel for if (rows > 10) schedule(static)
                for (auto i = 0; i < rows; ++i) {
                    Eigen::Matrix<std::complex<typename Derived::Scalar>, Eigen::Dynamic, 1> tmp(cols);
                    fft.fwd(tmp, mat.row(i).eval());
                    res.row(i) = tmp;
                }
#pragma omp parallel for if (cols > 10) schedule(static)
                for (auto i = 0; i < cols; ++i) {
                    Eigen::Matrix<std::complex<typename Derived::Scalar>, 1, Eigen::Dynamic> tmp(rows);
                    fft.fwd(tmp, res.col(i).eval());
                    res.col(i) = tmp;
                }
            }
        }
        return res;
    }

    template <typename XprType, typename RowShift, typename ColShift>
    Eigen::CircShiftedView<XprType, RowShift, ColShift> circShift(Eigen::DenseBase<XprType>& x, RowShift r, ColShift c)
    {
        return Eigen::CircShiftedView<XprType, RowShift, ColShift>(x.derived(), r, c);
    }

    template <typename XprType>
    Eigen::CircShiftedView<XprType, Eigen::Index, Eigen::Index> fftshift(Eigen::DenseBase<XprType>& x)
    {
        Eigen::Index rs = x.rows() / 2;
        Eigen::Index cs = x.cols() / 2;
        return Eigen::CircShiftedView<XprType, Eigen::Index, Eigen::Index>(x.derived(), rs, cs);
    }

    template <typename XprType>
    Eigen::CircShiftedView<XprType, Eigen::Index, Eigen::Index> ifftshift(Eigen::DenseBase<XprType>& x)
    {
        Eigen::Index rs = (x.rows() + 1) / 2;
        Eigen::Index cs = (x.cols() + 1) / 2;
        return Eigen::CircShiftedView<XprType, Eigen::Index, Eigen::Index>(x.derived(), rs, cs);
    }
}

#endif //M_MATH_M_FFT_EIGEN_HPP
