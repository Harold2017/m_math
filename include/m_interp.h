//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_M_INTERP_H
#define M_MATH_M_INTERP_H

#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_interp2d.h>
#include <gsl/gsl_spline2d.h>
#include <vector>
#include <algorithm>

namespace M_MATH {
    class Interpolation1D {
    public:
        enum InterpType {
            LINEAR,
            POLYNOMIAL,
            CSPLINE,
            CSPLINE_PERIODIC,
            AKIMA,
            AKIMA_PERIODIC,
            STEFFEN
        };

        static bool interpolate(double const* x,
                                double const* y,
                                size_t length,
                                InterpType type,
                                double const* n_x,
                                size_t n_length,
                                double* n_y);

        // this will create tmp var inside, better to use former one if possible
        template<typename IT_X, typename IT_Y, typename IT_NX, typename IT_NY>
        static bool interpolate(IT_X x_begin, IT_X x_end,
                                IT_Y y_begin, IT_Y y_end,
                                InterpType type,
                                IT_NX nx_begin, IT_NX nx_end,
                                IT_NY ny_begin);

    private:
        static const gsl_interp_type* InterpType2GSL(InterpType type);
    };

    class Interpolation2D {
    public:
        enum InterpType {
            BI_LINEAR,
            BI_CUBIC
        };

        static bool interpolate(double const* x,
                                double const* y,
                                double const* z,
                                size_t x_length,
                                size_t y_length,
                                InterpType type,
                                double const* n_x,
                                double const* n_y,
                                size_t n_x_length,
                                size_t n_y_length,
                                double* n_z);

        // this will create tmp var inside, better to use former one if possible
        template<typename IT_X, typename IT_Y, typename IT_Z, typename IT_NX, typename IT_NY, typename IT_NZ>
        static bool interpolate(IT_X x_begin, IT_X x_end,
                                IT_Y y_begin, IT_Y y_end,
                                IT_Z z_begin, IT_Z z_end,
                                InterpType type,
                                IT_NX nx_begin, IT_NX nx_end,
                                IT_NY ny_begin, IT_NY ny_end,
                                IT_NZ nz_begin);

    private:
        static const gsl_interp2d_type* InterpType2GSL(InterpType type);
    };

    const gsl_interp_type *Interpolation1D::InterpType2GSL(Interpolation1D::InterpType type) {
        switch (type) {
            case LINEAR:
                return gsl_interp_linear;
            case POLYNOMIAL:
                return gsl_interp_polynomial;
            case CSPLINE:
                return gsl_interp_cspline;
            case CSPLINE_PERIODIC:
                return gsl_interp_cspline_periodic;
            case AKIMA:
                return gsl_interp_akima;
            case AKIMA_PERIODIC:
                return gsl_interp_akima_periodic;
            case STEFFEN:
                return gsl_interp_steffen;
        }
        return nullptr;
    }

    bool Interpolation1D::interpolate(const double *x,
                                      const double *y,
                                      size_t length,
                                      Interpolation1D::InterpType type,
                                      const double *n_x,
                                      size_t n_length,
                                      double* n_y) {
        gsl_spline* spl = gsl_spline_alloc(InterpType2GSL(type), length);
        if (spl == nullptr)
            return false;
        gsl_interp_accel* acc = gsl_interp_accel_alloc();
        if (acc == nullptr) {
            return false;
        }
        gsl_spline_init(spl, x, y, length);
        for (auto i = 0; i < n_length; ++i)
            n_y[i] = gsl_spline_eval(spl, n_x[i], acc);
        gsl_spline_free(spl);
        gsl_interp_accel_free(acc);
        return true;
    }

    template<typename IT_X, typename IT_Y, typename IT_NX, typename IT_NY>
    bool Interpolation1D::interpolate(const IT_X x_begin, const IT_X x_end,
                                      const IT_Y y_begin, const IT_Y y_end,
                                      Interpolation1D::InterpType type,
                                      const IT_NX nx_begin, const IT_NX nx_end,
                                      IT_NY ny_begin) {
        auto len = std::min(std::distance(x_begin, x_end), std::distance(y_begin, y_end));
        auto n_len = std::distance(nx_begin, nx_end);
        std::vector<double> x_tmp(len), y_tmp(len);
        std::copy(x_begin, x_begin+len, x_tmp.begin());
        std::copy(y_begin, y_begin+len, y_tmp.begin());

        gsl_spline* spl = gsl_spline_alloc(InterpType2GSL(type), len);
        if (spl == nullptr)
            return false;
        gsl_interp_accel* acc = gsl_interp_accel_alloc();
        if (acc == nullptr) {
            return false;
        }
        gsl_spline_init(spl, x_tmp.data(), y_tmp.data(), len);
        for (auto i = 0; i < n_len; ++i)
            *(ny_begin+i) = gsl_spline_eval(spl, *(nx_begin+i), acc);
        gsl_spline_free(spl);
        gsl_interp_accel_free(acc);
        return true;
    }

    const gsl_interp2d_type *Interpolation2D::InterpType2GSL(Interpolation2D::InterpType type) {
        switch (type) {
            case BI_LINEAR:
                return gsl_interp2d_bilinear;
            case BI_CUBIC:
                return gsl_interp2d_bicubic;
        }
        return nullptr;
    }

    bool Interpolation2D::interpolate(const double *x,
                                      const double *y,
                                      const double *z,
                                      size_t x_length,
                                      size_t y_length,
                                      Interpolation2D::InterpType type,
                                      const double *n_x,
                                      const double *n_y,
                                      size_t n_x_length,
                                      size_t n_y_length,
                                      double *n_z) {
        gsl_spline2d* spl = gsl_spline2d_alloc(InterpType2GSL(type), x_length, y_length);
        if (spl == nullptr)
            return false;
        gsl_interp_accel *x_acc = gsl_interp_accel_alloc();
        if (x_acc == nullptr)
            return false;
        gsl_interp_accel *y_acc = gsl_interp_accel_alloc();
        if (y_acc == nullptr)
            return false;
        // zij = z[j * x_length + i]
        gsl_spline2d_init(spl, x, y, z, x_length, y_length);
        for (auto i = 0; i < n_x_length; ++i)
            for (auto j = 0; j < n_y_length; ++j)
                n_z[j * n_x_length + i] = gsl_spline2d_eval(spl, n_x[i], n_y[j], x_acc, y_acc);
        gsl_spline2d_free(spl);
        gsl_interp_accel_free(x_acc);
        gsl_interp_accel_free(y_acc);
        return true;
    }

    template<typename IT_X, typename IT_Y, typename IT_Z, typename IT_NX, typename IT_NY, typename IT_NZ>
    bool Interpolation2D::interpolate(const IT_X x_begin, const IT_X x_end,
                                      const IT_Y y_begin, const IT_Y y_end,
                                      const IT_Z z_begin, const IT_Z z_end,
                                      Interpolation2D::InterpType type,
                                      const IT_NX nx_begin, const IT_NX nx_end,
                                      const IT_NY ny_begin, const IT_NY ny_end,
                                      const IT_NZ nz_begin) {
        auto x_length = std::distance(x_begin, x_end);
        auto y_length = std::distance(y_begin, y_end);
        auto z_length = std::distance(z_begin, z_end);
        auto n_x_length = std::distance(nx_begin, nx_end);
        auto n_y_length = std::distance(ny_begin, ny_end);
        std::vector<double> x_tmp(x_length), y_tmp(y_length), z_tmp(z_length);
        std::copy(x_begin, x_end, x_tmp.begin());
        std::copy(y_begin, y_end, y_tmp.begin());
        std::copy(z_begin, z_end, z_tmp.begin());

        gsl_spline2d* spl = gsl_spline2d_alloc(InterpType2GSL(type), x_length, y_length);
        if (spl == nullptr)
            return false;
        gsl_interp_accel *x_acc = gsl_interp_accel_alloc();
        if (x_acc == nullptr)
            return false;
        gsl_interp_accel *y_acc = gsl_interp_accel_alloc();
        if (y_acc == nullptr)
            return false;
        // zij = z[j * x_length + i]
        gsl_spline2d_init(spl, x_tmp.data(), y_tmp.data(), z_tmp.data(), x_length, y_length);
        for (auto i = 0; i < n_x_length; ++i)
            for (auto j = 0; j < n_y_length; ++j)
                *(nz_begin + j * n_x_length + i) = gsl_spline2d_eval(spl, *(nx_begin + i), *(ny_begin + j), x_acc, y_acc);
        gsl_spline2d_free(spl);
        gsl_interp_accel_free(x_acc);
        gsl_interp_accel_free(y_acc);
        return true;
    }
}

#endif //M_MATH_M_INTERP_H
