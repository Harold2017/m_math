//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_M_CURVEFIT_H
#define M_MATH_M_CURVEFIT_H

#include <map>
#include <gsl/gsl_fit.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <numeric>
#include <vector>
#include <cmath>

namespace M_MATH {
    class CurveFit{
    public:
        enum robust_type {
            DEFAULT,
            BISQUARE,
            CAUCHY,
            FAIR,
            HUBER,
            OLS,
            WELSCH
        };
    public:
        bool is_valid() const { return !m_coefficient.empty(); }
        bool linearfit(double const* x, double const* y, size_t n);
        bool polyfit(double const* x, double const* y, size_t length, unsigned poly_n);
        bool robustfit(double const* x, double const* y, size_t length, robust_type type, unsigned poly_n);

        double getY(double x) const;

        template<typename IT_X, typename IT_Y>
        void getYs(IT_X x_begin, IT_X x_end, IT_Y y_begin) const;

        double get_coefficient(size_t n) const;
        void set_coefficient(size_t n, double c) { m_coefficient[n] = c; };
        size_t get_coefficient_size() const { return m_coefficient.size(); };

        double get_slope() { return m_coefficient[1]; };
        double get_intercept() { return m_coefficient[0]; };
        double get_SSR() const { return m_ssr; };
        double get_SSE() const { return m_sse; };
        double get_SST() const { return m_sst; };
        double get_RMSE() const { return m_rmse; };
        double get_RSquare() const { return m_RSquare; };
        double get_Goodness() const { return m_goodness; };

        void clear() {
            m_coefficient.clear();
            m_err.clear();
        }

    private:
        std::map<size_t, double> m_coefficient;  // <index, fitting coefficient>, [0]: slope, [1] intercept
        std::map<size_t, double> m_err;
        double m_cov;  // covariance
        double m_ssr;  // sum of squares due to regression
        double m_sse;  // sum of squares error
        double m_sst;  // m_sst = m_ssr + m_sse
        double m_rmse;  // RMS error
        double m_RSquare;  // R-Square

        double m_sumsq;  // sum of squares
        double m_goodness;  // fit goodness

        static void get_determinate_parameters(double const* y, double const* yf,
                                               size_t length,
                                               double &ssr,
                                               double &sse,
                                               double &sst,
                                               double &rmse,
                                               double &RSquare) {
            double y_mean = std::accumulate(y, y+length, 0.0);
            ssr = 0.0;
            for (auto i = 0; i < length; ++i) {
                ssr += (yf[i] - y_mean) * (yf[i] - y_mean);
                sse += (y[i] - yf[i]) * (y[i] - yf[i]);
            }
            sst = ssr + sse;
            rmse = std::sqrt(sse/(double)length);
            RSquare = 1.0 - ssr/sst;
        }

        static gsl_multifit_robust_type const* get_robust_type(robust_type type) {
            switch (type) {
                case DEFAULT:
                    return gsl_multifit_robust_default;
                case BISQUARE:
                    return gsl_multifit_robust_bisquare;
                case CAUCHY:
                    return gsl_multifit_robust_cauchy;
                case FAIR:
                    return gsl_multifit_robust_fair;
                case HUBER:
                    return gsl_multifit_robust_huber;
                case OLS:
                    return gsl_multifit_robust_ols;
                case WELSCH:
                    return gsl_multifit_robust_welsch;
            }
            return nullptr;
        }
    };

    inline bool CurveFit::linearfit(const double *x, const double *y, size_t n) {
        clear();
        m_coefficient[0] = 0;
        m_coefficient[1] = 1;
        m_err[0] = 0;
        m_err[0] = 0;
        // double const* x, size_t const x_stride,
        // double const* y, size_t const y_stride,
        // size_t n,
        // double &intercept,
        // double &slope,
        // double &intercept_err,
        // double &slope_err,
        // double &cov,
        // double &wssr
        int ret = gsl_fit_linear(x, 1, y, 1, n,
                                 &m_coefficient[0], &m_coefficient[1],
                                 &m_err[0], &m_err[1],
                                 &m_cov, &m_sumsq);
        if (ret != 0)
            return false;
        // gammaQ function
        m_goodness = gsl_cdf_chisq_Q(m_sumsq / 2.0, double(n - 2) / 2.0);
        {
            std::vector<double> yf(n, 0);
            getYs(x, x+n, yf.begin());
            get_determinate_parameters(y, &yf[0], n, m_ssr, m_sse, m_sst, m_rmse, m_RSquare);
        }
        return true;
    }

    inline bool CurveFit::polyfit(const double *x, const double *y, size_t length, unsigned poly_n) {
        gsl_matrix *Xf = gsl_matrix_alloc(length, poly_n+1);
        gsl_vector *Yf = gsl_vector_alloc(length);
        gsl_vector *c = gsl_vector_alloc(poly_n+1);
        gsl_matrix *cov = gsl_matrix_alloc(poly_n+1, poly_n+1);

        for (auto i = 0; i < length; i++) {
            gsl_matrix_set(Xf, i, 0, 1.0);
            gsl_vector_set(Yf, i, y[i]);
            for (auto j = 1; j < poly_n + 1; j++)
                gsl_matrix_set(Xf, i, j, std::pow(x[i], int(j)));
        }

        auto *workspace = gsl_multifit_linear_alloc(length, poly_n+1);
        double chisq;
        // temporary vec to hold computed coefficients
        std::vector<double> coe(poly_n+1, 0);

        int ret = gsl_multifit_linear(Xf, Yf, c, cov, &chisq, workspace);
        gsl_multifit_linear_free(workspace);
        for (auto i = 0; i < c->size; i++)
            coe[i] = gsl_vector_get(c, i);

        gsl_matrix_free(Xf);
        gsl_vector_free(Yf);
        gsl_vector_free(c);
        gsl_matrix_free(cov);

        // if not successfully computed, not change former results
        if (ret != 0)
            return false;

        // update results
        m_goodness = gsl_cdf_chisq_Q(chisq/2.0, double(length-2)/2.0);
        clear();
        for (auto i = 0; i < poly_n+1; i++)
            m_coefficient[i] = coe[i];
        {
            std::vector<double> yf(length, 0);
            getYs(x, x+length, yf.begin());
            get_determinate_parameters(y, &yf[0], length, m_ssr, m_sse, m_sst, m_rmse, m_RSquare);
        }
        return true;
    }

    inline double CurveFit::getY(double x) const {
        double ret = 0;
        for (auto const& it : m_coefficient)
            ret += (it.second) * std::pow(x, it.first);
        return ret;
    }

    template<typename IT_X, typename IT_Y>
    inline void CurveFit::getYs(IT_X x_begin, IT_X x_end, IT_Y y_begin) const {
        for (; x_begin != x_end; ++x_begin, ++y_begin)
            *y_begin = getY(*x_begin);
    }

    inline double CurveFit::get_coefficient(size_t n) const {
        auto it = m_coefficient.find(n);
        if (it != m_coefficient.end())
            return it->second;
        return 0.0;
    }

    // poly_n >= 2
    // poly_n = 2: linear fit
    bool CurveFit::robustfit(const double *x, const double *y, size_t length, robust_type type, unsigned int poly_n) {
        gsl_matrix *Xf = gsl_matrix_alloc(length, poly_n);
        gsl_vector *Yf = gsl_vector_alloc(length);
        gsl_vector *c = gsl_vector_alloc(poly_n);
        gsl_matrix *cov = gsl_matrix_alloc(poly_n, poly_n);

        for (auto i = 0; i < length; i++) {
            gsl_matrix_set(Xf, i, 0, 1.0);
            gsl_vector_set(Yf, i, y[i]);
            for (auto j = 1; j < poly_n; j++)
                gsl_matrix_set(Xf, i, j, std::pow(x[i], int(j)));
        }

        auto *workspace = gsl_multifit_robust_alloc(get_robust_type(type), length, poly_n);
        // temporary vec to hold computed coefficients
        std::vector<double> coe(poly_n, 0);

        int ret = gsl_multifit_robust(Xf, Yf, c, cov, workspace);
        gsl_multifit_robust_free(workspace);
        for (auto i = 0; i < c->size; i++)
            coe[i] = gsl_vector_get(c, i);

        gsl_matrix_free(Xf);
        gsl_vector_free(Yf);
        gsl_vector_free(c);
        gsl_matrix_free(cov);

        // if not successfully computed, not change former results
        if (ret != 0)
            return false;

        // update results
        clear();
        for (auto i = 0; i < poly_n; i++)
            m_coefficient[i] = coe[i];
        {
            std::vector<double> yf(length, 0);
            getYs(x, x+length, yf.begin());
            get_determinate_parameters(y, &yf[0], length, m_ssr, m_sse, m_sst, m_rmse, m_RSquare);
        }
        return true;
    }
}

#endif //M_MATH_M_CURVEFIT_H
