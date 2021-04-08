//
// Created by Harold on 2021/4/8.
//

#ifndef M_MATH_M_CURVEFIT_EIGEN_H
#define M_MATH_M_CURVEFIT_EIGEN_H

#include <map>
#include <Eigen/Core>
#include <Eigen/SVD>

namespace M_MATH {
    class CurveFitEigen {
    public:
        bool is_valid() const { return !m_coefficient.empty(); }
        void polyfit(double const* x, double const* y, size_t length, int poly_n);
        double getY(double x) const;
        template<typename IT_X, typename IT_Y>
        void getYs(IT_X x_begin, IT_X x_end, IT_Y y_begin) const;
        double get_coefficient(size_t n) const;
        double get_slope() { return m_coefficient[1]; };
        double get_intercept() { return m_coefficient[0]; };
        void clear() { m_coefficient.clear(); }
        double get_RSquare(double const* x, double const* y, size_t length) const;

    private:
        std::map<size_t, double> m_coefficient;
    };

    inline void CurveFitEigen::polyfit(const double* x, const double* y, size_t length, int poly_n)
    {
        clear();
        // matrix X
        auto X = Eigen::MatrixXd::Zero(poly_n + 1, poly_n + 1).eval();
        for (int i = 0; i < poly_n + 1; i++)
        {
            for (int j = 0; j < poly_n + 1; j++)
            {
                for (int k = 0; k < length; k++)
                {
                    X(i, j) += std::pow(x[k], i + j);
                }
            }
        }
        // matrix Y
        auto Y = Eigen::VectorXd::Zero(poly_n + 1).eval();
        for (int i = 0; i < poly_n + 1; i++)
        {
            for (int k = 0; k < length; k++)
            {
                Y(i, 0) += std::pow(x[k], i) * y[k];
            }
        }
        Eigen::VectorXd A(poly_n + 1);
        // solving matrix A
        A = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y).eval();
        // update coefficient
        for (size_t i = 0; i < poly_n + 1; ++i)
            m_coefficient[i] = A(i);
    }

    inline double CurveFitEigen::getY(double x) const {
        double ret = 0;
        for (auto const& it : m_coefficient)
            ret += (it.second) * std::pow(x, it.first);
        return ret;
    }

    template<typename IT_X, typename IT_Y>
    inline void CurveFitEigen::getYs(IT_X x_begin, IT_X x_end, IT_Y y_begin) const {
        for (; x_begin != x_end; ++x_begin, ++y_begin)
            *y_begin = getY(*x_begin);
    }

    inline double CurveFitEigen::get_coefficient(size_t n) const {
        auto it = m_coefficient.find(n);
        if (it != m_coefficient.end())
            return it->second;
        return 0.0;
    }

    inline double CurveFitEigen::get_RSquare(double const* x, double const* y, size_t length) const {
        double mean{ 0 };
        for (auto i = 0; i < length; ++i)
            mean += y[i];
        mean /= length;
        double Stot{ 0 }, Sres{ 0 };
        for (auto i = 0; i < length; ++i) {
            Stot += std::pow(y[i] - mean, 2);
            Sres += std::pow(y[i] - getY(x[i]), 2);
        }
        return 1.0 - Sres / Stot;
    }
}

#endif //M_MATH_M_CURVEFIT_EIGEN_H
