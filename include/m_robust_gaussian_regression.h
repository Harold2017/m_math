//
// Created by Harold on 2021/5/11.
//

#ifndef M_MATH_M_ROBUST_GAUSSIAN_REGRESSION_H
#define M_MATH_M_ROBUST_GAUSSIAN_REGRESSION_H

#include <Eigen/Dense>

namespace M_MATH {
    // 1D Second-Order Gaussian Regression Filter
    // w(k,p)= Ax2 + Bx + C where x = (k-p)dx
    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> 
    RGR2_1D(Eigen::Matrix<T, Eigen::Dynamic, 1> const& data,
                                                T lambdac = 0.8, 
                                                T dx = 0.01, 
                                                T tol = 1e-10,
                                                unsigned max_iteration_no = 29) {
        auto n = data.size();
        static double const constant = sqrt(log(2) / 2 / EIGEN_PI / EIGEN_PI);
        Eigen::Matrix<T, Eigen::Dynamic, 1> x = Eigen::Matrix<T, Eigen::Dynamic, 1>::LinSpaced(n, 0, n) * dx;

        unsigned iterationNo = 0;
        Eigen::Matrix<T, Eigen::Dynamic, 1> delta = Eigen::Matrix<T, Eigen::Dynamic, 1>::Ones(n);
        double CBx = 1;
        double CB_old = 1;

        Eigen::Matrix<T, 3, 3> M;
        Eigen::Matrix<T, 3, 1> Q;
        Eigen::Matrix<T, 3, 1> P;
        Eigen::Matrix<T, Eigen::Dynamic, 1> w1(n);
        Eigen::Matrix<T, Eigen::Dynamic, 1> r1(n);

        while (CBx > tol) {
            for (auto k = 0; k < n; k++) {
                auto p = Eigen::Matrix<T, Eigen::Dynamic, 1>::LinSpaced(n, 0, n);
                Eigen::Matrix<T, Eigen::Dynamic, 1> S = /*(1/sqrt(2 * EIGEN_PI * EIGEN_PI)/constant/lambdac) * */ exp(-0.5 * ((k - p.array()) * dx / constant / lambdac).array().pow(2));
                S /= S.sum();
                x = (k - p.array()) * dx;
                M(0, 0) = (delta.array() * S.array() * (x.array().pow(0))).sum();  // A0
                M(0, 1) = (delta.array() * S.array() * (x.array().pow(1))).sum();  // A1
                M(0, 2) = (delta.array() * S.array() * (x.array().pow(2))).sum();  // A2
                M(1, 0) = M(0, 1);  // A1
                M(1, 1) = M(0, 2);  // A2
                M(1, 2) = (delta.array() * S.array() * (x.array().pow(3))).sum();  // A3
                M(2, 0) = M(0, 2);  // A2
                M(2, 1) = M(1, 2);  // A3
                M(2, 2) = (delta.array() * S.array() * (x.array().pow(4))).sum();  // A4

                Q(0) = (delta.array() * data.array() * S.array()).sum();  // F0
                Q(1) = (delta.array() * data.array() * S.array() * x.array()).sum();  // F1
                Q(2) = (delta.array() * data.array() * S.array() * (x.array().pow(2))).sum();  // F2

                P = M.inverse() * Q;
                w1(k) = P(0);
            }

            r1 = data - w1;
            double CB_next = 4.4 * r1.array().abs().sum() / r1.size();
            CBx = abs((CB_old - CB_next) / CB_old);
            for (auto j = 0; j < n; j++)
                if (abs(r1(j) / CB_next) < 1)
                    delta(j) = pow(1 - pow(r1(j) / CB_next, 2), 2);
                else
                    delta(j) = 0;
            CB_old = CB_next;
            ++iterationNo;
            if (iterationNo > max_iteration_no)
                break;
        }
        return w1;
    }
}

#endif //M_MATH_M_ROBUST_GAUSSIAN_REGRESSION_H