//
// Created by Harold on 2021/2/4.
//

// some code from https://github.com/davideberly/GeometricTools/blob/master/GTE/Mathematics/ApprCylinder3.h

#ifndef M_MATH_M_CYLINDER_FIT_HPP
#define M_MATH_M_CYLINDER_FIT_HPP

#include <opencv2/core.hpp>

namespace M_MATH {
    /**
     * \struct cylinder
     * @tparam T
     */
    template<typename T>
    struct Cylinder {
        cv::Point3_<T> axis_dir;
        cv::Point3_<T> axis_point;
        T radius;
        T height;
        T fitting_error;
    };

    /**
     * Calculate the G function
     * @tparam T
     * @param point_num
     * @param mF0
     * @param mF1
     * @param mF2
     * @param mMu
     * @param W
     * @param PC
     * @param rsqr
     * @return fitting error
     */
    template<typename T>
    T G(size_t point_num,
        cv::Matx<T, 3, 3> const& mF0,
        cv::Matx<T, 3, 6> const& mF1,
        cv::Matx<T, 6, 6> const& mF2,
        cv::Vec<T, 6> const& mMu,
        cv::Vec<T, 3> const& W,
        cv::Vec<T, 3>& PC,
        T& rsqr) {
        // projection matrix
        auto P = cv::Matx<T, 3, 3>::eye() - W * W.t();
        // skew matrix of a direction w
        cv::Matx<T, 3, 3> S(0, -W[2], W[1],
                            W[2], 0, -W[0],
                            -W[1], W[0], 0);

        auto A = P * mF0 * P;
        auto hatA = -(S * A * S);
        auto hatAA = hatA * A;
        auto trace = cv::trace(hatAA);
        auto Q = hatA / trace;
        cv::Vec<T, 6> pVec{P(0, 0), P(0, 1), P(0, 2), P(1, 1), P(1, 2), P(2, 2)};
        auto alpha = mF1 * cv::Matx<T, 6, 1>(pVec);
        auto beta = Q * alpha;

        //std::cout << "P: " << P << '\n'
        //          << "mF0: " << mF0 << '\n'
        //          << "A: " << A << '\n'
        //          << "hatA: " << hatA << '\n'
        //          << "hatAA: " << hatAA << '\n'
        //          << "Q: " << Q << '\n'
        //          << std::endl;

        //std::cout << "alpha: " << alpha << '\n'
        //          << "beta: " << beta
        //          << std::endl;

        PC = cv::Vec<T, 3>(beta(0, 0), beta(1, 0), beta(2, 0));
        rsqr = pVec.dot(mMu) + beta.dot(beta);
        return (pVec.dot(mF2 * cv::Matx<T, 6, 1>(pVec)) - 4 * alpha.dot(beta) + 4 * beta.dot(mF0 * cv::Matx<T, 3, 1>(beta))) / point_num;
    }

    /**
     * \fn fit cylinder
     * https://www.geometrictools.com/Documentation/CylinderFitting.pdf
     * @tparam T
     * @param pts
     * @return cylinder (axis direction, cylinder axis origin, cylinder radius, cylinder height, fitting error)
     */
    template<typename T>
    Cylinder<T>
    CylinderFit(std::vector<cv::Point3_<T>> const& pts,
                bool using_covariance = false,
                unsigned int numPhiSamples = 100,
                unsigned int numThetaSamples = 100) {
        size_t const points_num = pts.size();
        // translate the center of mass (COM) of the data to the origin
        auto mX = cv::Mat(pts, true).reshape(1);  // copy here
        cv::Mat mean;
        cv::reduce(mX, mean, 0, cv::REDUCE_AVG);
        mX -= cv::repeat(mean, points_num, 1);

        cv::Vec<T, 6> mMu(T{});
        std::vector<cv::Vec<T, 6>> products(points_num);

        for (auto i = 0; i < points_num; ++i) {
            products[i][0] = mX.template at<T>(i, 0) * mX.template at<T>(i, 0);  // xx
            products[i][1] = mX.template at<T>(i, 0) * mX.template at<T>(i, 1);  // xy
            products[i][2] = mX.template at<T>(i, 0) * mX.template at<T>(i, 2);  // xz
            products[i][3] = mX.template at<T>(i, 1) * mX.template at<T>(i, 1);  // yy
            products[i][4] = mX.template at<T>(i, 1) * mX.template at<T>(i, 2);  // yz
            products[i][5] = mX.template at<T>(i, 2) * mX.template at<T>(i, 2);  // zz
            mMu[0] += products[i][0];
            mMu[1] += 2 * products[i][1];
            mMu[2] += 2 * products[i][2];
            mMu[3] += products[i][3];
            mMu[4] += 2 * products[i][4];
            mMu[5] += products[i][5];
        }
        mMu /= double(points_num);

        //std::cout << "mMu: " << mMu << std::endl;

        auto mF0 = cv::Matx<T, 3, 3>::zeros();
        auto mF1 = cv::Matx<T, 3, 6>::zeros();
        auto mF2 = cv::Matx<T, 6, 6>::zeros();
        for (auto i = 0; i < points_num; ++i) {
            cv::Vec<T, 6> delta(T{});
            delta[0] = products[i][0] - mMu[0];
            delta[1] = 2 * products[i][1] - mMu[1];
            delta[2] = 2 * products[i][2] - mMu[2];
            delta[3] = products[i][3] - mMu[3];
            delta[4] = 2 * products[i][4] - mMu[4];
            delta[5] = products[i][5] - mMu[5];
            mF0(0, 0) += products[i][0];
            mF0(0, 1) += products[i][1];
            mF0(0, 2) += products[i][2];
            mF0(1, 1) += products[i][3];
            mF0(1, 2) += products[i][4];
            mF0(2, 2) += products[i][5];
            mF1 += cv::Vec<T, 3>(mX.row(i)) * delta.t();
            mF2 += delta * delta.t();
        }

        mF0 /= float(points_num);
        mF0(1, 0) = mF0(0, 1);
        mF0(2, 0) = mF0(0, 2);
        mF0(2, 1) = mF0(1, 2);
        mF1 /= float(points_num);
        mF2 /= float(points_num);

        //std::cout << "mF0: " << mF0 << '\n'
        //          << "mF1: " << mF1 << '\n'
        //          << "mF2: " << mF2
        //          << std::endl;

        cv::Vec<T, 3> minPC, minW;
        T minRSqr, minError;

        if (using_covariance) {
            // compute use covariance matrix
            cv::Mat covariance_matrix, mean_mX;
            cv::calcCovarMatrix(mX, covariance_matrix, mean_mX, cv::COVAR_NORMAL | cv::COVAR_ROWS);
            covariance_matrix = covariance_matrix / (points_num - 1);
            cv::Mat eig_val, eig_vec;
            cv::eigen(covariance_matrix, eig_val, eig_vec);
            minW = eig_vec.row(2);
            //std::cout << "eig_val: " << eig_val << '\n'
            //          << "eig_vec: " << eig_vec
            //          << std::endl;
            minError = G(points_num, mF0, mF1, mF2, mMu, minW, minPC, minRSqr);
        } else {
            double const iMultiplier = M_PI / (double)points_num;
            double const jMultiplier = M_PI_2 / (double)points_num;

            // Handle the north pole (0,0,1) separately.
            minW = {0, 0, 1};
            minError = G(points_num, mF0, mF1, mF2, mMu, minW, minPC, minRSqr);

            for (unsigned int j = 1; j <= numPhiSamples; ++j)
            {
                T phi = jMultiplier * j;  // [0,pi/2]
                T csphi = std::cos(phi);
                T snphi = std::sin(phi);
                for (unsigned int i = 0; i < numThetaSamples; ++i)
                {
                    T theta = iMultiplier * i;  // [0,2*pi)
                    T cstheta = std::cos(theta);
                    T sntheta = std::sin(theta);
                    cv::Vec<T, 3> W{cstheta * snphi, sntheta * snphi, csphi};
                    cv::Vec<T, 3> PC;
                    T rsqr;
                    auto error = G(points_num, mF0, mF1, mF2, mMu, W, PC, rsqr);
                    if (error < minError)
                    {
                        minError = error;
                        minRSqr = rsqr;
                        minW = W;
                        minPC = PC;
                    }
                }
            }
        }

        // translate back to the original space by the mean of the points
        auto axis_origin = cv::Point3_<T>(minPC[0] + mean.template at<T>(0, 0),
                                          minPC[1] + mean.template at<T>(0, 1),
                                          minPC[2] + mean.template at<T>(0, 2));
        auto axis_direction = cv::Point3_<T>(minW[0], minW[1], minW[2]);

        // compute the cylinder radius
        auto cylinder_radius = std::sqrt(minRSqr);

        // project the points onto the cylinder axis and choose the
        // cylinder center and cylinder height as described in the
        // comments at the top of this header file
        auto tmin = T{};
        auto tmax = T{};
        for (unsigned int i = 0; i < points_num; ++i)
        {
            auto t = axis_direction.dot(pts[i] - axis_origin);
            tmin = std::min(t, tmin);
            tmax = std::max(t, tmax);
        }

        axis_origin += ((tmin + tmax) * 0.5) * axis_direction;
        auto cylinder_height = tmax - tmin;

        return Cylinder<T>{axis_direction,
                           axis_origin,
                           cylinder_radius,
                           cylinder_height,
                           minError};
    }
}

#endif //M_MATH_M_CYLINDER_FIT_HPP
