//
// Created by Harold on 2021/2/4.
//

#ifndef M_MATH_M_ELLIPSOID_FIT_HPP
#define M_MATH_M_ELLIPSOID_FIT_HPP

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Eigen>

namespace M_MATH {
    /**
     * \fn fit ellipsoid
     * Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
     * https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
     * @tparam T
     * @param pts
     * @return ellipsoid center and radii
     */
    template<typename T>
    std::pair<cv::Point3_<T>, cv::Point3_<T>> EllipsoidFit(std::vector<cv::Point3_<T>> const& pts) {
        auto mat = cv::Mat(pts).reshape(1);
        // use double for better accuracy
        mat.convertTo(mat, CV_64F);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>> data(mat.ptr<double>(), mat.rows, mat.cols);

        auto const& x = data.col(0);
        auto const& y = data.col(1);
        auto const& z = data.col(2);
        auto xx = x.cwiseProduct(x).eval();
        auto yy = y.cwiseProduct(y).eval();
        auto zz = z.cwiseProduct(z).eval();
        auto xy = x.cwiseProduct(y).eval();
        auto xz = x.cwiseProduct(z).eval();
        auto yz = y.cwiseProduct(z).eval();

        // design matrix
        // fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx +
        // 2Hy + 2Iz + J = 0 and A + B + C = 3 constraint removing one extra
        // parameter
        Eigen::Matrix<double, Eigen::Dynamic, 9> D;
        D.resize(data.rows(), 9);
        D.col(0) = xx + yy - 2. * zz;
        D.col(1) = xx + zz - 2. * yy;
        D.col(2) = 2. * xy;
        D.col(3) = 2. * xz;
        D.col(4) = 2. * yz;
        D.col(5) = 2. * x;
        D.col(6) = 2. * y;
        D.col(7) = 2. * z;
        D.col(8).setOnes();

        // solve the normal system of equations
        auto d2 = (xx + yy + zz).eval(); // the RHS of the llsq problem (y's)
        auto u = (D.transpose() * D)
                .bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
                .solve(D.transpose() * d2)
                .eval(); // solution to the normal equations

        //find the residual sum of errors
        // chi2 = sum( ( 1 - ( D * u ) ./ d2 ).^2 );
        // this chi2 is in the coordinate frame in which the ellipsoid is a unit sphere.

        Eigen::Matrix<double, 10, 1> v;
        v(0) = u(0) + u(1) - 1.;
        v(1) = u(0) - 2. * u(1) - 1.;
        v(2) = u(1) - 2. * u(0) - 1.;
        v.segment<7>(3) = u.segment<7>(2);

        // form the algebraic form of the ellipsoid
        // A = [ v(1) v(4) v(5) v(7); ...
        //       v(4) v(2) v(6) v(8); ...
        //       v(5) v(6) v(3) v(9); ...
        //       v(7) v(8) v(9) v(10) ];
        Eigen::Matrix4d A;
        A << v(0), v(3), v(4), v(6),
        v(3), v(1), v(5), v(7),
        v(4), v(5), v(2), v(8),
        v(6), v(7), v(8), v(9);

        Eigen::Vector3d center, radii;
        // find the center of the ellipsoid
        center = -A.block<3, 3>(0, 0)
                .bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
                .solve(v.segment<3>(6));

        // form the corresponding translation matrix
        Eigen::Matrix4d Trans(Eigen::Matrix4d::Identity());
        Trans.block<1, 3>(3, 0) = center.transpose();
        // translate to the center
        auto R = (Trans * A * Trans.transpose()).eval();
        // solve the eigen problem
        Eigen::EigenSolver<Eigen::Matrix3d> solver(R.block<3, 3>(0, 0) / -R(3, 3));
        radii = solver.eigenvalues().cwiseAbs().cwiseInverse().cwiseSqrt();
        for (size_t i = 0; i < 3; ++i)
            if (solver.eigenvalues()(i).real() < 0.)
                radii(i) = -radii(i);

    return std::make_pair(cv::Point3_<T>(center(0), center(1), center(2)),
            cv::Point3_<T>(radii(0), radii(1), radii(2)));
    }
}

#endif //M_MATH_M_ELLIPSOID_FIT_HPP
