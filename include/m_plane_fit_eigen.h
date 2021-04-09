//
// Created by Harold on 2021/4/9.
//

#ifndef M_MATH_M_PLANE_FIT_EIGEN_H
#define M_MATH_M_PLANE_FIT_EIGEN_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace M_MATH {
    /**
     * \fn  fit plane by PCA
     * @tparam T  floating point type
     * @param pts  pts on plane
     * @return  plane normal and plane center pair
     */
    template<typename T>
    std::pair<Eigen::Matrix<T, 3, 1>, Eigen::Matrix<T, 3, 1>> PlaneFit(std::vector<Eigen::Matrix<T, 3, 1>> const& pts) {
        auto rows = pts.size();
        Eigen::Matrix<T, Eigen::Dynamic, 3> pts_matrix = Eigen::MatrixXd::Map(pts[0].data(), 3, rows).transpose();  // transpose due to Eigen::Vector3T is <T, 3, 1>

        // construct covariance matrix for plane PCA
        Eigen::Matrix<T, 1, 3> mean = pts_matrix.colwise().mean();
        Eigen::Matrix<T, Eigen::Dynamic, 3> centered = pts_matrix.rowwise() - mean;
        Eigen::Matrix<T, 3, 3> covariance_matrix = (centered.adjoint() * centered) / (rows - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> solver;  // `SelfAdjointEigenSolver` for symmetric matrix and it sorts eigenvalues from lowest to highest
        solver.compute(covariance_matrix);

        return std::make_pair(solver.eigenvectors().col(0), mean.transpose());
    }
}

#endif //M_MATH_M_PLANE_FIT_EIGEN_H