//
// Created by Harold on 2021/4/9.
//

#ifndef M_MATH_M_PLANE_FIT_EIGEN_H
#define M_MATH_M_PLANE_FIT_EIGEN_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

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
        Eigen::Matrix<T, Eigen::Dynamic, 3> pts_matrix = Eigen::Matrix<T, Eigen::Dynamic, 3>::Map(pts[0].data(), 3, rows).transpose();  // transpose due to Eigen::Vector3T is <T, 3, 1>

        // construct covariance matrix for plane PCA
        Eigen::Matrix<T, 1, 3> mean = pts_matrix.colwise().mean();
        Eigen::Matrix<T, Eigen::Dynamic, 3> centered = pts_matrix.rowwise() - mean;
        Eigen::Matrix<T, 3, 3> covariance_matrix = (centered.adjoint() * centered) / (rows - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> solver;  // `SelfAdjointEigenSolver` for symmetric matrix and it sorts eigenvalues from lowest to highest
        solver.compute(covariance_matrix);

        // normal diretion has lowest variation
        // but for 3d line fitting: line_dir = solver.eigenvectors().col(2) -> highest variation

        return std::make_pair(solver.eigenvectors().col(0), mean.transpose());
    }

    /**
     * \fn  fit plane by PCA
     * SVD is slow but with higher numerical accuracy according to https://stats.stackexchange.com/questions/79043/why-pca-of-data-by-means-of-svd-of-the-data
     * @tparam T  floating point type
     * @param pts  pts on plane
     * @return  plane normal and plane center pair
     */
    template<typename T>
    std::pair<Eigen::Matrix<T, 3, 1>, Eigen::Matrix<T, 3, 1>> PlaneFitSVD(std::vector<Eigen::Matrix<T, 3, 1>> const& pts) {
        auto rows = pts.size();
        Eigen::Matrix<T, 3, Eigen::Dynamic> pts_matrix = Eigen::Matrix<T, Eigen::Dynamic, 3>::Map(pts[0].data(), 3, rows);

        // construct covariance matrix for plane PCA
        Eigen::Matrix<T, 3, 1> mean = pts_matrix.rowwise().mean();
        Eigen::Matrix<T, 3, Eigen::Dynamic> centered = pts_matrix.colwise() - mean;
        
        auto svd = centered.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

        return std::make_pair(svd.matrixU().col(2), mean.transpose());
    }
}

#endif //M_MATH_M_PLANE_FIT_EIGEN_H