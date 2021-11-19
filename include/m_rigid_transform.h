//
// Created by Harold on 2021/11/19.
//

// "Least-Squares Fitting of Two 3-D Point Sets", Arun, K. S. and Huang, T. S. and Blostein, S. D, IEEE Transactions on Pattern Analysis and Machine Intelligence, Volume 9 Issue 5, May 1987
// https://www.ece.queensu.ca/people/S-D-Blostein/papers/PAMI-3DLS-1987.pdf
// rotation and translation, no scaling

#ifndef M_MATH_M_RIGID_TRANSFORM_H
#define M_MATH_M_RIGID_TRANSFORM_H

#include <open3d/Open3D.h>
#include <open3d/3rdparty/Eigen/SVD>

namespace M_MATH {
    template<typename T>
    std::pair<Eigen::Matrix<T, 3, 3>, Eigen::Matrix<T, 3, 1>>
    RigidTransform(std::vector<Eigen::Matrix<T, 3, 1>> const& pts0, std::vector<Eigen::Matrix<T, 3, 1>> const& pts1)
    {
        if (pts0.size() != pts1.size())
        {
            throw "two point sets should have same size";
        }

        Eigen::Matrix<T, 3, Eigen::Dynamic> pts0_matrix = Eigen::Matrix<T, 3, Eigen::Dynamic>::Map(pts0[0].data(), 3, pts0.size());
        Eigen::Matrix<T, 3, Eigen::Dynamic> pts1_matrix = Eigen::Matrix<T, 3, Eigen::Dynamic>::Map(pts1[0].data(), 3, pts1.size());

        // column-wise mean
        Eigen::Matrix<T, 3, 1> pts0_mean = pts0_matrix.rowwise().mean();
        Eigen::Matrix<T, 3, 1> pts1_mean = pts1_matrix.rowwise().mean();

        // centered
        Eigen::Matrix<T, 3, Eigen::Dynamic> pts0_centered = pts0_matrix.colwise() - pts0_mean;
        Eigen::Matrix<T, 3, Eigen::Dynamic> pts1_centered = pts1_matrix.colwise() - pts1_mean;

        // H
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> H = pts0_centered * pts1_centered.transpose();

        // SVD
        auto svd = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto V = svd.matrixV();
        // rotation
        Eigen::Matrix<T, 3, 3> R = V * U.transpose();
        // det < 0 => reflection
        if (R.determinant() < 0)
        {
            V.col(2) *= -1;
            R = V * U.transpose();
        }

        // translation
        auto T = -R * pts0_mean + pts1_mean;

        return std::make_pair(R, T);
    }
}

#endif //M_MATH_M_RIGID_TRANSFORM_H