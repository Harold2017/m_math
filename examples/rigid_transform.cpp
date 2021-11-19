//
// Created by Harold on 2021/11/19.
//

#include "m_rigid_transform.h"

#include <iostream>

using namespace M_MATH;

int main()
{
    srand((unsigned int)time(0));

    Eigen::MatrixXd R = Eigen::MatrixXd::Random(3, 3);
    Eigen::Vector3d T = Eigen::Vector3d::Random();

    // make R be a forced orthonormal matrix
    auto svd = R.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    R = V * U.transpose();
    if (R.determinant() < 0)
    {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    // points
    size_t const N = 10;
    std::vector<Eigen::Vector3d> pts0(N), pts1(N);
    for (auto i = 0; i < N; i++)
    {
        pts0[i] = Eigen::Vector3d::Random();
        pts1[i] = R * pts0[i] + T;
    }

    auto RT = RigidTransform(pts0, pts1);
    
    std::vector<Eigen::Vector3d> errors(N);
    for (auto i = 0; i < N; i++)
    {
        errors[i] = pts1[i] - (RT.first * pts0[i] + RT.second);
    }

    auto ErrMat = Eigen::Matrix<double, 3, N>::Map(errors[0].data(), 3, errors.size()).transpose();

    auto RMSE =  sqrt((ErrMat.transpose() * ErrMat).sum() / N);

    std::cout << "RMSE: " << RMSE << std::endl;

    if (RMSE < 1e-5)
        std::cout << "Pass" << std::endl;
    else
        std::cerr << "Fail" << std::endl;

    return 0;
}