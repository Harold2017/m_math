//
// Created by Harold on 2021/8/22.
//

#include <open3d/3rdparty/Eigen/Eigen>
#include <iostream>
#include "../utils/stopwatch.h"

Eigen::Matrix3d cov(std::vector<Eigen::Vector3d> const& points) {
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    Eigen::Vector3d avg{ 0., 0., 0. };
    auto knn = points.size();
    for (auto i = 0; i < knn; i++) {
		avg += points[i];
	}
    avg = avg / knn;
    std::cout << "avg: " << avg.transpose() << std::endl;
    Eigen::Vector3d tmp{0., 0., 0.};
    double tmp_xy, tmp_xz, tmp_yz;
    for (auto i = 0; i < knn; i++)
    {
        tmp = points[i] - avg;
        tmp_xy = tmp.x() * tmp.y();
        tmp_xz = tmp.x() * tmp.z();
        tmp_yz = tmp.y() * tmp.z();

        cov(0, 0) += tmp.x() * tmp.x();
        cov(0, 1) += tmp_xy;
        cov(0, 2) += tmp_xz;

        cov(1, 0) += tmp_xy;
        cov(1, 1) += tmp.y() * tmp.y();
        cov(1, 2) += tmp_yz;

        cov(2, 0) += tmp_xz;
        cov(2, 1) += tmp_yz;
        cov(2, 2) += tmp.z() * tmp.z();
    }
    cov /= knn;
    return cov;
}

Eigen::Matrix3d cov1(std::vector<Eigen::Vector3d> const& points) {
    auto rows = points.size();
    Eigen::MatrixX3d pts_matrix = Eigen::Matrix3Xd::Map(points[0].data(), 3, rows).transpose();
    auto mean = pts_matrix.colwise().mean();
    std::cout << "mean: " << mean << std::endl;
    auto centered = pts_matrix.rowwise() - mean;
    return (centered.adjoint() * centered) / (rows - 1);
}

int main() {
    srand((unsigned)time(0));
    std::vector<Eigen::Vector3d> pts(200);
    std::generate(pts.begin(), pts.end(), []() { return Eigen::Vector3d::Random(); });

    Eigen::Matrix3d _cov, _cov1;

    {
        TIME_BLOCK("cov");
        _cov = cov(pts);
    }
    {
        TIME_BLOCK("cov1");
        _cov1= cov1(pts);
    }

    std::cout << _cov << "\n" << std::endl;
    std::cout << _cov1 << std::endl;

    return 0;
}