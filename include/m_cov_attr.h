//
// Created by Harold on 2021/4/30.
//

#ifndef M_MATH_M_COV_ATTR_H
#define M_MATH_M_COV_ATTR_H

#include <open3d/Open3D.h>
#include <open3d/3rdparty/Eigen/Eigen>
#include <ostream>

namespace M_MATH {
    // points set consist of its k nearest neighbors
    // eigenvalues: l0 > l1 > l2 >= 0
    // eigenvectors: e0, e1, e2
    struct CovarianceAttributes {
        double lambda0, lambda1, lambda2;
        double omni_var,     // (l0 * l1 * l2) ^ (1/3)
               surface_var,  // l2 / (l0 + l1 + l2)
               anisotropy,   // (l0 - l2) / l0
               // https://papers.ssrn.com/sol3/papers.cfm?abstract_id=2767549 
               entropy,      // effective dimensionality = exp(Shannon entropy), Shannon entropy = -sum(li * ln(li))
               linearity,    // (l0 - l1) / l0 ≈ 1   =>   l0 >> l1 ≈ l2
               planarity,    // (l1 - l2) / l0 ≈ 1   =>   l0 ≈ l1 >> l2
               sphericity;   // l2        / l0 ≈ 1   =>   l0 ≈ l1 ≈ l2, scatter

        CovarianceAttributes() = default;
        void Compute(std::vector<Eigen::Vector3d> const& pts, size_t pt_idx, int knn);
        friend std::ostream& operator<<(std::ostream& os, CovarianceAttributes const& ca);
    };

    void CovarianceAttributes::Compute(std::vector<Eigen::Vector3d> const& pts, size_t pt_idx, int knn) {
        // knn
        open3d::geometry::PointCloud pcd(pts);
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(pcd);
        std::vector<int> indices(knn);
        std::vector<double> distance2(knn);
        kdtree.SearchKNN(pts[pt_idx], knn, indices, distance2);
        std::vector<Eigen::Vector3d> points;
        points.reserve(knn + 1);
        points.push_back(pts[pt_idx]);
        std::transform(indices.begin(), indices.end(), std::back_inserter(points), [&](int i) { return pts[i]; });

        // construct cov matrix
        auto rows = points.size();
        Eigen::MatrixX3d pts_matrix = Eigen::Matrix3Xd::Map(points[0].data(), 3, rows).transpose();
        auto centered = pts_matrix.rowwise() - pts_matrix.colwise().mean();
        auto cov = (centered.adjoint() * centered) / (rows - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
        solver.compute(cov);
        Eigen::Vector3d lambda = solver.eigenvalues();
        lambda0 = lambda(2); //abs(lambda(2));
        lambda1 = lambda(1); //abs(lambda(1));
        lambda2 = lambda(0); //abs(lambda(0));

        omni_var = pow(lambda0 * lambda1 * lambda2, 1.0 / 3.0);
        surface_var = lambda2 / (lambda0 + lambda1 + lambda2);
        anisotropy = (lambda0 - lambda2) / lambda0;
        entropy = -(lambda0 * log(lambda0) + lambda1 * log(lambda1) + lambda2 * log(lambda2));
        linearity = (lambda0 - lambda1) / lambda0;
        planarity = (lambda1 - lambda2) / lambda0;
        sphericity = lambda2 / lambda0;
    }

    std::ostream& operator<<(std::ostream& os, CovarianceAttributes const& ca) {
        os << "Covariance Attributes: \n"
           << "\tlambda: [" << ca.lambda0 << ", " << ca.lambda1 << ", " << ca.lambda2 << "]\n"
           << "\tomni_var: " << ca.omni_var << '\n'
           << "\tsurface_var: " << ca.surface_var << '\n'
           << "\tanisotropy: " << ca.anisotropy << '\n'
           << "\tentropy: " << ca.entropy << '\n'
           << "\tlinearity: " << ca.linearity << '\n'
           << "\tplanarity: " << ca.planarity << '\n'
           << "\tsphericity: " << ca.sphericity << '\n';
        return os;
    }
}

#endif //M_MATH_M_COV_ATTR_H