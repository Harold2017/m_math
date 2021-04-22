//
// Created by Harold on 2021/4/19.
//

#ifndef M_MATH_M_PLANE_SEG_OPEN3D_HPP
#define M_MATH_M_PLANE_SEG_OPEN3D_HPP

#include <open3d/Open3D.h>
#include <open3d/3rdparty/Eigen/Eigenvalues>
#include <random>

// code from: https://github.com/intel-isl/Open3D/blob/master/cpp/open3d/geometry/PointCloudSegmentation.cpp
//
// std::tuple<Eigen::Vector4d, std::vector<size_t>> PointCloud::SegmentPlane(const double distance_threshold, const int ransac_n, const int num_iterations) const;
// above open3d function returns the plane model ax + by + cz + d = 0 and the indices of the plane inliers
namespace M_MATH {
    struct PlaneInfo {
        bool valid = false;
        Eigen::Vector3d plane_center, plane_normal;
        std::vector<Eigen::Vector3d> plane_pts;

        PlaneInfo() = default;

        PlaneInfo(PlaneInfo&& move) noexcept {
            valid = move.valid;
            plane_center = move.plane_center;
            plane_normal = move.plane_normal;
            std::swap(plane_pts, move.plane_pts);
        }
    };

    namespace details {
        // from "m_plane_fit.h"
        template<typename T>
        std::pair<Eigen::Matrix<T, 3, 1>, Eigen::Matrix<T, 3, 1>> PlaneFit(std::vector<Eigen::Matrix<T, 3, 1>> const& pts) {
            auto rows = pts.size();
            Eigen::Matrix<T, Eigen::Dynamic, 3> pts_matrix = Eigen::MatrixXd::Map(pts[0].data(), 3, rows).transpose();
            Eigen::Matrix<T, 1, 3> mean = pts_matrix.colwise().mean();
            Eigen::Matrix<T, Eigen::Dynamic, 3> centered = pts_matrix.rowwise() - mean;
            Eigen::Matrix<T, 3, 3> covariance_matrix = (centered.adjoint() * centered) / (rows - 1);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> solver;
            solver.compute(covariance_matrix);

            return std::make_pair(solver.eigenvectors().col(0), mean.transpose());
        }

        // from open3d
        class RANSACResult {
        public:
            RANSACResult() : fitness_(0), inlier_rmse_(0) {}
            ~RANSACResult() {}

        public:
            double fitness_;
            double inlier_rmse_;
        };

        // from open3d
        RANSACResult EvaluateRANSACBasedOnDistance(
            const std::vector<Eigen::Vector3d>& points,
            const Eigen::Vector4d plane_model,
            std::vector<size_t>& inliers,
            double distance_threshold,
            double error) {
            RANSACResult result;

            for (size_t idx = 0; idx < points.size(); ++idx) {
                Eigen::Vector4d point(points[idx](0), points[idx](1), points[idx](2), 1);
                double distance = std::abs(plane_model.dot(point));

                if (distance < distance_threshold) {
                    error += distance;
                    inliers.emplace_back(idx);
                }
            }

            size_t inlier_num = inliers.size();
            if (inlier_num == 0) {
                result.fitness_ = 0;
                result.inlier_rmse_ = 0;
            }
            else {
                result.fitness_ = (double)inlier_num / (double)points.size();
                result.inlier_rmse_ = error / std::sqrt((double)inlier_num);
            }
            return result;
        }
    }

    // from open3d
    PlaneInfo PlaneSeg(open3d::geometry::PointCloud const& pcd,
                       const double distance_threshold /* = 0.01 */,
                       const int ransac_n /* = 3 */,
                       const int num_iterations /* = 100 */) {
        PlaneInfo res;
        details::RANSACResult result;
        double error = 0;

        // Initialize the plane model ax + by + cz + d = 0.
        Eigen::Vector4d plane_model = Eigen::Vector4d(0, 0, 0, 0);
        // Initialize the best plane model.
        Eigen::Vector4d best_plane_model = Eigen::Vector4d(0, 0, 0, 0);

        // Initialize consensus set.
        std::vector<size_t> inliers;

        size_t num_points = pcd.points_.size();
        std::vector<size_t> indices(num_points);
        std::iota(std::begin(indices), std::end(indices), 0);

        std::random_device rd;
        std::mt19937 rng(rd());

        // Return if ransac_n is less than the required plane model parameters.
        if (ransac_n < 3) {
            std::cerr << "ransac_n should be set to higher than or equal to 3." << std::endl;
            return res;
        }
        if (num_points < size_t(ransac_n)) {
            std::cerr << "There must be at least 'ransac_n' points." << std::endl;
            return res;
        }

        for (int itr = 0; itr < num_iterations; itr++) {
            for (int i = 0; i < ransac_n; ++i) {
                std::swap(indices[i], indices[rng() % num_points]);
            }
            inliers.clear();
            for (int idx = 0; idx < ransac_n; ++idx) {
                inliers.emplace_back(indices[idx]);
            }

            // Fit model to num_model_parameters randomly selected points among the inliers.
            plane_model = open3d::geometry::TriangleMesh::ComputeTrianglePlane(pcd.points_[inliers[0]], pcd.points_[inliers[1]], pcd.points_[inliers[2]]);
            if (plane_model.isZero(0)) {
                continue;
            }

            error = 0;
            inliers.clear();
            auto this_result = details::EvaluateRANSACBasedOnDistance(pcd.points_, plane_model, inliers, distance_threshold, error);
            if (this_result.fitness_ > result.fitness_ ||
                (this_result.fitness_ == result.fitness_ &&
                    this_result.inlier_rmse_ < result.inlier_rmse_)) {
                result = this_result;
                best_plane_model = plane_model;
            }
        }

        // Find the final inliers using best_plane_model.
        inliers.clear();
        for (size_t idx = 0; idx < pcd.points_.size(); ++idx) {
            Eigen::Vector4d point(pcd.points_[idx](0), pcd.points_[idx](1), pcd.points_[idx](2), 1);
            double distance = std::abs(best_plane_model.dot(point));

            if (distance < distance_threshold) {
                inliers.emplace_back(idx);
            }
        }

        std::vector<Eigen::Vector3d> plane_pts;
        plane_pts.reserve(inliers.size());
        for (auto const idx : inliers)
            plane_pts.push_back(pcd.points_[idx]);
        auto cn = details::PlaneFit(plane_pts);
        res.plane_center = cn.second;
        res.plane_normal = cn.first;
        res.plane_pts = plane_pts;
        res.valid = true;
        return res;
    }

    /**
     * @brief segment multiple planes from point cloud
     * @param pcd input point cloud
     * @param min_left_points_ratio minimum left points ratio to end the segmentation
     * @param distance_threshold RANSAC distance threshold
     * @param ransac_n at least 3 points to form a plane primitive
     * @param num_iterations RANSAC iterations
     * @return list of segmented planes
    */
    std::vector<PlaneInfo> MultiplePlaneSeg(open3d::geometry::PointCloud const& pcd,
                                            const double min_left_points_ratio /* = 0.05 */,
                                            const double distance_threshold /* = 0.01 */,
                                            const int ransac_n /* = 3 */,
                                            const int num_iterations /* = 100 */) {
        std::vector<PlaneInfo> res;
        // copy pcd points
        open3d::geometry::PointCloud _pcd{};
        _pcd.points_ = pcd.points_;
        size_t num_points = (1 - min_left_points_ratio) * pcd.points_.size();
        size_t cnt = 0;

        auto move_points = [](std::vector<Eigen::Vector3d>& pts, std::vector<size_t>& idx) {
            std::vector<Eigen::Vector3d> res;
            res.reserve(idx.size());
            /* sort indices and then delete those elements from the vector from the highest to the lowest. 
             * deleting the highest index on a list will not invalidate the lower indices you want to delete, 
             * because only the elements higher than the deleted ones change their index.
             */
            std::sort(idx.begin(), idx.end());
            for (int i = idx.size() - 1; i >= 0; i--) {
                res.push_back(pts[idx[i]]);
                pts.erase(pts.begin() + idx[i]);
            }
            return res;
        };

        while (cnt < num_points) {
            auto plane = _pcd.SegmentPlane(distance_threshold, ransac_n, num_iterations);
            auto plane_equation = std::get<0>(plane);
            auto plane_pts_idx = std::get<1>(plane);

            cnt += plane_pts_idx.size();

            PlaneInfo pi;
            pi.valid = true;
            pi.plane_center = _pcd.points_[plane_pts_idx[0]];
            pi.plane_normal = Eigen::Vector3d(plane_equation[0], plane_equation[1], plane_equation[2]);
            pi.plane_pts = move_points(_pcd.points_, plane_pts_idx);
            res.emplace_back(std::move(pi));
        }

        return res;
    }
}

#endif //M_MATH_M_PLANE_SEG_OPEN3D_HPP