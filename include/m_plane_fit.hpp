//
// Created by Harold on 2021/2/4.
//

#ifndef M_MATH_M_PLANE_FIT_HPP
#define M_MATH_M_PLANE_FIT_HPP

#include <opencv2/core.hpp>

namespace M_MATH {
    /**
     * \fn  fit plane by PCA
     * @tparam T  floating point type
     * @param pts  pts on plane
     * @return  plane normal and plane center pair
     */
    template<typename T>
    std::pair<cv::Point3_<T>, cv::Point3_<T>> PlaneFit(std::vector<cv::Point3_<T>> const& pts) {
        cv::Mat covariance_matrix, mean;
        auto rows = pts.size();

        // construct covariance matrix for plane PCA
        cv::calcCovarMatrix(cv::Mat(pts).reshape(1), covariance_matrix, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS);
        covariance_matrix = covariance_matrix / (rows - 1);

        // center
        cv::Point3_<T> plane_center;
        plane_center.x = mean.at<double>(0, 0);
        plane_center.y = mean.at<double>(0, 1);
        plane_center.z = mean.at<double>(0, 2);

        cv::Mat eig_val, eig_vec;
        cv::eigen(covariance_matrix, eig_val, eig_vec);

        // normal
        cv::Point3_<T> plane_normal;
        plane_normal.x = eig_vec.at<double>(2, 0);
        plane_normal.y = eig_vec.at<double>(2, 1);
        plane_normal.z = eig_vec.at<double>(2, 2);

        // curvature
        //auto plane_curvature = eig_val.at<double>(2) / (eig_val.at<double>(0) + eig_val.at<double>(1) + eig_val.at<double>(2));
        return std::make_pair(plane_normal, plane_center);
    }
}

#endif //M_MATH_M_PLANE_FIT_HPP
