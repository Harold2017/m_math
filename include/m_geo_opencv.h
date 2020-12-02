//
// Created by Harold on 2020/11/23.
//

#ifndef M_MATH_M_GEO_OPENCV_H
#define M_MATH_M_GEO_OPENCV_H

#include <opencv2/core.hpp>

namespace M_MATH {
    /**
     * \fn  helper function to get plane normal and plane center pair from 3 points
     * @tparam T  floating point type
     * @param p0  point on plane
     * @param p1  point on plane
     * @param p2  point on plane
     * @return  plane normal and plane center pair
     */
    template<typename T>
    std::pair<cv::Point3_<T>, cv::Point3_<T>> GetPlane(cv::Point3_<T> const& p0,
                            cv::Point3_<T> const& p1,
                            cv::Point3_<T> const& p2) {
        return std::make_pair((p0-p1).cross(p0-p2), p0);
    }

    /**
     * \fn  helper function to get plane normal and plane center pair by PCA
     * @tparam T  floating point type
     * @param pts  pts on plane
     * @return  plane normal and plane center pair
     */
    template<typename T>
    std::pair<cv::Point3_<T>, cv::Point3_<T>> GetPlane(std::vector<cv::Point3_<T>> const& pts) {
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

    /**
     * \fn  helper function to get points within certain distance to reference plane
     * @tparam T  floating point type
     * @tparam IT  iterator of Point3D<T>
     * @param begin  iterator begin
     * @param end  iterator end
     * @param plane_normal  reference plane normal
     * @param plane_center  reference plane center
     * @return  vector of points
    */
    template<typename T, typename IT>
    std::vector<cv::Point3_<T>> PointsWithinDistanceToReferencePlane(IT begin, IT end,
                                                                     cv::Point3_<T> const& plane_normal,
                                                                     cv::Point3_<T> const& plane_center,
                                                                     T const threshold) {
        std::vector<cv::Point3_<T>> res;
        res.reserve(std::distance(begin, end));
        for (auto it = begin; it != end; ++it) {
            if (std::abs(((*it) - plane_center).dot(plane_normal)) <= threshold)
                res.push_back(*it);
        }
        return res;
    }

    /**
     * \fn  helper function to get points with distance within threshold from reference plane
     * @tparam T  floating point type
     * @tparam IT  iterator of Point3D<T>
     * @param begin  iterator begin
     * @param end  iterator end
     * @param plane_normal  reference plane normal
     * @param plane_center  reference plane center
     * @param threshold  distance threshold
     * @return  vector of points
     */
    template<typename T, typename IT>
    std::vector<cv::Point3_<T>> PlanePts(IT begin, IT end,
                                         cv::Point3_<T> const& plane_normal,
                                         cv::Point3_<T> const& plane_center,
                                         T const threshold) {
        return PointsWithinDistanceToReferencePlane(begin, end, plane_normal, plane_center, threshold);
    }

    /**
     * \fn  helper function to get points with distance within threshold from two cross reference plane
     * @tparam T  floating point type
     * @tparam IT  iterator of Point3D<T>
     * @param begin  iterator begin
     * @param end  iterator end
     * @param plane_normal_0  reference plane 0 normal
     * @param plane_center_0  reference plane 0 center
     * @param plane_normal_1  reference plane 1 normal
     * @param plane_center_1  reference plane 1 center
     * @param threshold_0  distance threshold 0
     * @param threshold_1  distance threshold 1
     * @return vector of points
     */
    template<typename T, typename IT>
    std::vector<cv::Point3_<T>> LinePts(IT begin, IT end,
                                        cv::Point3_<T> const& plane_normal_0,
                                        cv::Point3_<T> const& plane_center_0,
                                        cv::Point3_<T> const& plane_normal_1,
                                        cv::Point3_<T> const& plane_center_1,
                                        T const threshold_0,
                                        T const threshold_1) {
        auto pts = PointsWithinDistanceToReferencePlane(begin, end, plane_normal_0, plane_center_0, threshold_0);
        return PointsWithinDistanceToReferencePlane(pts.begin(), pts.end(), plane_normal_1, plane_center_1, threshold_1);
    }
}

#endif //M_MATH_M_GEO_OPENCV_H
