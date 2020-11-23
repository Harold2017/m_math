//
// Created by Harold on 2020/11/23.
//

#ifndef M_MATH_M_GEO_OPENCV_H
#define M_MATH_M_GEO_OPENCV_H

#include <opencv2/core.hpp>

namespace M_MATH {
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
