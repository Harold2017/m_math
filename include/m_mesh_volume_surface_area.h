//
// Created by Harold on 2020/10/30.
//

#ifndef M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
#define M_MATH_M_MESH_VOLUME_SURFACE_AREA_H

#include <opencv2/core.hpp>

namespace M_MATH {

    template<typename T>
    inline T SignedVolumeOfTriangle(cv::Point3_<T> const& v1,
                                    cv::Point3_<T> const& v2,
                                    cv::Point3_<T> const& v3) {
        return v1.dot(v2.cross(v3)) / 6.0;
    }

    // require closed mesh with no intersecting/overlapping triangles
    template<typename T>
    inline T MeshVolume(std::vector<cv::Point3_<T>> const& vertices, std::vector<cv::Point3i> const& triangle_v_idx) {
        auto volume = T{};
        for (auto & t : triangle_v_idx)
            volume += SignedVolumeOfTriangle(vertices[t.x], vertices[t.y], vertices[t.z]);
        return cv::abs(volume);
    }

    template<typename T>
    inline T AreaOfTriangle(cv::Point3_<T> const& v1,
                            cv::Point3_<T> const& v2,
                            cv::Point3_<T> const& v3) {
        return 0.5 * cv::norm(cv::Point3_<T>(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)
                                      .cross(cv::Point3_<T>(v1.x - v3.x, v1.y - v3.y, v1.z - v3.z)));
    }

    // require closed mesh with no intersecting/overlapping triangles
    template<typename T>
    inline T MeshSurface(std::vector<cv::Point3_<T>> const& vertices, std::vector<cv::Point3i> const& triangle_v_idx) {
        auto area = T{};
        for (auto & t : triangle_v_idx)
            area += AreaOfTriangle(vertices[t.x], vertices[t.y], vertices[t.z]);
        return area;
    }
}

#endif //M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
