//
// Created by Harold on 2020/10/30.
//

#ifndef M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
#define M_MATH_M_MESH_VOLUME_SURFACE_AREA_H

#include <opencv2/core.hpp>
#include <omp.h>

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
        auto N = triangle_v_idx.size();
        cv::Point3i t;
#pragma omp parallel for reduction(+:volume) private(t)
        for (auto i = 0; i < N; i++) {
            t = triangle_v_idx[i];
            volume += SignedVolumeOfTriangle(vertices[t.x], vertices[t.y], vertices[t.z]);
        }
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
        auto N = triangle_v_idx.size();
        cv::Point3i t;
#pragma omp parallel for reduction(+:area) private(t)
        for (auto i = 0; i < N; i++) {
            t = triangle_v_idx[i];
            area += AreaOfTriangle(vertices[t.x], vertices[t.y], vertices[t.z]);
        }
        return area;
    }

    // truncated triangular prism: V = 1/3 * A * (h1 + h2 + h3)
    template<typename T>
    inline T TriangularPrismVolume(cv::Point3_<T> const& v1,
                                   cv::Point3_<T> const& v2,
                                   cv::Point3_<T> const& v3,
                                   cv::Point3_<T> const& plane_center,
                                   cv::Point3_<T> const& plane_normal) {
        // project vertices onto plane
        auto v1p = v1 - (v1 - plane_center).dot(plane_normal) * plane_normal / cv::norm(plane_normal);
        auto v2p = v2 - (v2 - plane_center).dot(plane_normal) * plane_normal / cv::norm(plane_normal);
        auto v3p = v3 - (v3 - plane_center).dot(plane_normal) * plane_normal / cv::norm(plane_normal);

        auto A = AreaOfTriangle(v1p, v2p, v3p);
        auto h1 = cv::norm(v1p - v1);
        auto h2 = cv::norm(v2p - v2);
        auto h3 = cv::norm(v3p - v3);

        return 1./3. * A * (h1 + h2 + h3);
    }

    // triangle mesh volume against certain plane (project triangle to plane and compute truncated triangular prism)
    template<typename T>
    inline T MeshVolume(std::vector<cv::Point3_<T>> const& vertices,
                        std::vector<cv::Point3i> const& triangle_v_idx,
                        cv::Point3_<T> const& plane_center,
                        cv::Point3_<T> const& plane_normal) {
        auto volume = T{};
        auto N = triangle_v_idx.size();
        cv::Point3i t;
#pragma omp parallel for reduction(+:volume) private(t)
        for (auto i = 0; i < N; i++) {
            t = triangle_v_idx[i];
            volume += TriangularPrismVolume(vertices[t.x],
                                            vertices[t.y],
                                            vertices[t.z],
                                            plane_center,
                                            plane_normal);
        }
        return cv::abs(volume);
    }
}

#endif //M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
