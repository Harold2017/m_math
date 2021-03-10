//
// Created by Harold on 2020/10/30.
//

#ifndef M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
#define M_MATH_M_MESH_VOLUME_SURFACE_AREA_H

#include <open3d/Open3D.h>
#include <omp.h>

namespace M_MATH {
    template<typename Scalar>
    inline Scalar SignedVolumeOfTriangle(Eigen::Matrix<Scalar, 3, 1> const& v1,
                                         Eigen::Matrix<Scalar, 3, 1> const& v2,
                                         Eigen::Matrix<Scalar, 3, 1> const& v3) {
        return v1.dot(v2.cross(v3)) / 6.0;
    }


    // require closed mesh with no intersecting/overlapping triangles
    template<typename Scalar>
    inline Scalar MeshVolume(std::vector<Eigen::Matrix<Scalar, 3, 1>> const& vertices, std::vector<Eigen::Vector3i> const& triangle_v_idx) {
        double volume = 0.;
        auto N = triangle_v_idx.size();
        Eigen::Vector3i t;
#pragma omp parallel for reduction(+:volume) private(t)
        for (auto i = 0; i < N; i++) {
            t = triangle_v_idx[i];
            volume += SignedVolumeOfTriangle(vertices[t.x()], vertices[t.y()], vertices[t.z()]);
        }
        return std::abs(volume);
    }


    template<typename Scalar>
    inline Scalar AreaOfTriangle(Eigen::Matrix<Scalar, 3, 1> const& v1,
                                 Eigen::Matrix<Scalar, 3, 1> const& v2,
                                 Eigen::Matrix<Scalar, 3, 1> const& v3) {
        return 0.5 * (Eigen::Matrix<Scalar, 3, 1>(v1.x() - v2.x(), v1.y() - v2.y(), v1.z() - v2.z())
               .cross(Eigen::Matrix<Scalar, 3, 1>(v1.x() - v3.x(), v1.y() - v3.y(), v1.z() - v3.z()))).norm();
    }


    // require closed mesh with no intersecting/overlapping triangles
    template<typename Scalar>
    inline Scalar MeshSurface(std::vector<Eigen::Matrix<Scalar, 3, 1>> const& vertices, std::vector<Eigen::Vector3i> const& triangle_v_idx) {
        double area = 0.;
        auto N = triangle_v_idx.size();
        Eigen::Vector3i t;
#pragma omp parallel for reduction(+:area) private(t)
        for (auto i = 0; i < N; i++) {
            t = triangle_v_idx[i];
            area += AreaOfTriangle(vertices[t.x()], vertices[t.y()], vertices[t.z()]);
        }
        return area;
    }


    // truncated triangular prism: V = 1/3 * A * (h1 + h2 + h3)
    template<typename Scalar>
    inline Scalar TriangularPrismVolume(Eigen::Matrix<Scalar, 3, 1> const& v1,
                                        Eigen::Matrix<Scalar, 3, 1> const& v2,
                                        Eigen::Matrix<Scalar, 3, 1> const& v3,
                                        Eigen::Matrix<Scalar, 3, 1> const& plane_center,
                                        Eigen::Matrix<Scalar, 3, 1> const& plane_normal) {
        auto pn = plane_normal.normalized();
        // project vertices onto plane
        auto v1p = Eigen::Matrix<Scalar, 3, 1>(v1 - (v1 - plane_center).dot(plane_normal) * pn);
        auto v2p = Eigen::Matrix<Scalar, 3, 1>(v2 - (v2 - plane_center).dot(plane_normal) * pn);
        auto v3p = Eigen::Matrix<Scalar, 3, 1>(v3 - (v3 - plane_center).dot(plane_normal) * pn);

        auto A = AreaOfTriangle(v1p, v2p, v3p);
        auto h1 = (v1p - v1).norm();
        auto h2 = (v2p - v2).norm();
        auto h3 = (v3p - v3).norm();

        return 1. / 3. * A * (h1 + h2 + h3);
    }


    // triangle mesh volume against certain plane (project triangle to plane and compute truncated triangular prism)
    template<typename Scalar>
    inline Scalar MeshVolume(std::vector<Eigen::Matrix<Scalar, 3, 1>> const& vertices,
                             std::vector<Eigen::Vector3i> const& triangle_v_idx,
                             Eigen::Matrix<Scalar, 3, 1> const& plane_center,
                             Eigen::Matrix<Scalar, 3, 1> const& plane_normal) {
        double volume = 0.;
        auto N = triangle_v_idx.size();
        Eigen::Vector3i t;
#pragma omp parallel for reduction(+:volume) private(t)
        for (auto i = 0; i < N; i++) {
            t = triangle_v_idx[i];
            volume += TriangularPrismVolume(vertices[t.x()],
                                            vertices[t.y()],
                                            vertices[t.z()],
                                            plane_center,
                                            plane_normal);
        }
        return cv::abs(volume);
    }
}

#endif //M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
