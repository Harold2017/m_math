//
// Created by Harold on 2021/3/8.
//

#ifndef M_MATH_M_MESH_COMPARE_H
#define M_MATH_M_MESH_COMPARE_H

#include <open3d/Open3D.h>

namespace M_MATH {
    struct MeshCompareResults {
        std::vector<Eigen::Vector3d> vertices;  // same with compared_mesh
        std::vector<Eigen::Vector3d> vertex_normals;  // same with compared_mesh
        std::vector<double> vertex_distances;  // distance of point on compared_mesh to nearest point on base_mesh
        std::vector<Eigen::Vector3i> triangles;  // same with compared_mesh
        std::vector<double> triangle_distances;  // distance of mesh triangle on compared_mesh to nearest triangle on base_mesh
    };

    MeshCompareResults MeshCompare(std::shared_ptr<open3d::geometry::TriangleMesh> base_mesh, std::shared_ptr<open3d::geometry::TriangleMesh> compared_mesh) {
        MeshCompareResults mcr;
        auto N = compared_mesh->vertices_.size();
        auto M = compared_mesh->triangles_.size();
        mcr.vertices = compared_mesh->vertices_;  // copy
        mcr.vertex_normals = compared_mesh->vertex_normals_;  // copy
        mcr.vertex_distances.reserve(N);
        mcr.triangles = compared_mesh->triangles_;  // copy
        mcr.triangle_distances.reserve(M);

        // KNN
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(*base_mesh);
        std::vector<int> indices(1);
        std::vector<double> distance2(1);

        for (auto i = 0; i < N; i++) {
            kdtree.SearchKNN(compared_mesh->vertices_[i], 1, indices, distance2);
            mcr.vertex_distances.emplace_back(std::sqrt(distance2[0]));
        }

        for (auto i = 0; i < M; i++)
            mcr.triangle_distances.emplace_back((mcr.vertex_distances[mcr.triangles[i].x()] + mcr.vertex_distances[mcr.triangles[i].y()] + mcr.vertex_distances[mcr.triangles[i].z()]) / 3.0);
        return mcr;
    }
}

#endif //M_MATH_M_MESH_COMPARE_H