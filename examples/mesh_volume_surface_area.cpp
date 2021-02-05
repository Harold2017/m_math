//
// Created by Harold on 2020/10/31.
//

#include <iostream>
#include "m_mesh.h"
#include "m_mesh_volume_surface_area.h"

using namespace M_MATH;

int main() {
    /*
    // cubic
    std::vector<cv::Point3f> vertices = {{0, 0, 0},
                                         {1, 0, 0},
                                         {0, 1, 0},
                                         {1, 1, 0},
                                         {0, 0, 1},
                                         {1, 0, 1},
                                         {0, 1, 1},
                                         {1, 1, 1}};

    std::vector<cv::Point3i> triangles = {{0, 2, 1},
                                          {1, 2, 3},
                                          {4, 6, 5},
                                          {5, 6, 7},
                                          {0, 1, 4},
                                          {1, 5, 4},
                                          {2, 3, 6},
                                          {3, 7, 6},
                                          {2, 0, 4},
                                          {2, 4, 6},
                                          {3, 1, 5},
                                          {3, 5, 7}};

    std::cout << MeshVolume(vertices, triangles) << std::endl;
    std::cout << MeshSurface(vertices, triangles) << std::endl;
     */

    auto mesh = M_MATH::TriangleMesh::LoadMesh("x.ply");
    std::cout //<< "volume: " << mesh->GetVolume() << '\n'
            << "surface area: " << mesh->GetSurfaceArea()
            << std::endl;
    mesh->RemoveDuplicatedVertices();
    mesh->RemoveDuplicatedTriangles();
    std::vector<cv::Point3d> vert;
    std::vector<cv::Point3i> tri;
    vert.reserve(mesh->vertices_.size());
    tri.reserve(mesh->vertices_.size());
    for (auto const& v : mesh->vertices_)
        vert.emplace_back(v.x(), v.y(), v.z());
    for (auto const& t : mesh->triangles_)
        tri.emplace_back(t.x(), t.y(), t.z());
    std::cout << "mesh volume: " << MeshVolume(vert, tri) << std::endl;
    std::cout << "mesh surface: " << MeshSurface(vert, tri) << std::endl;
    auto min_v = std::min_element(mesh->vertices_.begin(), mesh->vertices_.end(),
                                  [](Eigen::Vector3d const& v1, Eigen::Vector3d const& v2){
        return v1.z() < v2.z();
    });
    std::cout << "mesh vertex min: " << "[" << min_v->x() << ", " << min_v->y() << ", " << min_v->z() << "]" << std::endl;
    std::cout << "mesh volume against plane: " << MeshVolume(vert, tri, {0, 0, min_v->z()}, {0, 0, 1});
    return 0;
}
