//
// Created by Harold on 2020/10/31.
//

#include <iostream>
#include "m_mesh.h"
#include "m_mesh_volume_surface_area.h"
#include "stopwatch.h"

using namespace M_MATH;

int main(int argc, char* argv[]) {
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

    assert(argc > 1);  // argv[1]: mesh_file, argv[2]: z cross section threshold for volume computation

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    {
        TIME_BLOCK("- load mesh: ");
        mesh = M_MATH::TriangleMesh::LoadMesh(argv[1]);
        std::cout //<< "volume: " << mesh->GetVolume() << '\n'
            << "surface area: " << mesh->GetSurfaceArea()
            << std::endl;
        mesh->RemoveDuplicatedVertices();
        mesh->RemoveDuplicatedTriangles();
    }

    std::vector<cv::Point3d> vert;
    std::vector<cv::Point3i> tri;
    vert.reserve(mesh->vertices_.size());
    tri.reserve(mesh->vertices_.size());
    {
        TIME_BLOCK("- vert/tri copy: ");
        for (auto const& v : mesh->vertices_)
            vert.emplace_back(v.x(), v.y(), v.z());
        for (auto const& t : mesh->triangles_)
            tri.emplace_back(t.x(), t.y(), t.z());
    }

    {
        TIME_BLOCK("- volume/surface calculation");
        std::cout << "mesh volume: " << MeshVolume(vert, tri) << std::endl;
        std::cout << "mesh surface: " << MeshSurface(vert, tri) << std::endl;

        if (argc > 2) {
            auto height = std::stod(argv[2]);
            std::cout << "mesh height threshold: " << height << std::endl;
            std::cout << "mesh volume against plane: " << MeshVolume(vert, tri, { 0, 0, height }, { 0, 0, 1 }) << std::endl;
        }
        else {
            auto minmax = std::minmax_element(mesh->vertices_.begin(), mesh->vertices_.end(),
                [](Eigen::Vector3d const& v1, Eigen::Vector3d const& v2) {
                    return v1.z() < v2.z();
                });
            std::cout << "mesh vertex zmin: " << minmax.first->z() << '\n'
                << "mesh vertex zmax: " << minmax.second->z() << std::endl;
            std::cout << "mesh volume against plane: " << MeshVolume(vert, tri, { 0, 0, minmax.first->z() }, { 0, 0, 1 }) << std::endl;
        }
    }
    
    return 0;
}
