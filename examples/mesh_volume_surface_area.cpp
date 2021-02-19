//
// Created by Harold on 2020/10/31.
//

#include <iostream>
#include "m_mesh.h"
#include "m_mesh_volume_surface_area.h"
#include "m_mesh_split.h"
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

    open3d::geometry::TriangleMesh l_mesh, r_mesh;
    auto plane_center = cv::Point3d(0, 0, mesh->GetMaxBound().z() * 0.2);
    auto plane_normal = cv::Point3d(0, 0, 1);
    plane_normal = plane_normal / cv::norm(plane_normal);
    {
        TIME_BLOCK("- split mesh: ");
        M_MATH::MeshSplit(*mesh, plane_center, plane_normal, l_mesh, r_mesh);
    }

    std::shared_ptr<open3d::geometry::TriangleMesh> p_l_mesh;
    p_l_mesh.reset(&l_mesh);
    printf("left mesh vertices size: %llu, triangles size: %llu\n", p_l_mesh->vertices_.size(), p_l_mesh->triangles_.size());
    open3d::visualization::DrawGeometries({ p_l_mesh }, "left mesh", 1920, 1080);

    std::vector<cv::Point3d> vert;
    std::vector<cv::Point3i> tri;
    vert.reserve(p_l_mesh->vertices_.size());
    tri.reserve(p_l_mesh->vertices_.size());
    {
        TIME_BLOCK("- vert/tri copy: ");
        for (auto const& v : p_l_mesh->vertices_)
            vert.emplace_back(v.x(), v.y(), v.z());
        for (auto const& t : p_l_mesh->triangles_)
            tri.emplace_back(t.x(), t.y(), t.z());
    }

    {
        TIME_BLOCK("- volume/surface calculation");
        std::cout << "mesh volume: " << MeshVolume(vert, tri) << std::endl;
        std::cout << "mesh surface: " << MeshSurface(vert, tri) << std::endl;
        std::cout << "mesh volume against plane: " << MeshVolume(vert, tri, plane_center, plane_normal) << std::endl;
    }
    
    return 0;
}
