//
// Created by Harold on 2020/10/31.
//

#include <iostream>
#include <iomanip>
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
    auto height = std::stod(argv[2]);
    std::cout << "mesh height threshold: " << height << std::endl;
    auto plane_center = cv::Point3d(0, 0, height);
    auto plane_normal = cv::Point3d(0, 0, 1);
    plane_normal = plane_normal / cv::norm(plane_normal);
    {
        TIME_BLOCK("- cut mesh: ");
        M_MATH::MeshCut(*mesh, plane_center, plane_normal, l_mesh, r_mesh);
    }

    std::shared_ptr<open3d::geometry::TriangleMesh> p_l_mesh;
    p_l_mesh.reset(&l_mesh);
    printf("left mesh vertices size: %llu, triangles size: %llu\n", p_l_mesh->vertices_.size(), p_l_mesh->triangles_.size());
    open3d::visualization::DrawGeometries({ p_l_mesh }, "left mesh", 1920, 1080);

    std::vector<cv::Point3d> vert, rvert;
    std::vector<cv::Point3i> tri;
    vert.reserve(p_l_mesh->vertices_.size());
    rvert.reserve(r_mesh.vertices_.size());
    tri.reserve(p_l_mesh->vertices_.size());
    {
        TIME_BLOCK("- vert/tri copy: ");
        for (auto const& v : p_l_mesh->vertices_)
            vert.emplace_back(v.x(), v.y(), v.z());
        for (auto const& t : p_l_mesh->triangles_)
            tri.emplace_back(t.x(), t.y(), t.z());
        for (auto const& v : r_mesh.vertices_)
            rvert.emplace_back(v.x(), v.y(), v.z());
    }

    std::cout << std::fixed;
    std::cout << std::setprecision(3);

    {
        TIME_BLOCK("- volume/surface calculation");
        std::cout << "mesh volume: " << MeshVolume(vert, tri) << std::endl;
        std::cout << "mesh surface: " << MeshSurface(vert, tri) * 1e6 << std::endl;
        std::cout << "mesh volume against plane: " << MeshVolume(vert, tri, plane_center, plane_normal) * 1e9 << std::endl;
    }

    /*
    {
        TIME_BLOCK("- save mesh to file");
        l_mesh.ComputeTriangleNormals();
        r_mesh.ComputeTriangleNormals();
        open3d::io::WriteTriangleMeshToSTL("l_mesh.stl", l_mesh, true, false, false, false, false, true);
        open3d::io::WriteTriangleMeshToSTL("r_mesh.stl", r_mesh, true, false, false, false, false, false);
    }
    */

    {
        TIME_BLOCK("- sub-mesh volume/surface calculation");

        std::cout << "\nl_mesh: \n";
        auto triangle_clusters = MeshSplit(l_mesh);
        std::cout << "cluster numbers: " << triangle_clusters.size() << std::endl;

        for (auto it = triangle_clusters.begin(); it != triangle_clusters.end(); it++) {
            std::cout << "cluster " << it->first << ": " << '\n';
            auto& triangle_indices = it->second;
            std::vector<cv::Point3i> triangles;
            triangles.reserve(triangle_indices.size());
            for (auto const& idx : triangle_indices) {
                auto t = l_mesh.triangles_[idx];
                triangles.emplace_back(t.x(), t.y(), t.z());
            }
            std::cout << "\tsurface: " << MeshSurface(vert, triangles) << '\n';
            std::cout << "\tvolume: " << MeshVolume(vert, triangles, plane_center, plane_normal) << "\n\n";
        }

        std::cout << "\nr_mesh: \n";
        triangle_clusters = MeshSplit(r_mesh);
        std::cout << "cluster numbers: " << triangle_clusters.size() << std::endl;

        for (auto it = triangle_clusters.begin(); it != triangle_clusters.end(); it++) {
            std::cout << "cluster " << it->first << ": " << '\n';
            auto& triangle_indices = it->second;
            std::vector<cv::Point3i> triangles;
            triangles.reserve(triangle_indices.size());
            for (auto const& idx : triangle_indices) {
                auto t = r_mesh.triangles_[idx];
                triangles.emplace_back(t.x(), t.y(), t.z());
            }
            std::cout << "\tsurface: " << MeshSurface(rvert, triangles) << '\n';
            std::cout << "\tvolume: " << MeshVolume(rvert, triangles, plane_center, plane_normal) << "\n\n";
        }
    }
    
    return 0;
}
