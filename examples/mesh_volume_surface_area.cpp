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
    auto plane_center = Eigen::Vector3d(0, 0, height);
    auto plane_normal = Eigen::Vector3d(0, 0, 1);
    plane_normal.normalize();
    {
        TIME_BLOCK("- cut mesh: ");
        M_MATH::MeshCut(*mesh, plane_center, plane_normal, l_mesh, r_mesh);
    }

    printf("left mesh vertices size: %llu, triangles size: %llu\n", l_mesh.vertices_.size(), l_mesh.triangles_.size());
    auto p_l_mesh = std::make_shared<open3d::geometry::TriangleMesh>(l_mesh);
    //open3d::visualization::DrawGeometries({ p_l_mesh }, "left mesh", 1920, 1080);

    std::cout << std::fixed;
    std::cout << std::setprecision(3);

    {
        TIME_BLOCK("- volume/surface calculation");
        std::cout << "mesh volume: " << MeshVolume(l_mesh.vertices_, l_mesh.triangles_) << std::endl;
        std::cout << "mesh surface: " << MeshSurface(l_mesh.vertices_, l_mesh.triangles_) * 1e6 << std::endl;
        std::cout << "mesh volume against plane: " << MeshVolume(l_mesh.vertices_, l_mesh.triangles_, plane_center, plane_normal) * 1e9 << std::endl;
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

    std::unordered_map<size_t, std::vector<size_t>> l_triangle_clusters, r_triangle_clusters;

    {
        TIME_BLOCK("- split l_mesh");

        l_triangle_clusters = MeshSplit(l_mesh);
        std::cout << "l_mesh cluster numbers: " << l_triangle_clusters.size() << std::endl;
    }

    {
        TIME_BLOCK("- split r_mesh");

        r_triangle_clusters = MeshSplit(r_mesh);
        std::cout << "r_mesh cluster numbers: " << r_triangle_clusters.size() << std::endl;
    }

    {
        TIME_BLOCK("- sub-mesh volume/surface calculation");

        {
            TIME_BLOCK("- l_sub-mesh volume/surface calculation");
            std::cout << "\nl_mesh: \n";
            for (auto it = l_triangle_clusters.begin(); it != l_triangle_clusters.end(); it++) {
                std::cout << "cluster " << it->first << ": " << '\n';
                auto& triangle_indices = it->second;
                std::vector<Eigen::Vector3i> triangles;
                triangles.reserve(triangle_indices.size());
                for (auto const& idx : triangle_indices)
                    triangles.emplace_back(l_mesh.triangles_[idx]);
                std::cout << "\tsurface: " << MeshSurface(l_mesh.vertices_, triangles) << '\n';
                std::cout << "\tvolume: " << MeshVolume(l_mesh.vertices_, triangles, plane_center, plane_normal) << "\n\n";
            }
        }

        {
            TIME_BLOCK("- r_sub-mesh volume/surface calculation");
            std::cout << "\nr_mesh: \n";
            for (auto it = r_triangle_clusters.begin(); it != r_triangle_clusters.end(); it++) {
                std::cout << "cluster " << it->first << ": " << '\n';
                auto& triangle_indices = it->second;
                std::vector<Eigen::Vector3i> triangles;
                triangles.reserve(triangle_indices.size());
                for (auto const& idx : triangle_indices)
                    triangles.emplace_back(r_mesh.triangles_[idx]);
                std::cout << "\tsurface: " << MeshSurface(r_mesh.vertices_, triangles) << '\n';
                std::cout << "\tvolume: " << MeshVolume(r_mesh.vertices_, triangles, plane_center, plane_normal) << "\n\n";
            }
        }
    }
    
    return 0;
}
