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
    assert(argc == 4);  // argv[1]: mesh_file path, argv[2]: z cross section threshold for volume computation, argv[3]: mesh subparts min area size
    auto mesh_file_path = std::string(argv[1]);
    auto height = std::stod(argv[2]);
    auto min_area = std::stod(argv[3]);
    std::cout << "mesh height threshold: " << height << '\n'
              << "mesh subparts min area size: " << min_area << '\n'
              << std::endl;

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    {
        TIME_BLOCK("- load mesh: ");
        mesh = M_MATH::TriangleMesh::LoadMesh(mesh_file_path);
        std::cout //<< "volume: " << mesh->GetVolume() << '\n'
                  << "surface area: " << mesh->GetSurfaceArea() << '\n'
                  << std::endl;
        mesh->RemoveDuplicatedVertices();
        mesh->RemoveDuplicatedTriangles();
    }

    open3d::geometry::TriangleMesh l_mesh, r_mesh;
    auto plane_center = Eigen::Vector3d(0, 0, height);
    auto plane_normal = Eigen::Vector3d(0, 0, 1);
    plane_normal.normalize();
    {
        TIME_BLOCK("- cut mesh: ");
        M_MATH::MeshCut(*mesh, plane_center, plane_normal, l_mesh, r_mesh);
    }

    printf("l_mesh vertices size: %llu, triangles size: %llu\n", l_mesh.vertices_.size(), l_mesh.triangles_.size());
    printf("r_mesh vertices size: %llu, triangles size: %llu\n", r_mesh.vertices_.size(), r_mesh.triangles_.size());
    //auto p_l_mesh = std::make_shared<open3d::geometry::TriangleMesh>(l_mesh);
    //auto p_r_mesh = std::make_shared<open3d::geometry::TriangleMesh>(r_mesh);
    //p_l_mesh->PaintUniformColor(Eigen::Vector3d{ 1, 0, 0 });
    //p_r_mesh->PaintUniformColor(Eigen::Vector3d{ 0, 1, 0 });
    //open3d::visualization::DrawGeometries({ p_l_mesh, p_r_mesh }, "cutted mesh", 1920, 1080);

    std::cout << std::fixed;
    std::cout << std::setprecision(3);

    double l_mesh_surface, l_mesh_volume, r_mesh_surface, r_mesh_volume;
    ///*
    {
        TIME_BLOCK("- l_mesh volume/surface calculation");
        l_mesh_surface = MeshSurface(l_mesh.vertices_, l_mesh.triangles_);
        l_mesh_volume = MeshVolume(l_mesh.vertices_, l_mesh.triangles_, plane_center, plane_normal);
        std::cout << "l_mesh surface: " << l_mesh_surface * 1e6 << std::endl;
        std::cout << "l_mesh volume against plane: " << l_mesh_volume * 1e9 << std::endl;
    }

    {
        TIME_BLOCK("- r_mesh volume/surface calculation");
        r_mesh_surface = MeshSurface(r_mesh.vertices_, r_mesh.triangles_);
        r_mesh_volume = MeshVolume(r_mesh.vertices_, r_mesh.triangles_, plane_center, plane_normal);
        std::cout << "r_mesh surface: " << r_mesh_surface * 1e6 << std::endl;
        std::cout << "r_mesh volume against plane: " << r_mesh_volume * 1e9 << std::endl;
    }
    //*/

    /*
    {
        TIME_BLOCK("- save mesh to file");
        l_mesh.ComputeTriangleNormals();
        r_mesh.ComputeTriangleNormals();
        open3d::io::WriteTriangleMeshToSTL("l_mesh.stl", l_mesh, true, false, false, false, false, true);
        open3d::io::WriteTriangleMeshToSTL("r_mesh.stl", r_mesh, true, false, false, false, false, false);
    }
    */

    /*
    // triangle clusters
    std::unordered_map<size_t, std::vector<size_t>> l_triangle_clusters, r_triangle_clusters;

    {
        TIME_BLOCK("- split l_mesh triangles");

        l_triangle_clusters = MeshTrianglesSplit(l_mesh, 0.01 * l_mesh_surface);
        std::cout << "l_mesh triangles cluster numbers: " << l_triangle_clusters.size() << std::endl;
    }

    {
        TIME_BLOCK("- split r_mesh triangles");

        r_triangle_clusters = MeshTrianglesSplit(r_mesh, 0.01 * r_mesh_surface);
        std::cout << "r_mesh triangles cluster numbers: " << r_triangle_clusters.size() << std::endl;
    }

    {
        TIME_BLOCK("- sub-mesh volume/surface calculation by triangles");

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
    */


    /*
    // mesh clusters
    std::unordered_map<size_t, std::shared_ptr<open3d::geometry::TriangleMesh>> l_mesh_clusters, r_mesh_clusters;

    {
        TIME_BLOCK("- split l_mesh");

        l_mesh_clusters = MeshSplit(l_mesh, min_area);
        std::cout << "l_mesh cluster numbers: " << l_mesh_clusters.size() << std::endl;
    }

    {
        TIME_BLOCK("- split r_mesh");

        r_mesh_clusters = MeshSplit(r_mesh, min_area);
        std::cout << "r_mesh cluster numbers: " << r_mesh_clusters.size() << std::endl;
    }

    {
        TIME_BLOCK("- sub-mesh volume/surface calculation by meshes");

        {
            TIME_BLOCK("- l_sub-mesh volume/surface calculation");
            std::cout << "\nl_mesh: \n";
            for (auto const& mesh : l_mesh_clusters) {
                std::cout << "cluster " << mesh.first << ": " << '\n';
                std::cout << "\tsurface: " << MeshSurface(mesh.second->vertices_, mesh.second->triangles_) << '\n';
                std::cout << "\tvolume: " << MeshVolume(mesh.second->vertices_, mesh.second->triangles_, plane_center, plane_normal) << "\n\n";
            }
        }

        {
            TIME_BLOCK("- r_sub-mesh volume/surface calculation");
            std::cout << "\nr_mesh: \n";
            for (auto const& mesh : r_mesh_clusters) {
                std::cout << "cluster " << mesh.first << ": " << '\n';
                std::cout << "\tsurface: " << MeshSurface(mesh.second->vertices_, mesh.second->triangles_) << '\n';
                std::cout << "\tvolume: " << MeshVolume(mesh.second->vertices_, mesh.second->triangles_, plane_center, plane_normal) << "\n\n";
            }
        }
    }
    */

    // mesh clusters 2
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> l_mesh_clusters, r_mesh_clusters;

    {
        TIME_BLOCK("- split l_mesh");

        l_mesh_clusters = MeshSplit2(l_mesh, min_area);
        std::cout << "l_mesh cluster numbers: " << l_mesh_clusters.size() << std::endl;
    }

    {
        TIME_BLOCK("- split r_mesh");

        r_mesh_clusters = MeshSplit2(r_mesh, min_area);
        std::cout << "r_mesh cluster numbers: " << r_mesh_clusters.size() << std::endl;
    }

    /*
    {
        TIME_BLOCK("- sub-mesh volume/surface calculation by meshes");

        {
            TIME_BLOCK("- l_sub-mesh volume/surface calculation");
            std::cout << "\nl_mesh: \n";
            for (auto i = 0; i < l_mesh_clusters.size(); ++i) {
                auto mesh = l_mesh_clusters[i];
                std::cout << "cluster " << i << ": " << '\n';
                std::cout << "\tsurface: " << MeshSurface(mesh->vertices_, mesh->triangles_) << '\n';
                std::cout << "\tvolume: " << MeshVolume(mesh->vertices_, mesh->triangles_, plane_center, plane_normal) << "\n\n";
            }
        }

        {
            TIME_BLOCK("- r_sub-mesh volume/surface calculation");
            std::cout << "\nr_mesh: \n";
            for (auto i = 0; i < r_mesh_clusters.size(); ++i) {
                auto mesh = r_mesh_clusters[i];
                std::cout << "cluster " << i << ": " << '\n';
                std::cout << "\tsurface: " << MeshSurface(mesh->vertices_, mesh->triangles_) << '\n';
                std::cout << "\tvolume: " << MeshVolume(mesh->vertices_, mesh->triangles_, plane_center, plane_normal) << "\n\n";
            }
        }
    }
    */

    // display subparts
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> d_mesh_cluster;
    // auto color = Eigen::Vector3d::Random().cwiseAbs();
    open3d::visualization::ColorMapWinter cm;
    auto Nl = l_mesh_clusters.size();
    auto Nr = r_mesh_clusters.size();
    auto N = Nl + Nr;
    d_mesh_cluster.reserve(N);
    size_t i = 0;
    for (; i < Nl; ++i) {
        l_mesh_clusters[i]->PaintUniformColor(cm.GetColor(double(i) / double(N)));
        d_mesh_cluster.push_back(l_mesh_clusters[i]);
    }
    for (; i < N; ++i) {
        r_mesh_clusters[i - Nl]->PaintUniformColor(cm.GetColor(double(i) / double(N)));
        d_mesh_cluster.push_back(r_mesh_clusters[i - Nl]);
    }
    open3d::visualization::DrawGeometries(d_mesh_cluster, "l_mesh", 1920, 1080);
    // TODO: mesh cluster based on triangle edge connectivity is not so good for fractured small parts, need a way to merge them?
    
    return 0;
}
