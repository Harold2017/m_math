//
// Created by Harold on 2020/12/14.
//

// https://github.com/intel-isl/Open3D/blob/master/examples/cpp/TriangleMesh.cpp

#include <iostream>
#include "m_mesh.h"

void PaintMesh(open3d::geometry::TriangleMesh &mesh,
               const Eigen::Vector3d &color) {
    mesh.vertex_colors_.resize(mesh.vertices_.size());
    for (size_t i = 0; i < mesh.vertices_.size(); i++) {
        mesh.vertex_colors_[i] = color;
    }
}

std::vector<cv::Point3f> ToVec3f(std::vector<Eigen::Vector3d> const& pts) {
    std::vector<cv::Point3f> res;
    res.reserve(pts.size());
    for (auto const& e : pts)
        res.emplace_back(e(0), e(1), e(2));
    return res;
}

int main() {
    // cubic
    //auto cube = open3d::geometry::TriangleMesh::CreateBox();
    //open3d::visualization::DrawGeometries({cube}, "Test", 1920, 1080);
    //open3d::io::WriteTriangleMesh("cube.ply", *cube, true, true);
    //auto pts = ToVec3f(cube->vertices_);

    // sphere
    auto sphere = open3d::geometry::TriangleMesh::CreateSphere();
    std::cout << "points: " << sphere->vertices_.size() << std::endl;
    std::cout << "triangles: " << sphere->triangles_.size() << std::endl;
    std::cout << sphere->GetSurfaceArea() << '\n';
    if (sphere->IsWatertight())
        std::cout << sphere->GetVolume() << '\n';
    std::cout << std::endl;
    //open3d::visualization::DrawGeometries({sphere}, "Test", 1920, 1080);
    // up-sample
    auto pcd = sphere->SamplePointsUniformly(20000);
    // down-sample (too slow)
    //pcd = sphere->SamplePointsPoissonDisk(500, 5, pcd);
    //open3d::visualization::DrawGeometries({pcd}, "Test", 1920, 1080);
    auto pts = ToVec3f(pcd->points_);

    // TODO: mesh quality is not so good and why surface area change strangely?
    auto mesh = M_MATH::TriangleMesh::GenMesh(pts, M_MATH::TriangleMesh::BallPivot);

    // save mesh
    //M_MATH::TriangleMesh::SaveMesh("mesh.obj", mesh);

    std::cout << "triangles: " << mesh->triangles_.size() << std::endl;

    PaintMesh(*mesh, Eigen::Vector3d(1.0, 0.75, 0.0));
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> v_mesh;
    v_mesh.push_back(mesh);
    open3d::visualization::DrawGeometries(v_mesh, "Test", 1920, 1080);

    std::cout << mesh->GetSurfaceArea() << '\n';
    if (mesh->IsWatertight())
        std::cout << mesh->GetVolume() << '\n';
    std::cout << std::endl;

    // filter mesh
    //mesh->FilterSmoothTaubin(10);
    //open3d::visualization::DrawGeometries({mesh}, "Test", 1920, 1080);

    // after ManifoldPlus
    // load mesh
    auto mesh_1 = M_MATH::TriangleMesh::LoadMesh("out.obj");
    open3d::visualization::DrawGeometries({mesh_1}, "Test", 1920, 1080);
    std::cout << "triangles: " << mesh_1->triangles_.size() << std::endl;
    std::cout << mesh_1->GetSurfaceArea() << '\n';
    if (mesh_1->IsWatertight())
        std::cout << mesh_1->GetVolume() << '\n';
    std::cout << std::endl;

    // remove redundant triangles (no effect here)
    //auto mesh_2 = std::make_shared<open3d::geometry::TriangleMesh>(mesh_1->RemoveDuplicatedVertices().RemoveDegenerateTriangles().RemoveDuplicatedTriangles());
    //std::cout << "triangles: " << mesh_2->triangles_.size() << std::endl;
    //std::cout << mesh_2->GetSurfaceArea() << '\n';
    //if (mesh_2->IsWatertight())
    //    std::cout << mesh_2->GetVolume() << '\n';
    //std::cout << std::endl;

    // simplification
    auto bound_1 = mesh_1->GetMaxBound() - mesh_1->GetMinBound();
    auto voxel_size_1 = *std::max_element(bound_1.begin(), bound_1.end()) / 16;
    std::cout << "voxel_size: " << voxel_size_1 << std::endl;
    auto mesh_11 = mesh_1->SimplifyVertexClustering(voxel_size_1, open3d::geometry::HalfEdgeTriangleMesh::SimplificationContraction::Average);
    std::cout << "triangles: " << mesh_11->triangles_.size() << std::endl;
    std::cout << mesh_11->GetSurfaceArea() << '\n';
    if (mesh_11->IsWatertight())
        std::cout << mesh_11->GetVolume() << '\n';
    std::cout << std::endl;
    open3d::visualization::DrawGeometries({mesh_11}, "Test", 1920, 1080);

    return 0;
}
