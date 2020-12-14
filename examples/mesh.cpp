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
    //open3d::visualization::DrawGeometries({sphere}, "Test", 1920, 1080);
    // up-sample
    auto pcd = sphere->SamplePointsUniformly(20000);
    // down-sample (too slow)
    //pcd = sphere->SamplePointsPoissonDisk(500, 5, pcd);
    //open3d::visualization::DrawGeometries({pcd}, "Test", 1920, 1080);
    auto pts = ToVec3f(pcd->points_);

    // TODO: mesh quality is not so good
    auto mesh = M_MATH::TriangleMesh::GenMesh(pts, M_MATH::TriangleMesh::BallPivot);

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
    mesh->FilterSmoothTaubin(10);
    open3d::visualization::DrawGeometries({mesh}, "Test", 1920, 1080);


    return 0;
}
