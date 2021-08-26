//
// Created by Harold on 2021/8/26.
//

// avoid max/min macro in Windows.h
#define NOMINMAX

#include "../utils/io/arg_parser.h"
#include "../utils/io/filesystem.h"
#include "m_mesh_split.h"
#include "stopwatch.h"

int main(int argc, char* argv[]) {
    auto mesh = open3d::io::CreateMeshFromFile(M_ARG_PARSER::ParseAsString(argc, argv, "-i", ""), false);
    mesh->RemoveDuplicatedVertices();
    mesh->RemoveDuplicatedTriangles();
    std::cout << "mesh vertices number: " << mesh->vertices_.size() << std::endl;
    std::cout << "mesh triangles number: " << mesh->triangles_.size() << std::endl;
    std::cout << "mesh total height: " << mesh->GetAxisAlignedBoundingBox().GetExtent().z() << std::endl;
    auto min_z = std::min_element(mesh->vertices_.begin(), mesh->vertices_.end(), [](Eigen::Vector3d const& v1, Eigen::Vector3d const& v2) { return v1.z() < v2.z(); });
    std::cout << "mesh lowest z: " << min_z->z() << std::endl;

    auto height = M_ARG_PARSER::ParseAsDouble(argc, argv, "-z", 0);
    std::cout << "mesh cutting height: " << height << std::endl;
    auto plane_center = Eigen::Vector3d(0, 0, height);
    auto plane_normal = Eigen::Vector3d(0, 0, 1);

    // cut and split mesh
    auto upper_mesh = std::make_shared<open3d::geometry::TriangleMesh>(std::vector<Eigen::Vector3d>{}, std::vector<Eigen::Vector3i>{});
    auto lower_mesh = std::make_shared<open3d::geometry::TriangleMesh>(std::vector<Eigen::Vector3d>{}, std::vector<Eigen::Vector3i>{});
    {
        TIME_BLOCK("-mesh cut");
        bool ret = M_MATH::MeshCut(*mesh, plane_center, plane_normal, *upper_mesh, *lower_mesh);
        // cutting plane not intersect with mesh bounding box
        if (!ret)
        {
            if (plane_center.z() < mesh->GetAxisAlignedBoundingBox().GetCenter().z())
                upper_mesh = mesh;
            else
                lower_mesh = mesh;
        }
    }
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> subparts;
    {
        TIME_BLOCK("-mesh split");
        // split upper_mesh into sub-meshes
        subparts = M_MATH::MeshSplit2(*upper_mesh, 10.0);  // stable and accurate, since mesh includes topological info
    }
    auto N = subparts.size();
    std::cout << "sub-meshes number: " << N << std::endl;
    open3d::visualization::ColorMapJet cm;
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> pgeos;
    for (auto i = 0; i < N; i++) {
        subparts[i]->PaintUniformColor(cm.GetColor(double(i) / double(N)));
        pgeos.push_back(subparts[i]);
    }
    open3d::visualization::DrawGeometries({ pgeos }, "subparts");

    // cut and split pcd
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.reserve(mesh->vertices_.size());
    {
        TIME_BLOCK("-pcd cut");
        std::copy_if(mesh->vertices_.cbegin(), mesh->vertices_.cend(), std::back_inserter(pcd->points_), [=](Eigen::Vector3d const& pt) { return pt.z() >= height; });
    }
    std::vector<int> labels;
    {
        TIME_BLOCK("-pcd split");
        labels = pcd->ClusterDBSCAN(0.3, 15, false);  // very slow and highly depends on eps value setting (neighbor distance)
    }
    auto M = *std::max_element(labels.cbegin(), labels.cend()) + 1;
    std::cout << "sub-pcd number: " << M << std::endl;
    pcd->PaintUniformColor({0, 0, 0});
    for (auto i = 0; i < labels.size(); i++)
        if (labels[i] > 0)
            pcd->colors_[i] = cm.GetColor(double(labels[i]) / double(M));
    open3d::visualization::DrawGeometries({ pcd }, "split pcd");

    return 0;
}