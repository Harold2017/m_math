//
// Created by Harold on 2020/12/14.
//

#ifndef M_MATH_M_MESH_H
#define M_MATH_M_MESH_H

#include <opencv2/core.hpp>
#include <open3d/Open3D.h>
#include <omp.h>

namespace M_MATH {
    class TriangleMesh {
    public:
        enum MeshType {
            BallPivot,
            Poisson
        };
    public:
        static std::shared_ptr<open3d::geometry::TriangleMesh> GenMesh(std::vector<cv::Point3f> const& pts, MeshType type);
        static bool SaveMesh(std::string const& filename, std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                             bool write_ascii = false,
                             bool compressed = false,
                             bool write_vertex_normals = true,
                             bool write_vertex_colors = true,
                             bool write_triangle_uvs = true,
                             bool print_progress = false);
        static std::shared_ptr<open3d::geometry::TriangleMesh> LoadMesh(std::string const& filename,
                                                                        bool print_progress = false);

    private:
        static std::shared_ptr<open3d::geometry::PointCloud> ToPointCloud(std::vector<cv::Point3f> const& pts);
        static std::shared_ptr<open3d::geometry::TriangleMesh> GenMesh(std::shared_ptr<open3d::geometry::PointCloud> const& pcd, MeshType type);
    };

    std::shared_ptr<open3d::geometry::PointCloud> TriangleMesh::ToPointCloud(const std::vector<cv::Point3f> &pts) {
        auto pcd = std::make_shared<open3d::geometry::PointCloud>();
        pcd->Clear();
        pcd->points_.resize(pts.size());
#pragma omp parallel for
        for (auto i = 0; i < pts.size(); i++) {
            pcd->points_[i](0) = pts[i].x;
            pcd->points_[i](1) = pts[i].y;
            pcd->points_[i](2) = pts[i].z;
        }
        return pcd;
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    TriangleMesh::GenMesh(const std::shared_ptr<open3d::geometry::PointCloud>& pcd, TriangleMesh::MeshType type) {
        if (pcd->IsEmpty())
            return nullptr;

        // compute normal
        if (!pcd->HasNormals()) {
            //pcd->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(), false);
            pcd->EstimateNormals();
            pcd->NormalizeNormals();
        }

        // gen mesh
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
        switch (type) {
            case BallPivot: {
                std::vector<double> distances = pcd->ComputeNearestNeighborDistance();
                double avg_dist = 0.f;
                for (auto dis : distances) avg_dist += dis;
                avg_dist /= distances.size();
                double radius = 1.5 * avg_dist;  // adjust this coefficient
                std::vector<double> radii = {radius, radius * 2, radius * 4, radius * 8};
                mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*pcd, radii);
                break;
            }
            case Poisson: {
                auto tuple = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pcd);
                auto p_mesh = std::get<0>(tuple);
                auto bbox = pcd->GetAxisAlignedBoundingBox();
                mesh = p_mesh->Crop(bbox);
                break;
            }
        }
        return mesh;
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    TriangleMesh::GenMesh(const std::vector<cv::Point3f> &pts, TriangleMesh::MeshType type) {
        auto pcd = ToPointCloud(pts);
        return GenMesh(pcd, type);
    }

    bool TriangleMesh::SaveMesh(const std::string &filename, std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                                bool write_ascii, bool compressed, bool write_vertex_normals, bool write_vertex_colors,
                                bool write_triangle_uvs, bool print_progress) {
        return open3d::io::WriteTriangleMeshToOBJ(filename, *mesh,
                                                  write_ascii,
                                                  compressed,
                                                  write_vertex_normals,
                                                  write_vertex_colors,
                                                  write_triangle_uvs,
                                                  print_progress);
    }

    std::shared_ptr<open3d::geometry::TriangleMesh> TriangleMesh::LoadMesh(const std::string &filename, bool print_progress) {
        return open3d::io::CreateMeshFromFile(filename, print_progress);
    }
}

#endif //M_MATH_M_MESH_H
