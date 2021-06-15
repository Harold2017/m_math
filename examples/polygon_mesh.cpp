//
// Created by Harold on 2021/06/8.
//

#include "m_polygon_mesh.h"
#include "m_mesh_cut.hpp"
#include "stopwatch.h"

int main(int argc, char* argv[]) {
    assert(argc == 3);  // argv[1]: mesh_file path, argv[2]: z cross section threshold for volume computation
    auto mesh_file_path = std::string(argv[1]);
    auto height = std::stod(argv[2]);

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    {
        TIME_BLOCK("- load mesh: ");
        mesh = open3d::io::CreateMeshFromFile(mesh_file_path, false);
        mesh->RemoveDuplicatedVertices();
        mesh->RemoveDuplicatedTriangles();
    }

    open3d::geometry::TriangleMesh l_mesh, r_mesh;
    auto plane_center = Eigen::Vector3d(0, 0, height);
    auto plane_normal = Eigen::Vector3d(0, 0, 1);
    plane_normal.normalize();
    std::vector<size_t> r_tmp_mesh_intersected_pt_indices;
    {
        TIME_BLOCK("- cut mesh: ");
        M_MATH::MeshCut(*mesh, plane_center, plane_normal, l_mesh, r_mesh, r_tmp_mesh_intersected_pt_indices);
    }

    printf("r_tmp_mesh_intersected_pt_indices size: %zu\n", r_tmp_mesh_intersected_pt_indices.size());

    std::vector<Eigen::Vector3d> intersected_pts(r_tmp_mesh_intersected_pt_indices.size());
    std::transform(r_tmp_mesh_intersected_pt_indices.begin(), r_tmp_mesh_intersected_pt_indices.end(), intersected_pts.begin(), [&](size_t idx) { return r_mesh.vertices_[idx]; });

    /*
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> polygon_meshes;
    {
        TIME_BLOCK("- contour polygon mesh: ");
        polygon_meshes = M_MATH::PlanePointsToContourPolygonMesh(intersected_pts, 0.0, 20);
    }
    */
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> v_polygons;// (polygon_meshes.begin(), polygon_meshes.end());

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> polygon_pcds;
    {
        TIME_BLOCK("- contour polygon pcd: ");
        polygon_pcds = M_MATH::PlanePointsToContourPolygonPCD(intersected_pts, 0.05, 20, true);
    }
    v_polygons.insert(v_polygons.end(), polygon_pcds.begin(), polygon_pcds.end());
    double zoom = 15;  // larger zoom value -> smaller display shape size
    open3d::visualization::DrawGeometries(v_polygons, "contours polygons", 1920, 1080,
                    50,
                    50,
                    false,
                    false,
                    false,
                    nullptr,
                    nullptr,
                    nullptr,
                    &zoom);

    return 0;
}