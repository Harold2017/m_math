//
// Created by Harold on 2021/6/1.
//

#include "m_mesh_cut.hpp"
#include "stopwatch.h"

#include <opencv2/imgproc.hpp>

/*
#include "earcut.hpp"

namespace mapbox {
    namespace util {
        template <>
        struct nth<0, Eigen::Vector2d> {
            inline static auto get(const Eigen::Vector2d &t) {
                return t(0);
            };
        };
        template <>
        struct nth<1, Eigen::Vector2d> {
            inline static auto get(const Eigen::Vector2d &t) {
                return t(1);
            };
        };
    }
}
*/

// https://www.wikiwand.com/en/Polygon
// not self-intersecting
double PolygonArea(std::vector<Eigen::Vector2d> const& polygon_pts, bool isClosed = false) {
    if (polygon_pts.empty())
        return 0;
    double res = 0;
    auto p1 = polygon_pts[0];
    for (auto const& pt : polygon_pts) {
        res += pt(0) * p1(1) - pt(1) * p1(0);
        p1 = pt;
    }
    if (!isClosed)
        res += p1(0) * polygon_pts[0](1) - p1(1) * polygon_pts[0](0);  // pn = p0
    return abs(res) / 2.0;
}

// find convex hull of polygon
std::vector<Eigen::Vector2d> ConvexHull(std::vector<Eigen::Vector2d> const& polygon_pts) {
    cv::Mat mat = cv::Mat(int(polygon_pts.size()), 2, CV_32F, (void*)&polygon_pts[0](0));
    std::vector<int> indices;
    cv::convexHull(mat, indices, false, false);
    std::vector<Eigen::Vector2d> res(indices.size());
    std::transform(indices.begin(), indices.end(), res.begin(), [&](int idx){ return polygon_pts[idx]; });
    return res;
}

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

    printf("r_tmp_mesh_intersected_pt_indices size: %lu\n", r_tmp_mesh_intersected_pt_indices.size());

    std::vector<Eigen::Vector3d> intersected_pts(r_tmp_mesh_intersected_pt_indices.size());
    std::transform(r_tmp_mesh_intersected_pt_indices.begin(), r_tmp_mesh_intersected_pt_indices.end(), intersected_pts.begin(), [&](size_t idx) { return r_mesh.vertices_[idx]; });
    std::vector<Eigen::Vector2d> intersected_pts_2d(r_tmp_mesh_intersected_pt_indices.size());
    std::transform(r_tmp_mesh_intersected_pt_indices.begin(), r_tmp_mesh_intersected_pt_indices.end(), intersected_pts_2d.begin(), [&](size_t idx) { return Eigen::Vector2d{r_mesh.vertices_[idx].x(), r_mesh.vertices_[idx].y()}; });

    auto convex_hull = ConvexHull(intersected_pts_2d);
    printf("intersected polygon convex hull points size: %lu\n", convex_hull.size());
    printf("intersected polygon convex hull plane area: %f\n", PolygonArea(convex_hull));

    /*
    std::vector<size_t> indices;
    indices.reserve(intersected_pts_2d.size());
    std::vector<std::vector<Eigen::Vector2d>> polygon;
    polygon.push_back(intersected_pts_2d);
    {
        // very slow
        TIME_BLOCK("- intersected points triangulation: ");
        indices = mapbox::earcut<size_t>(polygon);
    }
    std::vector<Eigen::Vector3i> triangles;
    triangles.reserve(indices.size() / 3);
    for (auto i = 0; i < indices.size(); i+=3)
        triangles.emplace_back(indices[i], indices[i+1], indices[i+2]);
    auto plane_mesh = std::make_shared<open3d::geometry::TriangleMesh>(intersected_pts, triangles);
    open3d::visualization::DrawGeometries({ plane_mesh }, "intersected plane mesh", 1920, 1080);

    // - load mesh: : 3 s 526 ms 798 us 568 ns
    // - cut mesh: : 652 ms 272 us 980 ns
    // r_tmp_mesh_intersected_pt_indices size: 24436
    // - intersected points triangulation: : 5 minutes 51 s 375 ms 155 us 878 ns
    //
    // see result in `files/poly_tri_mesh.png`
    */

    return 0;
}
