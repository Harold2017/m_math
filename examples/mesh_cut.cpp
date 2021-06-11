//
// Created by Harold on 2021/6/1.
//

#include "m_mesh_cut.hpp"
#include "stopwatch.h"

#include <opencv2/imgproc.hpp>

#include "concaveman.hpp"

#include "m_voxelgrid2d.h"

#include "earcut.hpp"

//#include <fstream>

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
    std::vector<cv::Point2d> pts(polygon_pts.size());
    std::transform(polygon_pts.begin(), polygon_pts.end(), pts.begin(), [](Eigen::Vector2d const& p) { return cv::Point2d{ p(0), p(1) }; });
    cv::Mat mat(pts);
    mat.convertTo(mat, CV_32FC1);
    std::vector<int> indices;
    cv::convexHull(mat, indices, false, false);
    std::vector<Eigen::Vector2d> res(indices.size());
    std::transform(indices.begin(), indices.end(), res.begin(), [&](int idx) { return polygon_pts[idx]; });
    return res;
}


// concave hull
std::vector<Eigen::Vector2d> ConcaveHull(std::vector<Eigen::Vector2d> const& polygon_pts) {
    std::vector<std::array<double, 2>> pts(polygon_pts.size());
    std::transform(polygon_pts.begin(), polygon_pts.end(), pts.begin(), [](Eigen::Vector2d const& p){ return std::array<double, 2>{p(0), p(1)}; });

    std::vector<cv::Point2d> pts_cv(polygon_pts.size());
    std::transform(polygon_pts.begin(), polygon_pts.end(), pts_cv.begin(), [](Eigen::Vector2d const& p) { return cv::Point2d{ p(0), p(1) }; });
    cv::Mat mat(pts_cv);
    mat.convertTo(mat, CV_32FC1);
    std::vector<int> indices;
    cv::convexHull(mat, indices, false, false);

    auto concave_hull = M_MATH::CONCAVEMAN::concaveman<double, 16>(pts, indices);
    std::vector<Eigen::Vector2d> res(concave_hull.size());
    std::transform(concave_hull.begin(), concave_hull.end(), res.begin(), [](std::array<double, 2> const& p){ return Eigen::Vector2d{p[0], p[1]}; });
    return res;
}


// get voxel grid and contours
std::pair<M_MATH::VoxelGrid<double>, std::vector<std::vector<cv::Point>>> voxelization_to_find_contours(std::vector<Eigen::Vector2d> const& points) {
    // find four extreme points
    double x_min = points[0](0);
    double x_max = points[0](0);
    double y_min = points[0](1);
    double y_max = points[0](1);
    for (auto const& p : points) {
        if (p(0) < x_min) x_min = p(0);
        if (p(0) > x_max) x_max = p(0);
        if (p(1) < y_min) y_min = p(1);
        if (p(1) > y_max) y_max = p(1);
    }
    // estimate voxel size, assume points are even distributed
    double vs = 2 * std::sqrt((x_max - x_min) * (y_max - y_min) / double(points.size()));
    // set voxelgrid a little larger than bounding box of points
    double grid_x_min = x_min - (x_max - x_min) * 0.1;
    double grid_y_min = y_min - (y_max - y_min) * 0.1;
    double grid_x_max = x_max + (x_max - x_min) * 0.1;
    double grid_y_max = y_max + (y_max - y_min) * 0.1;
    size_t rows = MAX(1, static_cast<size_t>((grid_y_max - grid_y_min) / vs));
    size_t cols = MAX(1, static_cast<size_t>((grid_x_max - grid_x_min) / vs));
    // cv points
    std::vector<cv::Point2d> pts_cv(points.size());
    std::transform(points.begin(), points.end(), pts_cv.begin(), [](Eigen::Vector2d const& p) { return cv::Point2d{ p(0), p(1) }; });
    // voxel grid
    auto vg = M_MATH::VoxelGrid<double>(rows, cols, vs, vs);
    auto img = vg.CreateImg(pts_cv, true, cv::Point2d{ grid_x_min, grid_y_min });
    std::cout << "voxel grid img size: " << img.size << std::endl;
    // find contours
    cv::threshold(img, img, 127, 255, 0);
    std::vector<std::vector<cv::Point>> idx_contours;
    ///\note only consider the outer contours here
    cv::findContours(img, idx_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    // filter out closed contours
    std::vector<std::vector<cv::Point>> filtered_contours;
    for (auto i = 0; i < idx_contours.size(); i++)
        if (cv::contourArea(idx_contours[i]) <= cv::arcLength(idx_contours[i], true)) continue; // open contour (area <= perimeter)
        else filtered_contours.push_back(idx_contours[i]);  // closed contour
    return std::make_pair(std::move(vg), std::move(filtered_contours));
}

// mapping voxel (pixel) index to point by offset (one voxel to one point)
std::vector<std::vector<Eigen::Vector2d>> mapping_voxel_idx_to_pt(std::vector<std::vector<cv::Point>> const& idx_contours, M_MATH::VoxelGrid<double> const& vg, size_t min_contour_points_num) {
    std::vector<std::vector<Eigen::Vector2d>> contours;
    auto vx = vg.m_vx;
    auto vy = vg.m_vy;
    auto ox = vg.m_origin.x;
    auto oy = vg.m_origin.y;
    for (auto i = 0; i < idx_contours.size(); i++) {
        std::vector<Eigen::Vector2d> tmp;
        if (idx_contours[i].size() < min_contour_points_num) continue;
        for (auto j = 0; j < idx_contours[i].size(); j++) {
            auto p_idx = idx_contours[i][j];
            tmp.emplace_back(static_cast<double>(p_idx.x * vx + ox + vx / 2),
                             static_cast<double>(p_idx.y * vy + oy + vy / 2));
        }
        contours.push_back(tmp);
    }
    return contours;
}

// mapping voxel (pixel) index to point by grid point ids (one voxel to its point ids)
std::vector<std::vector<size_t>> mapping_voxel_idx_to_pt_ids(std::vector<std::vector<cv::Point>> const& idx_contours, M_MATH::VoxelGrid<double> const& vg, size_t min_contour_points_num = 20) {
    std::vector<std::vector<size_t>> pt_ids;
    auto const& voxels = vg.m_voxels;
    for (auto i = 0; i < idx_contours.size(); i++) {
        std::vector<size_t> tmp;
        if (idx_contours[i].size() < min_contour_points_num) continue;
        for (auto j = 0; j < idx_contours[i].size(); j++) {
            auto p_idx = idx_contours[i][j];
            for (auto s : voxels[p_idx.y][p_idx.x].m_pt_ids)  // notice: voxle[y][x] here
                tmp.push_back(s);
        }
        pt_ids.push_back(tmp);
    }
    return pt_ids;
}

// contour
std::vector<std::vector<Eigen::Vector2d>> Contours(std::vector<Eigen::Vector2d> const& points, size_t min_contour_points_num = 20) {
    auto pair = voxelization_to_find_contours(points);
    return mapping_voxel_idx_to_pt(pair.second, pair.first, min_contour_points_num);
}
// contour of point ids
std::vector<std::vector<size_t>> Contours_pt_ids(std::vector<Eigen::Vector2d> const& points, size_t min_contour_points_num = 20) {
    auto pair = voxelization_to_find_contours(points);
    return mapping_voxel_idx_to_pt_ids(pair.second, pair.first, min_contour_points_num);
}

std::vector<Eigen::Vector3i> PolygonTriangulation(std::vector<Eigen::Vector2d> const& pts) {
    auto indices = mapbox::earcut<size_t>(std::vector<std::vector<Eigen::Vector2d>>{ pts });
    std::vector<Eigen::Vector3i> triangles;
    triangles.reserve(indices.size() / 3);
    for (auto i = 0; i < indices.size(); i += 3)
        triangles.emplace_back(indices[i], indices[i + 1], indices[i + 2]);
    return triangles;
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

    printf("r_tmp_mesh_intersected_pt_indices size: %zu\n", r_tmp_mesh_intersected_pt_indices.size());

    std::vector<Eigen::Vector2d> intersected_pts_2d(r_tmp_mesh_intersected_pt_indices.size());
    std::transform(r_tmp_mesh_intersected_pt_indices.begin(), r_tmp_mesh_intersected_pt_indices.end(), intersected_pts_2d.begin(), [&](size_t idx) { return Eigen::Vector2d{r_mesh.vertices_[idx].x(), r_mesh.vertices_[idx].y()}; });


    std::vector<Eigen::Vector2d> convex_hull;
    {
        TIME_BLOCK("- convex hull: ");
        convex_hull = ConvexHull(intersected_pts_2d);
    }
    printf("intersected polygon convex hull points size: %zu\n", convex_hull.size());
    printf("intersected polygon convex hull plane area: %f\n", PolygonArea(convex_hull));

    std::vector<Eigen::Vector2d> concave_hull;
    {
        TIME_BLOCK("- concave hull: ");
        concave_hull = ConcaveHull(intersected_pts_2d);
    }
    printf("intersected polygon concave hull points size: %zu\n", concave_hull.size());
    printf("intersected polygon concave hull plane area: %f\n", PolygonArea(concave_hull));

    ///*
    // contours with point ids
    std::vector<std::vector<size_t>> contours_pt_ids;
    {
        TIME_BLOCK("- contours with point ids: ");
        contours_pt_ids = Contours_pt_ids(intersected_pts_2d, 20);
    }
    // notice: contours and contours_pt_ids are not 1 to 1 indexing
    std::vector<std::vector<Eigen::Vector2d>> contours(contours_pt_ids.size());
    std::transform(contours_pt_ids.begin(), contours_pt_ids.end(), contours.begin(),
        [&](std::vector<size_t> const& pt_ids) {
            std::vector<Eigen::Vector2d> pts(pt_ids.size());
            std::transform(pt_ids.begin(), pt_ids.end(), pts.begin(), [&](size_t id) { return intersected_pts_2d[id]; });
            return ConcaveHull(pts); });  // compute concave hull here to ensure earcut mesh quality
    //*/
    /*
    std::vector<std::vector<Eigen::Vector2d>> contours;
    {
        TIME_BLOCK("- contours: ");
        //contours = Contours(intersected_pts_2d, 20);
        // find contour concave hull
        auto _contours = Contours(intersected_pts_2d, 20);
        contours.resize(_contours.size());
        std::transform(_contours.begin(), _contours.end(), contours.begin(), [](std::vector<Eigen::Vector2d> const& c) { return ConcaveHull(c); });
    }
    */
    printf("intersected polygon contours size: %zu\n", contours.size());
    // simply sum up all contours' area
    auto area = std::accumulate(contours.begin(), contours.end(), 0.0, [](double s, std::vector<Eigen::Vector2d> const& pts) { return s + PolygonArea(pts); });
    printf("intersected polygon contours plane area: %f\n", area);

    std::vector<Eigen::Vector3d> convex_hull_pts(convex_hull.size()), concave_hull_pts(concave_hull.size());
    std::transform(convex_hull.begin(), convex_hull.end(), convex_hull_pts.begin(), [=](Eigen::Vector2d const& p) { return Eigen::Vector3d(p(0), p(1), height); });
    std::transform(concave_hull.begin(), concave_hull.end(), concave_hull_pts.begin(), [=](Eigen::Vector2d const& p) { return Eigen::Vector3d(p(0), p(1), height); });
    std::vector<std::vector<Eigen::Vector3d>> contours_pts;
    for (auto const& v : contours) {
        std::vector<Eigen::Vector3d> cpts;
        cpts.clear();
        for (auto const& p : v) cpts.emplace_back(p(0), p(1), height);
        contours_pts.push_back(cpts);
    }
    auto convex_hull_pcd = std::make_shared<open3d::geometry::PointCloud>(convex_hull_pts);
    auto concave_hull_pcd = std::make_shared<open3d::geometry::PointCloud>(concave_hull_pts);
    convex_hull_pcd->PaintUniformColor({1, 0, 0});
    concave_hull_pcd->PaintUniformColor({0, 1, 0});
    auto contours_pcds = std::vector<std::shared_ptr<open3d::geometry::PointCloud>>(contours_pts.size());
    std::transform(contours_pts.begin(), contours_pts.end(), contours_pcds.begin(), [](std::vector<Eigen::Vector3d> const& ps) { return std::make_shared<open3d::geometry::PointCloud>(ps); });
    std::for_each(contours_pcds.begin(), contours_pcds.end(), [](std::shared_ptr<open3d::geometry::PointCloud> pcd) { pcd->PaintUniformColor({ 0, 0, 1 }); });
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> v_pcds(contours_pcds.begin(), contours_pcds.end()); v_pcds.push_back(concave_hull_pcd);
    open3d::visualization::DrawGeometries({ convex_hull_pcd }, "intersected plane convex", 1920, 1080);
    open3d::visualization::DrawGeometries({ convex_hull_pcd, concave_hull_pcd }, "intersected plane concave", 1920, 1080);
    open3d::visualization::DrawGeometries(v_pcds, "intersected plane contours", 1920, 1080);

    // triangulate contours
    std::vector<std::vector<Eigen::Vector3i>> triangles(contours.size());
    {
        TIME_BLOCK("- contours triangulation: ");
        std::transform(contours.begin(), contours.end(), triangles.begin(), [](std::vector<Eigen::Vector2d> const& c) { return PolygonTriangulation(c); });
    }
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> plane_meshes(triangles.size());
    std::transform(triangles.begin(), triangles.end(), contours_pts.begin(), plane_meshes.begin(), [](std::vector<Eigen::Vector3i> const& tris, std::vector<Eigen::Vector3d> const& verts) { return std::make_shared<open3d::geometry::TriangleMesh>(verts, tris); });
    double contours_mesh_area = std::accumulate(plane_meshes.begin(), plane_meshes.end(), 0.0, [](double s, std::shared_ptr<open3d::geometry::TriangleMesh> const& mesh) { return s + mesh->GetSurfaceArea(); });
    std::cout << "contours mesh area: " << contours_mesh_area << std::endl;
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> v_plane_meshes(plane_meshes.begin(), plane_meshes.end()); v_plane_meshes.push_back(concave_hull_pcd);
    open3d::visualization::DrawGeometries(v_plane_meshes, "contours plane mesh", 1920, 1080);

    // plot contours plane mesh with r_mesh
    auto p_r_mesh = std::make_shared<open3d::geometry::TriangleMesh>(std::move(r_mesh));
    v_plane_meshes.pop_back();
    v_plane_meshes.push_back(p_r_mesh);
    open3d::visualization::DrawGeometries({ v_plane_meshes }, "contours plane mesh + r_mesh", 1920, 1080);

    /*
    // combine contours mesh with r_mesh
    // contour pt ids mapping to intersected pt indices
    // FIXME: contours and contours_pt_ids are not 1 to 1 indexing due to contours are computed with ConcaveHull, so the result is incorrect; since earcut not return indices, need a hash map? 
    std::vector<std::vector<size_t>> contours_indices(contours_pt_ids.size());
    std::transform(contours_pt_ids.begin(), contours_pt_ids.end(), contours_indices.begin(), [&](std::vector<size_t> const& vi) { 
        std::vector<size_t> tmp(vi.size());
        std::transform(vi.begin(), vi.end(), tmp.begin(), [&](size_t index) { return r_tmp_mesh_intersected_pt_indices[index]; });
        return tmp; });
    //std::ofstream of1("contours_indices.txt");
    //std::for_each(contours_indices.begin(), contours_indices.end(), [&](std::vector<size_t> const& tmp) { std::copy(tmp.rbegin(), tmp.rend(), std::ostream_iterator<size_t>(of1, "\n")); });
    //of1.close();
    
    // triangles mapping to contours indices
    std::vector<Eigen::Vector3i> contours_triangles;
    contours_triangles.reserve(contours_indices[0].size());
    for (auto i = 0; i < contours_indices.size(); i++) {
        for (auto j = 0; j < contours_indices[i].size(); j += 3)
            if (int(contours_indices[i][j]) >= 0 && int(contours_indices[i][j + 1]) >= 0 && int(contours_indices[i][j + 2]) >= 0)  // may overflow
                contours_triangles.emplace_back(contours_indices[i][j], contours_indices[i][j + 1], contours_indices[i][j + 2]);
            else continue;
    }
    //std::ofstream of2("contours_triangles.txt");
    //std::copy(contours_triangles.rbegin(), contours_triangles.rend(), std::ostream_iterator<Eigen::Vector3i>(of2, "\n"));
    //of2.close();

    auto p_r_mesh = std::make_shared<open3d::geometry::TriangleMesh>(std::move(r_mesh));
    p_r_mesh->triangles_.insert(r_mesh.triangles_.end(), contours_triangles.begin(), contours_triangles.end());
    open3d::visualization::DrawGeometries({ p_r_mesh }, "r_mesh", 1920, 1080);
    */


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
