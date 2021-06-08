//
// Created by Harold on 2021/06/8.
//

#ifndef M_MATH_M_POLYGON_MESH_H
#define M_MATH_M_POLYGON_MESH_H

#include <open3d/Open3D.h>
#include <opencv2/imgproc.hpp>

#include "concaveman.hpp"
#include "earcut.hpp"

#include "m_voxelgrid2d.h"

// for earcut
namespace mapbox {
    namespace util {
        template <>
        struct nth<0, cv::Point2d> {
            inline static auto get(const cv::Point2d& t) {
                return t.x;
            };
        };
        template <>
        struct nth<1, cv::Point2d> {
            inline static auto get(const cv::Point2d& t) {
                return t.y;
            };
        };
    }
}

namespace M_MATH {
    namespace details {
        // contour of point ids
        std::vector<std::vector<size_t>> Contours(std::vector<cv::Point2d> const& points, size_t min_contour_points_num = 20) {
            // find four extreme points
            double x_min = points[0].x;
            double x_max = points[0].x;
            double y_min = points[0].y;
            double y_max = points[0].y;
            for (auto const& p : points) {
                if (p.x < x_min) x_min = p.x;
                if (p.x > x_max) x_max = p.x;
                if (p.y < y_min) y_min = p.y;
                if (p.y > y_max) y_max = p.y;
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
            // voxel grid
            auto vg = M_MATH::VoxelGrid<double>(rows, cols, vs, vs);
            auto img = vg.CreateImg(points, true, cv::Point2d{ grid_x_min, grid_y_min });
            // find contours
            cv::threshold(img, img, 127, 255, 0);
            std::vector<std::vector<cv::Point>> idx_contours;
            std::vector<cv::Vec4i> hierarchy;
            ///\note here only find the outer contours
            // TODO: according to contour hierarchy, find nested contours and find a way to make a ring-like mesh
            cv::findContours(img, idx_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            // find pt ids
            std::vector<std::vector<size_t>> pt_ids;
            for (auto i = 0; i < idx_contours.size(); i++) {
                std::vector<size_t> tmp;
                if (idx_contours[i].size() < min_contour_points_num) continue;
                for (auto j = 0; j < idx_contours[i].size(); j++) {
                    auto p_idx = idx_contours[i][j];
                    for (auto s : vg.m_voxels[p_idx.y][p_idx.x].m_pt_ids)  // notice: voxle[y][x] here
                        tmp.push_back(s);
                }
                pt_ids.push_back(tmp);
            }
            return pt_ids;
        }

        // concave hull
        std::vector<cv::Point2d> ConcaveHull(std::vector<cv::Point2d> const& polygon_pts) {
            cv::Mat mat(polygon_pts);
            mat.convertTo(mat, CV_32FC1);
            std::vector<int> indices;
            cv::convexHull(mat, indices, false, false);

            std::vector<std::array<double, 2>> pts(polygon_pts.size());
            std::transform(polygon_pts.begin(), polygon_pts.end(), pts.begin(), [](cv::Point2d const& p) { return std::array<double, 2>{p.x, p.y}; });
            auto concave_hull = M_MATH::CONCAVEMAN::concaveman<double, 16>(pts, indices);
            std::vector<cv::Point2d> res(concave_hull.size());
            std::transform(concave_hull.begin(), concave_hull.end(), res.begin(), [](std::array<double, 2> const& p) { return cv::Point2d{ p[0], p[1] }; });
            return res;
        }

        // polygon triangulation
        std::vector<Eigen::Vector3i> PolygonTriangulation(std::vector<cv::Point2d> const& pts) {
            auto indices = mapbox::earcut<size_t>(std::vector<std::vector<cv::Point2d>>{ pts });
            std::vector<Eigen::Vector3i> triangles;
            triangles.reserve(indices.size() / 3);
            for (auto i = 0; i < indices.size(); i += 3)
                triangles.emplace_back(indices[i], indices[i + 1], indices[i + 2]);
            return triangles;
        }
    }

    // plane points to contour polygon mesh (parallel to XOY plane, namely normal direction is Z direction)
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> PlanePointsToContourPolygonMesh(std::vector<Eigen::Vector3d> const& plane_pts, size_t min_contour_points_num = 20) {
        auto height = plane_pts[0](2);
        std::vector<cv::Point2d> cv_pts(plane_pts.size());
        std::transform(plane_pts.begin(), plane_pts.end(), cv_pts.begin(), [](Eigen::Vector3d const& p) { return cv::Point2d{ p(0), p(1) }; });
        std::vector<std::vector<size_t>> contours_pt_ids = details::Contours(cv_pts, min_contour_points_num);

        std::vector<std::vector<cv::Point2d>> contours;
        contours.reserve(contours_pt_ids.size());
        for (auto const& ids : contours_pt_ids) {
            std::vector<cv::Point2d> pts;
            pts.reserve((ids.size()));
            for (auto const& id : ids) pts.emplace_back(cv_pts[id]);
            contours.push_back(details::ConcaveHull(pts));
        }

        std::vector<std::vector<Eigen::Vector3d>> contours_pts;
        contours_pts.reserve(contours.size());
        for (auto const& v : contours) {
            std::vector<Eigen::Vector3d> cpts;
            cpts.reserve(v.size());
            for (auto const& p : v) cpts.emplace_back(p.x, p.y, height);
            contours_pts.push_back(cpts);
        }

        std::vector<std::vector<Eigen::Vector3i>> triangles(contours.size());
        std::transform(contours.begin(), contours.end(), triangles.begin(), [](std::vector<cv::Point2d> const& c) { return details::PolygonTriangulation(c); });

        std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> plane_meshes(triangles.size());
        std::transform(triangles.begin(), triangles.end(), contours_pts.begin(), plane_meshes.begin(), [](std::vector<Eigen::Vector3i> const& tris, std::vector<Eigen::Vector3d> const& verts) { return std::make_shared<open3d::geometry::TriangleMesh>(verts, tris); });

        return plane_meshes;
    }
}

#endif //M_MATH_M_POLYGON_MESH_H