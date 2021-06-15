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

//#define DEBUG_PRINT_AND_PLOT
#ifdef DEBUG_PRINT_AND_PLOT
#include <opencv2/highgui.hpp>
#endif

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
        // pt ids of closed contours groups
        // for each contour group: first main contour, then holes
        std::vector<std::vector<std::vector<size_t>>>
            ContoursGroups(std::vector<cv::Point2d> const& points, double voxel_size = 0.0, size_t min_contour_points_num = 20, bool use_external_contour_only = false) {
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
            double vs = voxel_size > 0.0 ? voxel_size : 2 * std::sqrt((x_max - x_min) * (y_max - y_min) / double(points.size()));
#ifdef DEBUG_PRINT_AND_PLOT
            printf("voxel_size: %f\n", vs);
#endif
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
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;

            if (use_external_contour_only) cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            // according to contour tree hierarchy, find nested contours
            else cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

#ifdef DEBUG_PRINT_AND_PLOT
            //printf("total contours number: %zu\n", contours.size());
            std::cout << "contours hierarchy: \n";
            std::copy(hierarchy.begin(), hierarchy.end(), std::ostream_iterator<cv::Vec4i>(std::cout));
            std::cout << std::endl;

            // plot contours
            cv::Mat img2(800, 800, CV_8UC3, cv::Scalar(0));
            cv::drawContours(img2, contours, -1, CV_RGB(255, 255, 255), 1);
            cv::namedWindow("contours");
            cv::imshow("contours", img2);
            cv::waitKey();
#endif

            // FIXME: complex contours' hierarchy may be INCORRECT, better use cv::RETR_EXTERNAL?
                        // closed contours groups: outer contour -> inside holes
            std::map<size_t, std::vector<size_t>> contours_indices_groups;  // contour idx : holes...  (map -> index from outer-most(0) to inner-most(x))

            if (use_external_contour_only) {
                for (auto i = 0; i < contours.size(); i++)
                    contours_indices_groups[i] = std::vector<size_t>();
            }
            else {
                for (auto i = 0; i < contours.size(); i++)
                    // filter out closed contours
                    if (hierarchy[i][2] < 0) continue;  // open contours (has no child)
                    else {  // closed contours
                        contours_indices_groups[i] = std::vector<size_t>();
                        if (hierarchy[hierarchy[i][2]][2] >= 0) {  // closed contour and has grand closed contours
                            auto grandson_index = hierarchy[hierarchy[i][2]][2];
                            contours_indices_groups[i].push_back(grandson_index);
                            auto next_grandson_index = hierarchy[hierarchy[hierarchy[i][2]][2]][0];
                            while (next_grandson_index >= 0) {
                                contours_indices_groups[i].push_back(next_grandson_index);
                                next_grandson_index = hierarchy[next_grandson_index][0];
                            }
                        }
                        // skip inner contour (first child)
                        i += 1;
                    }

#ifdef DEBUG_PRINT_AND_PLOT
                //printf("contours_indices_groups size: %zu\n", contours_indices_groups.size());
#endif

                // correct the group relationship
                std::vector<bool> computed(contours_indices_groups.size(), false);
                for (auto it = contours_indices_groups.cbegin(); it != contours_indices_groups.cend();) {
                    if (computed[it->first])
                        it = contours_indices_groups.erase(it);
                    else {
                        computed[it->first] = true;
                        for (auto i : it->second)
                            computed[i] = true;
                        ++it;
                    }
                }
            }

#ifdef DEBUG_PRINT_AND_PLOT
            // print contours_indices_groups
            std::cout << "contours_indices_groups (contour area)\n";
            for (auto const& e : contours_indices_groups) {
                std::cout << e.first << " (" << cv::contourArea(contours[e.first]) << "): ";
                for (auto x : e.second)
                    std::cout << x << " (" << cv::contourArea(contours[x]) << ")  ";
                std::cout << '\n';
            }
            std::cout << std::endl;

            // plot contours
            cv::namedWindow("contours");
            img2 = cv::Scalar(0);
            cv::RNG rng(time(0));
            for (auto const& e : contours_indices_groups) {
                img2 = cv::Scalar(0);
                cv::drawContours(img2, contours, e.first, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 1);
                for (auto x : e.second)
                   cv::drawContours(img2, contours, x, CV_RGB(255, 255, 255), 1);
                cv::imshow("contours", img2);
                cv::waitKey();
            }
            cv::drawContours(img2, contours, -1, CV_RGB(255, 255, 255), 1);
            cv::imshow("contours", img2);
            cv::waitKey();
#endif

            // contours groups pt ids
            std::vector<std::vector<std::vector<size_t>>> contours_groups_pt_ids;
            contours_groups_pt_ids.reserve(contours_indices_groups.size());
            for (auto const& p : contours_indices_groups) {
                std::vector<size_t> tmp;
                tmp.reserve(1 + p.second.size());
                tmp.push_back(p.first);  // main contour
                tmp.insert(tmp.end(), p.second.begin(), p.second.end());  // holes
                std::vector<std::vector<size_t>> pt_ids;
                pt_ids.reserve(tmp.size());
                for (auto i : tmp) {
                    std::vector<size_t> ids;
                    ids.reserve(contours[i].size());  // better use a scale factor for reserve?
                    for (auto j = 0; j < contours[i].size(); j++) {
                        auto p_idx = contours[i][j];
                        for (auto s : vg.m_voxels[p_idx.y][p_idx.x].m_pt_ids)  // notice: voxle[y][x] here
                            ids.push_back(s);
                    }
                    // skip contours which not have enough number of points
                    if (ids.size() < min_contour_points_num) continue;
                    pt_ids.push_back(ids);
                }
                contours_groups_pt_ids.push_back(pt_ids);
            }
            //printf("contours_groups_pt_ids size: %zu\n", contours_groups_pt_ids.size());
            return contours_groups_pt_ids;
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
        std::vector<Eigen::Vector3i> PolygonTriangulation(std::vector<std::vector<cv::Point2d>> const& pts) {
            auto indices = mapbox::earcut<size_t>(pts);
            std::vector<Eigen::Vector3i> triangles;
            triangles.reserve(indices.size() / 3);
            for (auto i = 0; i < indices.size(); i += 3)
                triangles.emplace_back(indices[i], indices[i + 1], indices[i + 2]);
            return triangles;
        }
    }

    // plane points to contour polygon point cloud (parallel to XOY plane, namely normal direction is Z direction)
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> PlanePointsToContourPolygonPCD(std::vector<Eigen::Vector3d> const& plane_pts, double voxel_size = 0.0, size_t min_contour_points_num = 20, bool use_external_contour_only = false) {
        auto height = plane_pts[0](2);
        std::vector<cv::Point2d> cv_pts(plane_pts.size());
        std::transform(plane_pts.begin(), plane_pts.end(), cv_pts.begin(), [](Eigen::Vector3d const& p) { return cv::Point2d{ p(0), p(1) }; });
        auto contours_groups = details::ContoursGroups(cv_pts, voxel_size, min_contour_points_num, use_external_contour_only);

        std::vector<std::vector<std::vector<cv::Point2d>>> contours_groups_concave_hulls;
        contours_groups_concave_hulls.reserve(contours_groups.size());
        for (auto const& cg : contours_groups) {
            std::vector<std::vector<cv::Point2d>> contours;
            contours.reserve(cg.size());
            for (auto const& c : cg) {
                std::vector<cv::Point2d> contour;
                contour.reserve(c.size());
                for (auto const& id : c) contour.emplace_back(cv_pts[id]);
                contours.push_back(details::ConcaveHull(contour));
            }
            contours_groups_concave_hulls.push_back(contours);
        }

        std::vector<std::vector<Eigen::Vector3d>> contours_groups_pts;
        contours_groups_pts.reserve(contours_groups_concave_hulls.size());
        for (auto const& cg : contours_groups_concave_hulls) {
            std::vector<Eigen::Vector3d> cpts;
            auto size = std::accumulate(cg.begin(), cg.end(), size_t(0), [](size_t s, std::vector<cv::Point2d> const& cs) { return s + cs.size(); });
            cpts.reserve(size);
            for (auto const& c : cg) for (auto const& p : c) cpts.emplace_back(p.x, p.y, height);
            contours_groups_pts.push_back(cpts);
        }

        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> plane_pcds(contours_groups_pts.size());
        std::transform(contours_groups_pts.begin(), contours_groups_pts.end(), plane_pcds.begin(), [](std::vector<Eigen::Vector3d> const& verts) { return std::make_shared<open3d::geometry::PointCloud>(verts); });

        return plane_pcds;
    }

    // plane points to contour polygon mesh (parallel to XOY plane, namely normal direction is Z direction)
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> PlanePointsToContourPolygonMesh(std::vector<Eigen::Vector3d> const& plane_pts, double voxel_size = 0.0, size_t min_contour_points_num = 20, bool use_external_contour_only = false) {
        auto height = plane_pts[0](2);
        std::vector<cv::Point2d> cv_pts(plane_pts.size());
        std::transform(plane_pts.begin(), plane_pts.end(), cv_pts.begin(), [](Eigen::Vector3d const& p) { return cv::Point2d{ p(0), p(1) }; });
        auto contours_groups = details::ContoursGroups(cv_pts, voxel_size, min_contour_points_num, use_external_contour_only);

        std::vector<std::vector<std::vector<cv::Point2d>>> contours_groups_concave_hulls;
        contours_groups_concave_hulls.reserve(contours_groups.size());
        for (auto const& cg : contours_groups) {
            std::vector<std::vector<cv::Point2d>> contours;
            contours.reserve(cg.size());
            for (auto const& c : cg) {
                std::vector<cv::Point2d> contour;
                contour.reserve(c.size());
                for (auto const& id : c) contour.emplace_back(cv_pts[id]);
                contours.push_back(details::ConcaveHull(contour));
            }
            contours_groups_concave_hulls.push_back(contours);
        }

        std::vector<std::vector<Eigen::Vector3d>> contours_groups_pts;
        contours_groups_pts.reserve(contours_groups_concave_hulls.size());
        for (auto const& cg : contours_groups_concave_hulls) {
            std::vector<Eigen::Vector3d> cpts;
            auto size = std::accumulate(cg.begin(), cg.end(), size_t(0), [](size_t s, std::vector<cv::Point2d> const& cs) { return s + cs.size(); });
            cpts.reserve(size);
            for (auto const& c : cg) for (auto const& p : c) cpts.emplace_back(p.x, p.y, height);
            contours_groups_pts.push_back(cpts);
        }

        std::vector<std::vector<Eigen::Vector3i>> triangles(contours_groups_pts.size());
        std::transform(contours_groups_concave_hulls.begin(), contours_groups_concave_hulls.end(), triangles.begin(), [](std::vector<std::vector<cv::Point2d>> const& c) { return details::PolygonTriangulation(c); });

        std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> plane_meshes(triangles.size());
        std::transform(triangles.begin(), triangles.end(), contours_groups_pts.begin(), plane_meshes.begin(), [](std::vector<Eigen::Vector3i> const& tris, std::vector<Eigen::Vector3d> const& verts) { return std::make_shared<open3d::geometry::TriangleMesh>(verts, tris); });

        return plane_meshes;
    }
}

#endif //M_MATH_M_POLYGON_MESH_H