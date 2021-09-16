//
// Created by Harold on 2021/9/16.
//

// avoid max/min macro in Windows.h
#define NOMINMAX

#include <open3d/Open3D.h>
#include "../utils/io/arg_parser.h"
#include "../utils/io/filesystem.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char* argv[]) {
    // pcd file io
    auto in_file = M_ARG_PARSER::ParseAsString(argc, argv, "-i", "");
    if (in_file.empty()) {
        std::cerr << "invalid input filename: " << in_file << std::endl;
        exit(1);
    }
    auto in_ext = M_FILESYSTEM::GetFileExtensionLower(in_file);
    auto p_source = open3d::io::CreatePointCloudFromFile(in_file, in_ext);
    if (!p_source) {
        std::cerr << "can not read in file: " << in_file << std::endl;
        exit(1);
    }

    // pick two points to form a line (shift + left mouse)
    open3d::visualization::VisualizerWithEditing vis;
    vis.CreateVisualizerWindow();
    vis.AddGeometry(p_source);
    vis.Run();
    vis.DestroyVisualizerWindow();
    auto pts_indices = vis.GetPickedPoints();

    auto const& points = p_source->points_;
    auto p1 = points[pts_indices[0]];
    auto p2 = points[pts_indices[1]];
    // line direction
    auto abs = (p2 - p1).norm();
    Eigen::Vector2d dir((p2.x() - p1.x()) / abs, (p2.y() - p1.y()) / abs);
    // line normal
    Eigen::Vector2d norm((p1.y() - p2.y()) / abs, (p2.x() - p1.x()) / abs);
    // plane normal, plane center
    Eigen::Vector3d plane_normal(norm.x(), norm.y(), 0);
    Eigen::Vector3d plane_center(p1.x(), p1.y(), 0);
    // threshold
    double threshold = 0.01;
    std::vector<Eigen::Vector3d> plane_pts;
    std::vector<size_t> plane_pts_indices;
    plane_pts.reserve(points.size());
    plane_pts_indices.reserve(points.size());
    for (auto i = 0; i < points.size(); i++)
        if (std::abs((points[i] - plane_center).dot(plane_normal)) <= threshold) {
            plane_pts.push_back(points[i]);
            plane_pts_indices.push_back(i);
        }
    // draw plane points
    p_source->PaintUniformColor({ 0, 1, 0 });  // green
    for (auto i : plane_pts_indices)
        p_source->colors_[i] = { 1, 0, 1 };  // red
    open3d::visualization::DrawGeometries({ p_source });

    // project point to line
    auto extent = p_source->GetAxisAlignedBoundingBox().GetExtent();
    auto max_x = std::sqrt(extent.x() * extent.x() + extent.y() * extent.y());
    auto max_y = extent.z();
    int img_width = 1200, img_height = 600;
    std::vector<cv::Point> line_pts;
    line_pts.reserve(plane_pts.size());
    for (auto const& pt : plane_pts) {
        auto x = pt.x() * dir.x() + pt.y() * dir.y();
        line_pts.emplace_back(x / max_x * img_width, img_height - pt.z() / max_y * img_height);  // (0, 0) at left-top
    }
    // sort and remove duplicates
    std::sort(line_pts.begin(), line_pts.end(), 
        [](cv::Point const& pt1, cv::Point const& pt2) {
            return pt1.x < pt2.x;
        });
    line_pts.erase(std::unique(line_pts.begin(), line_pts.end()), line_pts.end());
    line_pts.shrink_to_fit();
    // make x axis starts from 0
    auto tmp_x = line_pts[0].x;
    std::for_each(line_pts.begin(), line_pts.end(), [=](cv::Point& pt) { pt.x -= tmp_x; });
    line_pts.shrink_to_fit();
    // draw polylines
    auto img = cv::Mat(img_height, img_width, CV_8UC3);
    cv::polylines(img, line_pts, false, cv::Scalar(255, 255, 255), 2, 8, 0);
    cv::imshow("line points", img);
    cv::waitKey();

    return 0;
}