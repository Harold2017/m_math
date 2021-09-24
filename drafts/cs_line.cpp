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
#include <thread>

#define IMG_WIDTH 1200
#define IMG_HEIGHT 600

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
    p_source->PaintUniformColor({ 0, 1, 0 });  // green


    Eigen::Vector2d dir, norm;
    std::vector<Eigen::Vector3d> plane_pts;
    plane_pts.reserve(p_source->points_.size());

    cv::namedWindow("line points", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("line points", IMG_WIDTH + 50, 20);
    // project point to line
    auto project_point_to_line = [&]() {
        auto extent = p_source->GetAxisAlignedBoundingBox().GetExtent();
        auto max_x = std::sqrt(extent.x() * extent.x() + extent.y() * extent.y());
        auto max_y = extent.z();
        auto img = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
        std::vector<cv::Point> line_pts;
        line_pts.reserve(plane_pts.size());
        for (auto const& pt : plane_pts) {
            auto x = pt.x() * dir.x() + pt.y() * dir.y();
            line_pts.emplace_back(x / max_x * IMG_WIDTH, IMG_HEIGHT - pt.z() / max_y * IMG_HEIGHT);  // (0, 0) at left-top
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
        cv::polylines(img, line_pts, false, cv::Scalar(255, 255, 255), 2, 8, 0);
        cv::imshow("line points", img);
        cv::waitKey(1);  // make it non-blocking
        img.release();
    };

    // pick two points to form a line (shift + left mouse)
    open3d::visualization::VisualizerWithVertexSelection vis;
    vis.CreateVisualizerWindow("PCD", IMG_WIDTH, IMG_HEIGHT, 50, 50, true);
    vis.AddGeometry(p_source);
    vis.RegisterSelectionChangedCallback([&]() {
        auto pts_indices = vis.GetPickedPoints();
        if (pts_indices.size() < 2)
            return;

        // clear former selection
        plane_pts.clear();
        p_source->PaintUniformColor({ 0, 1, 0 });  // green
        vis.UpdateGeometry();
        auto const& points = p_source->points_;
        auto p1 = points[pts_indices[0].index];
        auto p2 = points[pts_indices[1].index];
        // line direction
        auto abs = (p2 - p1).norm();
        dir = Eigen::Vector2d((p2.x() - p1.x()) / abs, (p2.y() - p1.y()) / abs);
        // line normal
        norm = Eigen::Vector2d((p1.y() - p2.y()) / abs, (p2.x() - p1.x()) / abs);
        // plane normal, plane center
        Eigen::Vector3d plane_normal(norm.x(), norm.y(), 0);
        Eigen::Vector3d plane_center(p1.x(), p1.y(), 0);
        // threshold
        double threshold = 0.01;
        std::vector<size_t> plane_pts_indices;
        plane_pts_indices.reserve(points.size());
        for (auto i = 0; i < points.size(); i++)
            if (std::abs((points[i] - plane_center).dot(plane_normal)) <= threshold) {
                plane_pts.push_back(points[i]);
                plane_pts_indices.push_back(i);
            }
        // draw plane points
        for (auto i : plane_pts_indices)
            p_source->colors_[i] = { 1, 0, 1 };  // red
        vis.UpdateGeometry();
        vis.ClearPickedPoints();
        project_point_to_line();
    });


    vis.Run();
    vis.DestroyVisualizerWindow();
    cv::destroyWindow("line points");

    return 0;
}