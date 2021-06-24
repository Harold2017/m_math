//
// Created by Harold on 2021/6/24.
//

#include <iostream>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "m_mesh_split.h"
#include "m_mesh_vertices_filter.h"

#include "../utils/io/arg_parser_o3d.h"

#include "stopwatch.h"

// <img, pixel_size, pixelgrid of pt_ids>
std::tuple<cv::Mat, double, std::vector<std::vector<std::vector<size_t>>>> 
ProjectToCuttingPlane(std::vector<Eigen::Vector3d> const& points, double pixel_size = 0.0, size_t min_pixel_pts_num = 10) {
    // find four extreme points
    double x_min = points[0].x();
    double x_max = points[0].x();
    double y_min = points[0].y();
    double y_max = points[0].y();
    for (auto const& p : points) {
        if (p.x() < x_min) x_min = p.x();
        if (p.x() > x_max) x_max = p.x();
        if (p.y() < y_min) y_min = p.y();
        if (p.y() > y_max) y_max = p.y();
    }
    // estimate pixel size, assume points are even distributed
    double ps = pixel_size > 0.0 ? pixel_size : 2 * std::sqrt((x_max - x_min) * (y_max - y_min) / double(points.size()));
    // set pixelgrid a little larger than bounding box of points
    double grid_x_min = x_min - (x_max - x_min) * 0.1;
    double grid_y_min = y_min - (y_max - y_min) * 0.1;
    double grid_x_max = x_max + (x_max - x_min) * 0.1;
    double grid_y_max = y_max + (y_max - y_min) * 0.1;
    size_t rows = std::max(size_t(1), static_cast<size_t>((grid_y_max - grid_y_min) / ps));
    size_t cols = std::max(size_t(1), static_cast<size_t>((grid_x_max - grid_x_min) / ps));
    // pixel grid
    auto pg = std::vector<std::vector<std::vector<size_t>>>(rows, std::vector<std::vector<size_t>>(cols, std::vector<size_t>()));
    auto ips = 1.0 / ps;
    auto origin = Eigen::Vector2d{ grid_x_min, grid_y_min };
    for (auto i = 0; i < points.size(); i++) {
        auto ix = static_cast<size_t>(std::floor((points[i].x() - origin.x()) * ips));
        auto iy = static_cast<size_t>(std::floor((points[i].y() - origin.y()) * ips));
        pg[iy][ix].push_back(i);
    }
    cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (auto i = 0; i < rows; i++)
        for (auto j = 0; j < cols; j++)
            if (pg[i][j].size() >= min_pixel_pts_num)
                img.at<uchar>(i, j) = 255;
    return std::make_tuple(std::move(img), ps, std::move(pg));
}

// find outer-most contour (which has max area)
size_t OuterMostContour(std::vector<std::vector<cv::Point>> const& contours) {
    return std::distance(contours.begin(), std::max_element(contours.begin(), contours.end(), 
        [](std::vector<cv::Point> const& c1, std::vector<cv::Point> const& c2) { return cv::contourArea(c1) < cv::contourArea(c2); }));
}

double Perimeter(std::vector<cv::Point> const& contour) {
    return cv::arcLength(contour, true);
}

// horizontal, vertical
std::pair<double, double> FeretDiameter(std::vector<cv::Point> const& contour) {
    auto rect = cv::boundingRect(contour);
    return std::make_pair(rect.width, rect.height);
}

// center, radius
std::pair<cv::Point2d, double> EquivalentCircle(std::vector<cv::Point> const& contour) {
    auto area = cv::contourArea(contour);
    auto r = sqrt(area / CV_PI);
    auto moments = cv::moments(contour);
    auto c = cv::Point2d{ moments.m10 / moments.m00, moments.m01 / moments.m00 };
    return std::make_pair(c, r);
}

int main(int argc, char* argv[]) {
    // -f "filename": mesh_file
    // -z "1": z height threshold
    // -roip "{{0, 5}; {0, 15}; {15, 15}; {15, 5}}"
    // -roic "{{5, 5}; 10}"
    auto mesh = open3d::io::CreateMeshFromFile(M_ARG_PARSER::ParseAsString(argc, argv, "-f", ""), false);
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

    if (M_ARG_PARSER::OptionExists(argc, argv, "-roip") && M_ARG_PARSER::OptionExists(argc, argv, "-roic"))
    {
        std::cerr << "only support once one ROI" << std::endl;
        exit(1);
    }
    else if (M_ARG_PARSER::OptionExists(argc, argv, "-roip"))
    {
        auto str = M_ARG_PARSER::ParseAsString(argc, argv, "-roip", "");
        auto roi = M_ARG_PARSER::ParsePolyROI(str);
        auto indices = M_MATH::VerticesInsidePolygonROI(*mesh, roi);
        mesh = M_MATH::GetSubMeshFromVertices(*mesh, indices);
    }
    else if (M_ARG_PARSER::OptionExists(argc, argv, "-roic"))
    {
        auto str = M_ARG_PARSER::ParseAsString(argc, argv, "-roic", "");
        auto roi = M_ARG_PARSER::ParseCircleROI(str);
        auto indices = M_MATH::VerticesInsideCircleROI(*mesh, roi.first, roi.second);
        mesh = M_MATH::GetSubMeshFromVertices(*mesh, indices);
    }
    else
        std::cout << "whole mesh as ROI" << std::endl;

    //open3d::visualization::DrawGeometries({ mesh }, "ROI");

    // set print precision
    std::cout << std::fixed << std::setprecision(3);

    // empty mesh
    auto upper_mesh = std::make_shared<open3d::geometry::TriangleMesh>(std::vector<Eigen::Vector3d>{}, std::vector<Eigen::Vector3i>{});
    auto lower_mesh = std::make_shared<open3d::geometry::TriangleMesh>(std::vector<Eigen::Vector3d>{}, std::vector<Eigen::Vector3i>{});
    bool ret = M_MATH::MeshCut(*mesh, plane_center, plane_normal, *upper_mesh, *lower_mesh);
    // cutting plane not intersect with mesh bounding box
    if (!ret)
    {
        if (plane_center.z() < mesh->GetAxisAlignedBoundingBox().GetCenter().z())
            upper_mesh = mesh;
        else
            lower_mesh = mesh;
    }

    //open3d::visualization::DrawGeometries({ upper_mesh }, "cut mesh");

    // split upper_mesh into sub-meshes
    auto subparts = M_MATH::MeshSplit2(*upper_mesh, 1.0);
    std::cout << "sub-meshes number: " << subparts.size() << std::endl;

    auto selected_mesh = subparts[0];

    // project mesh vertices onto cutting plane
    auto img_and_pg = ProjectToCuttingPlane(selected_mesh->vertices_, 0.0, 1);
    auto img = std::move(std::get<0>(img_and_pg));
    auto pixel_size = std::get<1>(img_and_pg);
    auto pg = std::move(std::get<2>(img_and_pg));

    cv::namedWindow("projected img", cv::WINDOW_AUTOSIZE);
    cv::imshow("projected img", img);
    cv::waitKey();

    // compute cutting plane projected points area
    auto non_zero_pixel_num = cv::countNonZero(img);
    auto area = non_zero_pixel_num * pixel_size * pixel_size;
    std::cout << "cutting plane projected points area: " << area << std::endl;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    auto const& outer_most_contour = contours[OuterMostContour(contours)];
    auto perimeter = Perimeter(outer_most_contour);
    auto feret_diameter = FeretDiameter(outer_most_contour);
    auto equivalent_circle = EquivalentCircle(outer_most_contour);
    // need multiply pixel_size to obtain the physical dimension
    std::cout << "perimeter: " << perimeter * pixel_size << '\n'
              << "horizontal_feret_diameter: " << feret_diameter.first * pixel_size << '\n'
              << "vertical_feret_diameter: " << feret_diameter.second * pixel_size << '\n'
              << "circle_equivalent_radius: " << equivalent_circle.second * pixel_size << '\n'
              << std::endl;

    return 0;
}
