//
// Created by Harold on 2021/8/22.
//

// avoid max/min macro in Windows.h
#define NOMINMAX

#include "../utils/io/arg_parser.h"
#include "../utils/io/filesystem.h"
#include "m_plane_seg_open3d.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <omp.h>

template<typename DerivedA, typename DerivedB>
bool allclose(const Eigen::DenseBase<DerivedA>& a,
              const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& rtol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
              const typename DerivedA::RealScalar& atol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs()
          <= (atol + rtol * b.derived().array().abs())).all();
}

// quaternion
Eigen::Matrix3d FindRotationMatrix(Eigen::Vector3d const &in_normal, Eigen::Vector3d const &out_normal) {
    Eigen::Vector3d u = in_normal.cross(out_normal).normalized();
    double theta_half = acos(in_normal.dot(out_normal)) / 2.0;
    // rotation quaternion (a, bi, cj, dk)
    double a = cos(theta_half);
    double b = sin(theta_half) * u.x();
    double c = sin(theta_half) * u.y();
    double d = sin(theta_half) * u.z();

    Eigen::Matrix3d rot_mat;
    rot_mat(0, 0) = (1 - 2 * c * c - 2 * d * d);
    rot_mat(0, 1) = (2 * b * c - 2 * a * d);
    rot_mat(0, 2) = (2 * a * c + 2 * b * d);
    rot_mat(1, 0) = (2 * b * c + 2 * a * d);
    rot_mat(1, 1) = (1 - 2 * b * b - 2 * d * d);
    rot_mat(1, 2) = (2 * c * d - 2 * a * b);
    rot_mat(2, 0) = (2 * b * d - 2 * a * c);
    rot_mat(2, 1) = (2 * a * b + 2 * c * d);
    rot_mat(2, 2) = (1 - 2 * b * b - 2 * c * c);
    return rot_mat;
}

// z = 0
std::vector<Eigen::Vector3d> TransformPlanePointsToXOY(M_MATH::PlaneInfo const& plane_info) {
    // rotate
    auto o_norm = plane_info.plane_normal.normalized();
    auto n_norm = Eigen::Vector3d(0, 0, 1);
    auto plane_center = plane_info.plane_center;
    auto points = plane_info.plane_pts;
    // no need to rotate, translate only
    if (allclose(o_norm, n_norm))
#pragma omp parallel for
		for (auto i = 0; i < points.size(); i++)
			points[i] -= plane_center;
    // need rotate and translate
    else {
        auto rot_mat = FindRotationMatrix(o_norm, n_norm);
#pragma omp parallel for
		for (auto i = 0; i < points.size(); i++) {
			points[i] -= plane_center;
			points[i] = rot_mat * points[i];
        }
    }
    return points;
}

// img, origin
std::pair<cv::Mat, Eigen::Vector2d> PlaneToImage(M_MATH::PlaneInfo const& plane_info, double pixel_size = 0.0, size_t min_pixel_pts_num = 1) {
    auto points = TransformPlanePointsToXOY(plane_info);
    // find four extreme points
    double x_min = points[0].x();
    double x_max = points[0].x();
    double y_min = points[0].y();
    double y_max = points[0].y();
    for (auto const &p : points) {
        if (p.x() < x_min)
            x_min = p.x();
        if (p.x() > x_max)
            x_max = p.x();
        if (p.y() < y_min)
            y_min = p.y();
        if (p.y() > y_max)
            y_max = p.y();
    }
    // estimate pixel size, assume points are even distributed
    double ps = pixel_size > 0.0 ? pixel_size : 2 * std::sqrt((x_max - x_min) * (y_max - y_min) / double(points.size()));
    // set pixelgrid a little larger than bounding box of points
    double grid_x_min = x_min - (x_max - x_min) * 0.1;
    double grid_y_min = y_min - (y_max - y_min) * 0.1;
    double grid_x_max = x_max + (x_max - x_min) * 0.1;
    double grid_y_max = y_max + (y_max - y_min) * 0.1;
    size_t rows = (std::max)(size_t(1), static_cast<size_t>((grid_y_max - grid_y_min) / ps));
    size_t cols = (std::max)(size_t(1), static_cast<size_t>((grid_x_max - grid_x_min) / ps));
    // pixel grid
    auto pg = std::vector<std::vector<std::vector<size_t>>>(rows, std::vector<std::vector<size_t>>(cols, std::vector<size_t>()));
    auto ips = 1.0 / ps;
    auto origin = Eigen::Vector2d{grid_x_min, grid_y_min};
    for (auto i = 0; i < points.size(); i++) {
        auto ix = static_cast<size_t>(std::floor((points[i].x() - origin.x()) * ips));
        auto iy = static_cast<size_t>(std::floor((points[i].y() - origin.y()) * ips));
        pg[iy][ix].push_back(i);
    }
    cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC1);
    // compute and use intensity (points number in grid) as pixel value
    size_t max_pts_in_voxel = 0;
    for (auto i = 0; i < rows; i++)
        for (auto j = 0; j < cols; j++)
            max_pts_in_voxel = max_pts_in_voxel < pg[i][j].size() ? pg[i][j].size() : max_pts_in_voxel;
    for (auto i = 0; i < rows; i++)
        for (auto j = 0; j < cols; j++)
            img.at<uchar>(i, j) = static_cast<uchar>(255 * pg[i][j].size() / max_pts_in_voxel);
    return std::make_pair(std::move(img), std::move(origin));
}

// z = 0
Eigen::Vector3d TransformXOYPointToPlane(Eigen::Vector2d const& point_2d, Eigen::Vector3d const& plane_normal, Eigen::Vector3d const& plane_center) {
    Eigen::Vector3d point{ point_2d(0), point_2d(1), 0. };
    auto o_norm = Eigen::Vector3d(0, 0, 1);
    auto n_norm = plane_normal.normalized();
    // rotate
    if (!allclose(o_norm, n_norm)) {
        auto rot_mat = FindRotationMatrix(o_norm, n_norm);
        point = rot_mat * point;
    }
    // translate
    return point + plane_center;
}

// extract lineset from plane
std::shared_ptr<open3d::geometry::LineSet> PlaneToLineSet(M_MATH::PlaneInfo const& plane_info, double voxel_size = 0.0, size_t min_pixel_pts_num = 1) {
    // convert plane into image
    auto img_and_origin = PlaneToImage(plane_info, voxel_size);
    auto img = img_and_origin.first;
    auto origin = img_and_origin.second;
    //cv::imshow("img", img);
    //cv::waitKey();

    /*
    // LSD implementation is [removed](https://github.com/opencv/opencv_contrib/issues/2524#issue-615242133)
    cv::Ptr<cv::LineSegmentDetector> det;
    det = cv::createLineSegmentDetector();
    cv::Mat lines;
    det->detect(img, lines);
    det->drawSegments(img, lines);
    cv::imshow("lines", img);
    cv::waitKey();
    */

    // use houghline to detect lines
    cv::GaussianBlur(img, img, { 5, 5 }, 0);
    cv::Mat edges;
    cv::Canny(img, edges, 50, 150);
    std::vector<cv::Vec4i> linesP;
    cv::HoughLinesP(edges, linesP, 1, CV_PI/180, 50, 50, 10);

    /*
    // draw lines
    cv::Mat dst;
    cv::cvtColor(img, dst, cv::COLOR_GRAY2BGR);
    for (auto i = 0; i < linesP.size(); i++) {
        cv::Vec4i l = linesP[i];
        cv::line(dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }
    cv::imshow("lines", dst);
    cv::waitKey();
    */

    // transform 2d line to 3d <start_pt, end_point>
    std::vector<Eigen::Vector3d> _points;
    std::vector<Eigen::Vector2i> _lines;
    _points.reserve(linesP.size() * 2);
    _lines.reserve(linesP.size());
    for (auto i = 0; i < linesP.size(); i++) {
        // FIXME: translate line point back to global coordinate NOT so accurate
        _points.emplace_back(TransformXOYPointToPlane({ linesP[i][0] * voxel_size + origin.x() + voxel_size / 2,  // [0, cols - 1]
                                                        (img.rows - linesP[i][1]) * voxel_size + origin.y() - voxel_size / 2 },  // [0, rows - 1] 
                                                      plane_info.plane_normal, 
                                                      plane_info.plane_center));
        _points.emplace_back(TransformXOYPointToPlane({ linesP[i][2] * voxel_size + origin.x() + voxel_size / 2, 
                                                        (img.rows - linesP[i][3]) * voxel_size + origin.y() - voxel_size / 2 }, 
                                                      plane_info.plane_normal, 
                                                      plane_info.plane_center));
        _lines.emplace_back(Eigen::Vector2i(2 * i, 2 * i + 1));
    }
    return std::make_shared<open3d::geometry::LineSet>(_points, _lines);
}

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
    std::cout << "original point cloud size: " << p_source->points_.size() << std::endl;


    // plane segmentation
    double voxel_size = 0.05;
    int min_plane_points = 20;
    double min_left_points_ratio = 0.01;

    // remove nan/inf
    p_source->RemoveNonFinitePoints(true, true);
    // down sample
    p_source = p_source->VoxelDownSample(voxel_size);
    // remove point clouds noise using statitical noise removal method
    p_source->RemoveStatisticalOutliers(20, 2.0);

    std::cout << "preprocessed point cloud size: " << p_source->points_.size() << std::endl;
    // display pcd
    p_source->PaintUniformColor({ 0, 1, 0 });
    open3d::visualization::DrawGeometries({ p_source }, "preprocessed pcd");

    auto planes = M_MATH::MultiplePlaneSeg(*p_source, min_left_points_ratio, voxel_size * 3, min_plane_points, 200);
    auto N = planes.size();
    std::cout << "planes number: " << N << std::endl;
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> pgeos;
    open3d::visualization::ColorMapJet cm;
    for (auto i = 0; i < N; i++) {
        auto ppl = std::make_shared<open3d::geometry::PointCloud>(planes[i].plane_pts);
        ppl->PaintUniformColor(cm.GetColor(double(i) / double(N)));
        pgeos.push_back(ppl);
    }
    open3d::visualization::DrawGeometries(pgeos);

    // extract 3d lines and display
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> _pgeos;
    _pgeos.push_back(p_source);
    for (auto i = 0; i < N; i++) {
        auto pls = PlaneToLineSet(planes[i], voxel_size);
        pls->PaintUniformColor({ 1, 0, 1 });
        _pgeos.push_back(pls);
    }
    open3d::visualization::DrawGeometries(_pgeos, "preprocessed pcd with lines");

    return 0;
}