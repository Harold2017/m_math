//
// Created by Harold on 2021/8/22.
//

#include <open3d/Open3D.h>
#include <open3d/3rdparty/Eigen/Eigen>
#include <iostream>
#include <omp.h>
#include "../utils/io/arg_parser.h"
#include "../utils/io/filesystem.h"

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
    // display pcd
    open3d::visualization::DrawGeometries({ p_source }, "original source pcd");

    // down sample
    //p_source = p_source->VoxelDownSample(0.05);
    //std::cout << "downsampled point cloud size: " << p_source->points_.size() << std::endl;
    // display pcd
    //open3d::visualization::DrawGeometries({ p_source }, "downsampled source pcd");

    // paint pcd to green
    p_source->PaintUniformColor({0, 1, 0});

    // knn
    int knn = 30;
    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*p_source);
    std::vector<double> surface_variations;
    surface_variations.reserve(p_source->points_.size());
    // compute cov attrs of every point
#pragma omp parallel for
    for (auto i = 0; i < p_source->points_.size(); ++i) {
        auto pt = p_source->points_[i];
        std::vector<int> indices(knn);
        std::vector<double> distance2(knn);
        kdtree.SearchKNN(pt, knn, indices, distance2);
        std::vector<Eigen::Vector3d> points;
        points.reserve(knn);
        std::transform(indices.cbegin(), indices.cend(), std::back_inserter(points), [&](int i) { return p_source->points_[i]; });

        // construct cov matrix
        auto rows = points.size();
        Eigen::MatrixX3d pts_matrix = Eigen::Matrix3Xd::Map(points[0].data(), 3, rows).transpose();
        auto centered = pts_matrix.rowwise() - pts_matrix.colwise().mean();
        auto cov = (centered.adjoint() * centered) / (rows - 1);
        // compute eigen values
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
        solver.compute(cov);
        Eigen::Vector3d lambda = solver.eigenvalues();

        surface_variations.emplace_back(lambda(0)/ (lambda(0) + lambda(1) + lambda(2)));
    }

    // find statics of surface_variations
    auto minmax = std::minmax_element(surface_variations.cbegin(), surface_variations.cend());
    auto minSV = *minmax.first;
    auto maxSV = *minmax.second;
    auto mean = std::accumulate(surface_variations.cbegin(), surface_variations.cend(), 0.0) / surface_variations.size();
    auto std = sqrt(std::accumulate(surface_variations.cbegin(), surface_variations.cend(), 0.0, 
        [=](double x, double y) { return x + (y - mean) * (y - mean); }) / (surface_variations.size() - 1));

    std::cout << "minSV: " << minSV << ", maxSV: " << maxSV << ", mean: " << mean << ", std: " << std << std::endl;

    // FIXME: seems this way is useless (followed this [paper](https://github.com/denabazazian/Edge_Extraction))
    // display surface_variations
    open3d::visualization::ColorMapHot colormap;
    for (auto i = 0; i < surface_variations.size(); i++) {
        p_source->colors_[i] = colormap.GetColor((surface_variations[i] - minSV) / (maxSV - minSV));
    }
    open3d::visualization::DrawGeometries({ p_source }, "surface_variations");

    // use threshold to filter out edge points (TODO: howto?)
    double threshold = 0.03; //minSV + 0.003 * (2 * std - minSV) / 256;
    std::cout << "threshold: " << threshold << std::endl;

    std::vector<size_t> edge_pts_indices;
    edge_pts_indices.reserve(surface_variations.size()/10);
    for (auto i = 0; i < surface_variations.size(); ++i)
        if (surface_variations[i] > threshold)
            edge_pts_indices.push_back(i);
    edge_pts_indices.shrink_to_fit();

    std::cout << "edge points size: " << edge_pts_indices.size() << std::endl;

    // paint pcd to green
    p_source->PaintUniformColor({0, 1, 0});

    // paint edge points to red
    for(auto i : edge_pts_indices)
        p_source->colors_[i] = {1, 0, 0};

    // display pcd
    open3d::visualization::DrawGeometries({ p_source }, "source pcd with edge points highlighted");

    return 0;
}