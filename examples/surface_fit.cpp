//
// Created by Harold on 2021/2/2.
//

#include <iostream>
#include <random>
#include "m_surface_fit.hpp"

double get_random()
{
    static std::default_random_engine e;
    static std::uniform_real_distribution<> dis(0, 1); // [0 - 1]
    return dis(e);
}

// https://www.wikiwand.com/en/Ellipsoid
Eigen::Matrix<double, Eigen::Dynamic, 3> ellipsoid_gen(Eigen::Vector3d const& center,
                                                       Eigen::Vector3d const& radii,
                                                       size_t samples) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> points;
    points.resize(samples, 3);

    for (size_t i = 0; i < samples; ++i) {
        Eigen::Vector3d point;
        double theta = (get_random() / 2. - 1.) * M_PI; // [-pi/2, pi/2]
        double phi = (get_random() * 2. - 1.) * M_PI; // [-pi,  pi]

        point.x() = center.x() + radii.x() * std::cos(theta) * std::cos(phi);
        point.y() = center.y() + radii.y() * std::cos(theta) * std::sin(phi);
        point.z() = center.z() + radii.z() * std::sin(theta);

        points.row(i) = point.transpose();
    }

    return points;
}


int main() {
    Eigen::Vector3d center, radii;
    center << 1, 2, 3;
    radii << 4, 5, 6;

    auto points = ellipsoid_gen(center, radii, 100);

    //std::cout << points << std::endl;

    std::vector<cv::Point3f> pts;
    pts.resize(points.rows());
    for (auto i = 0; i < points.rows(); i++) {
        pts[i].x = points.row(i).x();
        pts[i].y = points.row(i).y();
        pts[i].z = points.row(i).z();
    }

    //std::cout << pts << std::endl;

    auto ellipsoid = M_MATH::EllipsoidFit(pts);

    std::cout << "center: " << ellipsoid.first << '\n'
              << "radii: " << ellipsoid.second
              << std::endl;

    return 0;
}
