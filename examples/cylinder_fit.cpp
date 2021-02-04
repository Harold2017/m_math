//
// Created by Harold on 2021/2/2.
//

#include <iostream>
#include <opencv2/viz.hpp>
#include "m_cylinder_fit.hpp"

/**
 * \fn generate cylinder points (spiral)
 * @tparam T
 * @param theta
 * @param phi
 * @param radius
 * @param height
 * @return points and direction
 */
template<typename T>
std::pair<std::vector<cv::Point3_<T>>, cv::Point3_<T>> cylinder_gen(T theta,
                                                                    T phi,
                                                                    T radius,
                                                                    cv::Point3_<T> const& center,
                                                                    size_t N) {
    auto rotation_matrix_from_axis_and_angle = [](cv::Point3_<T> const& dir, T angle) {
        auto x = dir.x;
        auto y = dir.y;
        auto z = dir.z;
        auto s = sin(angle);
        auto c = cos(angle);

        T arr[] = {c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s,
                   y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s,
                   z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)};
        cv::Matx<T, 3, 3> rot(arr);
        return rot;
    };

    auto M = rotation_matrix_from_axis_and_angle({0, 0, 1}, phi).dot(rotation_matrix_from_axis_and_angle({0, 1, 0}, theta));
    auto x = M * cv::Point3_<T>(1, 0, 0);
    auto y = M * cv::Point3_<T>(0, 1, 0);
    auto z = M * cv::Point3_<T>(0, 0, 1);

    auto delta = 10. / 180. * M_PI;

    std::vector<cv::Point3_<T>> pts;
    pts.reserve(N);
    for (auto i = 0; i < N; i++)
        pts.push_back(center + radius * (cos(i * delta) * x + sin(i * delta) * y + i * 0.1 * z));
    return std::make_pair(pts, z);
}


int main() {
    auto pts_dir = cylinder_gen(1.f, -0.3f, 10.f, {0.f, 0.f, 0.f}, 100);
    //std::cout << pts_dir.first << '\n';

    cv::Mat mat(pts_dir.first);
    //cv::FileStorage fs("cylinder.txt", cv::FileStorage::WRITE);
    //fs << "cylinder" << mat;
    //std::cout << fs.releaseAndGetString() << std::endl;

    cv::viz::Viz3d window; //creating a Viz window
    // display the coordinate origin (0,0,0)
    window.showWidget("coordinate", cv::viz::WCoordinateSystem(100));
    // display the 3D points in green
    window.showWidget("points", cv::viz::WCloud(mat, cv::viz::Color::green()));

    std::cout << "generated cylinder direction: " << pts_dir.second << '\n'
              << std::endl;

    ///\note eigenvector of covariance is not good for
    /// fitting of samples on a cylinder ring that is cut skewed relative to the cylinder axis
    // using covariance matrix
    auto cylinder = M_MATH::CylinderFit(pts_dir.first, true);
    std::cout << "cylinder using covariance: " << '\n'
              << "cylinder direction: " << cylinder.axis_dir<< '\n'
              << "cylinder origin: " << cylinder.axis_point << '\n'
              << "cylinder radius: " << cylinder.radius << '\n'
              << "cylinder height: " << cylinder.height << '\n'
              << "fitting error: " << cylinder.fitting_error << '\n'
              << std::endl;

    // display fitting cylinder
    auto point_1 = cv::Vec3d(cylinder.axis_point.x, cylinder.axis_point.y, cylinder.axis_point.z);
    auto point_2 = cv::Vec3d(cylinder.axis_point.x + cylinder.height * cylinder.axis_dir.x,
                             cylinder.axis_point.y + cylinder.height * cylinder.axis_dir.y,
                             cylinder.axis_point.z + cylinder.height * cylinder.axis_dir.z);
    window.showWidget("cylinder_covariance", cv::viz::WCylinder(point_1,
                                                     point_2,
                                                     double(cylinder.radius),
                                                     30,
                                                     cv::viz::Color::white()));

    // using hemisphere
    cylinder = M_MATH::CylinderFit(pts_dir.first);
    std::cout << "cylinder using covariance: " << '\n'
              << "cylinder direction: " << cylinder.axis_dir<< '\n'
              << "cylinder origin: " << cylinder.axis_point << '\n'
              << "cylinder radius: " << cylinder.radius << '\n'
              << "cylinder height: " << cylinder.height << '\n'
              << "fitting error: " << cylinder.fitting_error << '\n'
              << std::endl;

    // display fitting cylinder
    point_1 = cv::Vec3d(cylinder.axis_point.x, cylinder.axis_point.y, cylinder.axis_point.z);
    point_2 = cv::Vec3d(cylinder.axis_point.x + cylinder.height * cylinder.axis_dir.x,
                        cylinder.axis_point.y + cylinder.height * cylinder.axis_dir.y,
                        cylinder.axis_point.z + cylinder.height * cylinder.axis_dir.z);
    window.showWidget("cylinder_hemisphere", cv::viz::WCylinder(point_1,
                                                     point_2,
                                                     double(cylinder.radius),
                                                     30,
                                                     cv::viz::Color::blue()));

    // show window
    window.spin();

    return 0;
}
