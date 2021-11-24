//
// Created by Harold on 2021/11/24.
//

#include <open3d/Open3D.h>
#include <open3d/3rdparty/Eigen/Geometry>

#include <fstream>
#include <iostream>
#include <sstream>

#define MY_PI 3.14159265358979323846

struct Camera
{
    Eigen::Affine3f projection;
    Eigen::Affine3f cam2world;
    Eigen::Vector3f world_pos;
    Eigen::Vector3f world_euler;
};

std::ifstream& go_to_line(std::ifstream& file, unsigned int num);
bool read_cam(std::string filename, Camera& cam);
std::ostream& operator<<(std::ostream& os, Camera const& cam);

Eigen::Matrix3f create_rotation_matrix(float ax, float ay, float az);
Eigen::Matrix3f create_rotation_matrix(Eigen::Vector3f const& euler);

int main(int argc, char* argv[])
{
    Camera cam;
    read_cam(argv[1], cam);
    std::cout << cam << std::endl;

    auto euler = cam.world_euler;
    // unity is left handed, need transfer it into right handed
    // https://stackoverflow.com/questions/31191752/right-handed-euler-angles-xyz-to-left-handed-euler-angles-xyz
    euler.x() *= -1;
    euler.y() *= -1;
    auto rot = create_rotation_matrix(euler);
    std::cout << "Rotation Matrix from Euler:\n" << rot << std::endl;

    Eigen::Affine3f world2cam;
    world2cam.linear() = rot;
    world2cam.translation() = cam.cam2world.translation();
    world2cam.translation().z() *= -1;  // LHC to RHC
    std::cout << "WorldToCamera Matrix: \n" << world2cam.matrix() << std::endl;

    return 0;
}

std::ifstream& go_to_line(std::ifstream& file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for (int i = 0; i < num - 1; ++i)
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return (file);
}

bool read_cam(std::string filename, Camera& cam)
{
    std::ifstream myReadFile;
    myReadFile.open(filename.c_str(), std::ios::in);
    if (!myReadFile.is_open())
    {
        std::cerr << "Error opening file: " << filename.c_str() << std::endl;
        return false;
    }
    myReadFile.seekg(std::ios::beg);
    float val;
    char c;

    // projection matrix
    go_to_line(myReadFile, 3);
    myReadFile >> val;
    cam.projection.matrix()(0, 0) = val;
    myReadFile >> val;
    cam.projection.matrix()(0, 1) = val;
    myReadFile >> val;
    cam.projection.matrix()(0, 2) = val;
    myReadFile >> val;
    cam.projection.matrix()(0, 3) = val;

    go_to_line(myReadFile, 4);
    myReadFile >> val;
    cam.projection.matrix()(1, 0) = val;
    myReadFile >> val;
    cam.projection.matrix()(1, 1) = val;
    myReadFile >> val;
    cam.projection.matrix()(1, 2) = val;
    myReadFile >> val;
    cam.projection.matrix()(1, 3) = val;

    go_to_line(myReadFile, 5);
    myReadFile >> val;
    cam.projection.matrix()(2, 0) = val;
    myReadFile >> val;
    cam.projection.matrix()(2, 1) = val;
    myReadFile >> val;
    cam.projection.matrix()(2, 2) = val;
    myReadFile >> val;
    cam.projection.matrix()(2, 3) = val;

    go_to_line(myReadFile, 6);
    myReadFile >> val;
    cam.projection.matrix()(3, 0) = val;
    myReadFile >> val;
    cam.projection.matrix()(3, 1) = val;
    myReadFile >> val;
    cam.projection.matrix()(3, 2) = val;
    myReadFile >> val;
    cam.projection.matrix()(3, 3) = val;


    // camera_to_world matrix
    go_to_line(myReadFile, 9);
    myReadFile >> val;
    cam.cam2world.matrix()(0, 0) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(0, 1) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(0, 2) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(0, 3) = val;

    go_to_line(myReadFile, 10);
    myReadFile >> val;
    cam.cam2world.matrix()(1, 0) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(1, 1) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(1, 2) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(1, 3) = val;

    go_to_line(myReadFile, 11);
    myReadFile >> val;
    cam.cam2world.matrix()(2, 0) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(2, 1) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(2, 2) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(2, 3) = val;

    go_to_line(myReadFile, 12);
    myReadFile >> val;
    cam.cam2world.matrix()(3, 0) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(3, 1) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(3, 2) = val;
    myReadFile >> val;
    cam.cam2world.matrix()(3, 3) = val;

    // world position
    go_to_line(myReadFile, 15);
    myReadFile >> c;
    myReadFile >> val;
    cam.world_pos(0) = val;
    myReadFile >> c;
    myReadFile >> val;
    cam.world_pos(1) = val;
    myReadFile >> c;
    myReadFile >> val;
    cam.world_pos(2) = val;

    // world euler
    go_to_line(myReadFile, 17);
    myReadFile >> c;
    myReadFile >> val;
    cam.world_euler(0) = val * MY_PI / 180.;
    myReadFile >> c;
    myReadFile >> val;
    cam.world_euler(1) = val * MY_PI / 180.;
    myReadFile >> c;
    myReadFile >> val;
    cam.world_euler(2) = val * MY_PI / 180.;

    // close file
    myReadFile.close();
    return true;
}

std::ostream& operator<<(std::ostream& os, Camera const& cam)
{
    std::cout << "Projection Matrix:\n"
              << cam.projection.matrix() << '\n'
              << "CameraToWorld Matrix:\n"
              << cam.cam2world.matrix() << '\n'
              << "World Position:\n"
              << cam.world_pos.transpose() << '\n'
              << "World Euler:\n"
              << cam.world_euler.transpose();
    return os;
}

Eigen::Matrix3f create_rotation_matrix(float ax, float ay, float az) {
    Eigen::Matrix3f m;
    auto x = Eigen::AngleAxisf(ax, Eigen::Vector3f::UnitX());
    auto y = Eigen::AngleAxisf(ay, Eigen::Vector3f::UnitY());
    auto z = Eigen::AngleAxisf(az, Eigen::Vector3f::UnitZ());
    // rotate 180 along x-axis Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
    const Eigen::Matrix3f b = (Eigen::Matrix3f() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished();
    m = z * y * x * b;
    return m;
}

Eigen::Matrix3f create_rotation_matrix(Eigen::Vector3f const& euler)
{
    return create_rotation_matrix(euler.x(), euler.y(), euler.z());
}