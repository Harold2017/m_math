//
// Created by Harold on 2021/11/15.
//

#include "m_texture_mapping.hpp"
#include "../utils/stopwatch.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <fstream>

using namespace M_MATH;
using namespace texture_mapping;

std::ifstream& go_to_line(std::ifstream& file, unsigned int num) {
  file.seekg(std::ios::beg);
  for (int i = 0; i < num - 1; ++i) {
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return (file);
}

bool read_cam_pose_file(std::string filename,
                        Camera& cam) {
  std::ifstream myReadFile;
  myReadFile.open(filename.c_str(), std::ios::in);
  if (!myReadFile.is_open()) {
    printf("Error opening file %s\n", filename.c_str());
    return false;
  }
  myReadFile.seekg(std::ios::beg);

  double val;

  go_to_line(myReadFile, 1);
  myReadFile >> val;
  std::cout  << ("t ");
  std::cout  << val;
  cam.pose(0, 3) = val;  // TX
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  cam.pose(1, 3) = val;  // TY
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.pose(2, 3) = val;  // TZ

  go_to_line(myReadFile, 2);
  myReadFile >> val;
  std::cout  << ("r1 ");
  std::cout  << val;
  cam.pose(0, 0) = val;
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  cam.pose(0, 1) = val;
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.pose(0, 2) = val;

  go_to_line(myReadFile, 3);
  myReadFile >> val;
  std::cout  << ("r2 ");
  std::cout  << val;
  cam.pose(1, 0) = val;
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  cam.pose(1, 1) = val;
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.pose(1, 2) = val;

  go_to_line(myReadFile, 4);
  myReadFile >> val;
  std::cout  << ("r3");
  std::cout  << val;
  cam.pose(2, 0) = val;
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  cam.pose(2, 1) = val;
  myReadFile >> val;
  std::cout  << (" ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.pose(2, 2) = val;

  cam.pose(3, 0) = 0.0;
  cam.pose(3, 1) = 0.0;
  cam.pose(3, 2) = 0.0;
  cam.pose(3, 3) = 1.0;  // Scale

  go_to_line(myReadFile, 5);
  myReadFile >> val;
  std::cout  << ("fx ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.focal_length_w = val;
  go_to_line(myReadFile, 6);
  myReadFile >> val;
  std::cout  << ("fy ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.focal_length_h = val;
  go_to_line(myReadFile, 7);
  myReadFile >> val;
  std::cout  << ("cx ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.center_w = val;
  go_to_line(myReadFile, 8);
  myReadFile >> val;
  std::cout  << ("cy ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.center_h = val;
  go_to_line(myReadFile, 9);
  myReadFile >> val;
  std::cout  << ("h ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.height = val;
  go_to_line(myReadFile, 10);
  myReadFile >> val;
  std::cout  << ("w ");
  std::cout  << val;
  std::cout  << ("\n");
  cam.width = val;

  // close file
  myReadFile.close();
  return true;
}

std::shared_ptr<open3d::geometry::Image> img_cv2o3d(std::string const& path)
{
    auto cv_img = cv::imread(path, cv::IMREAD_COLOR);
    auto o3d_img = std::make_shared<open3d::geometry::Image>();
    int bytes_per_channel = (cv_img.depth() / 2 + 1);
    o3d_img->Prepare(cv_img.cols, cv_img.rows, cv_img.channels(), bytes_per_channel);
    std::memcpy(o3d_img->data_.data(), cv_img.data, cv_img.total() * cv_img.channels() * bytes_per_channel);
    return o3d_img;
}

// .\m_texture_mapping.exe un_textured.obj camera.txt camera_img.png occluded.jpg
int main(int argc, char* argv[])
{
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
	{
		TIME_BLOCK("- load mesh: ");
		mesh = open3d::io::CreateMeshFromFile(argv[1], false);
	}
    open3d::visualization::DrawGeometries({ mesh }, "original mesh", 1920, 1080);

    CameraVector cameras;
    Camera camera;
    read_cam_pose_file(argv[2], camera);
    camera.texture_file = argv[3];
    cameras.push_back(camera);

    // add material
    auto texture_img = img_cv2o3d(camera.texture_file);
    mesh->textures_ = std::vector<open3d::geometry::Image>({*texture_img});
    if (argc >= 5)
    {
        auto occluded_img = img_cv2o3d(argv[4]);
        mesh->textures_.push_back(*occluded_img);
    }

    {
        TIME_BLOCK("- texture mapping: ");
        TextureMeshWithMultipleCameras(mesh, cameras);
    }

    open3d::visualization::DrawGeometries({ mesh }, "textured mesh", 1920, 1080);

    return 0;
}
