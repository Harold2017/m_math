//
// Created by Harold on 2020/10/31.
//

#include <iostream>
#include "m_mesh_volume_surface_area.h"

using namespace M_MATH;

int main() {
    // cubic
    std::vector<cv::Point3f> vertices = {{0, 0, 0},
                                         {1, 0, 0},
                                         {0, 1, 0},
                                         {1, 1, 0},
                                         {0, 0, 1},
                                         {1, 0, 1},
                                         {0, 1, 1},
                                         {1, 1, 1}};

    std::vector<cv::Point3i> triangles = {{0, 2, 1},
                                          {1, 2, 3},
                                          {4, 6, 5},
                                          {5, 6, 7},
                                          {0, 1, 4},
                                          {1, 5, 4},
                                          {2, 3, 6},
                                          {3, 7, 6},
                                          {2, 0, 4},
                                          {2, 4, 6},
                                          {3, 1, 5},
                                          {3, 5, 7}};

    std::cout << MeshVolume(vertices, triangles) << std::endl;
    std::cout << MeshSurface(vertices, triangles) << std::endl;

    return 0;
}
