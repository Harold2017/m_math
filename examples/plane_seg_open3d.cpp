//
// Created by Harold on 2021/4/19.
//

#include "m_plane_seg_open3d.h"

int main(int argc, char* argv[]) {
    auto ppcd = open3d::io::CreatePointCloudFromFile(argv[1]);
    std::cout << ppcd->points_.size() << '\n' << std::endl;
    auto plane = M_MATH::PlaneSeg(*ppcd, 0.003, 100, 100);
    std::cout << plane.plane_center << '\n'
              << plane.plane_normal << '\n'
              << plane.plane_pts.size() << '\n'
              << std::endl;

    ppcd->PaintUniformColor({ 1, 0, 0 });

    auto pplane = std::make_shared<open3d::geometry::PointCloud>(plane.plane_pts);
    pplane->PaintUniformColor({ 0, 1, 0 });

    open3d::visualization::DrawGeometries({ pplane, ppcd });

    return 0;
}