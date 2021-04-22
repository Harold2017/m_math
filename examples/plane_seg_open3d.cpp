//
// Created by Harold on 2021/4/19.
//

#include "m_plane_seg_open3d.h"

int main(int argc, char* argv[]) {
    auto ppcd = open3d::io::CreatePointCloudFromFile(argv[1]);
    // remove nan/inf
    ppcd->RemoveNonFinitePoints(true, true);
    // down sample
    ppcd = ppcd->VoxelDownSample(0.003);
    // remove point clouds noise using statitical noise removal method
    ppcd->RemoveStatisticalOutliers(20, 2.0);
    std::cout << ppcd->points_.size() << '\n' << std::endl;
    /*
    auto plane = M_MATH::PlaneSeg(*ppcd, 0.003, 100, 100);
    std::cout << plane.plane_center << '\n'
              << plane.plane_normal << '\n'
              << plane.plane_pts.size() << '\n'
              << std::endl;

    ppcd->PaintUniformColor({ 1, 0, 0 });

    auto pplane = std::make_shared<open3d::geometry::PointCloud>(plane.plane_pts);
    pplane->PaintUniformColor({ 0, 1, 0 });

    open3d::visualization::DrawGeometries({ pplane, ppcd });
    */

    auto planes = M_MATH::MultiplePlaneSeg(*ppcd, 0.01, 0.003, 3, 100);
    auto N = planes.size();
    std::cout << "planes number: " << N << std::endl;
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> pgeos;
    open3d::visualization::ColorMapJet cm;
    for (auto i = 0; i < N; ++i) {
        auto ppl = std::make_shared<open3d::geometry::PointCloud>(planes[i].plane_pts);
        ppl->PaintUniformColor(cm.GetColor(double(i) / double(N)));
        pgeos.push_back(ppl);
    }
    open3d::visualization::DrawGeometries(pgeos);

    return 0;
}