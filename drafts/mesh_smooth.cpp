//
// Created by Harold on 2021/11/29.
//

#include <open3d/Open3D.h>
#include "../utils/stopwatch.h"

int main(int argc, char* argv[])
{
    auto mesh = open3d::io::CreateMeshFromFile(argv[1], false);
    mesh->RemoveDuplicatedVertices();
    mesh->RemoveDuplicatedTriangles();

    // open3d::visualization::DrawGeometries({ mesh }, "original mesh", 640, 480, 50, 50, false, true);

    // subdivide mesh
    // auto subdivided = mesh->SubdivideLoop(2);
    // open3d::visualization::DrawGeometries({ mesh }, "subdivided mesh", 640, 480, 50, 50, false, true);

    // smooth mesh
    // auto averaged = mesh->FilterSmoothSimple(5);
    // averaged->ComputeVertexNormals();
    // open3d::visualization::DrawGeometries({ averaged }, "averaged mesh", 640, 480, 50, 50, false, true);

    // auto laplacian = mesh->FilterSmoothLaplacian(5, 0.1);
    // laplacian->ComputeVertexNormals();
    // open3d::visualization::DrawGeometries({ laplacian }, "laplacian-filtered mesh", 640, 480, 50, 50, false, true);

    std::shared_ptr<open3d::geometry::TriangleMesh> taubin = nullptr;
    {
        TIME_BLOCK("FilterSmoothTaubin:");
        taubin = mesh->FilterSmoothTaubin(50);
        taubin->ComputeVertexNormals();
    }
    // open3d::visualization::DrawGeometries({ taubin }, "taubin-filtered mesh", 640, 480, 50, 50, false, true);

    // subdivide mesh
    std::shared_ptr<open3d::geometry::TriangleMesh> subdivided = nullptr;
    {
        TIME_BLOCK("SubdivideLoop:");
        subdivided = taubin->SubdivideLoop(2);
    }
    // open3d::visualization::DrawGeometries({ subdivided }, "subdivided taubin-filtered mesh", 640, 480, 50, 50, false, true);

    open3d::io::WriteTriangleMesh(argv[2], *subdivided);

    return 0;
}