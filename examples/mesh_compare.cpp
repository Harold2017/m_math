#include "m_mesh_compare.h"
#include "m_mesh.h"

using namespace M_MATH;

void DrawRegistrationResult(std::shared_ptr<open3d::geometry::PointCloud> source, 
                            std::shared_ptr<open3d::geometry::PointCloud> target, 
                            Eigen::Matrix4d const& transformation) {
    if (!source->HasColors())
        source->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    if (!target->HasColors())
        target->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
    // make copy
    source = std::make_shared<open3d::geometry::PointCloud>(*source);
    source->Transform(transformation);
    open3d::visualization::DrawGeometries({ source, target }, "registration result");
}

void DrawRegistrationResult(std::shared_ptr<open3d::geometry::PointCloud> source, 
                            std::shared_ptr<open3d::geometry::PointCloud> target, 
                            Eigen::Matrix3d const& rot,
                            Eigen::Vector3d const& center,
                            Eigen::Vector3d const& trans) {
    if (!source->HasColors())
        source->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    if (!target->HasColors())
        target->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
    // make copy
    source = std::make_shared<open3d::geometry::PointCloud>(*source);
    source->Rotate(rot, center);
    source->Translate(trans);
    open3d::visualization::DrawGeometries({ source, target }, "registration result");
}

std::pair<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::pipelines::registration::Feature>> 
PreProcessPCD(std::shared_ptr<open3d::geometry::PointCloud> pcd, double voxel_size) {
    auto pcd_down = pcd->VoxelDownSample(voxel_size);
    auto radius_normal = voxel_size * 2;
    printf("Estimate normal with search radius: %f\n", radius_normal);
    pcd_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, 30));
    auto radius_feature = voxel_size * 5;
    printf("Compute FPFH feature with search radius: %f\n", radius_feature);
    auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        *pcd_down,
        open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
    return std::make_pair(pcd_down, pcd_fpfh);
}

std::shared_ptr<open3d::geometry::PointCloud> FilterPCDByZ(std::shared_ptr<open3d::geometry::PointCloud> const& pcd, double z) {
    std::vector<size_t> indices;
    indices.reserve(pcd->points_.size());
    for (auto i = 0; i < pcd->points_.size(); ++i)
        if (pcd->points_[i].z() > z)
            indices.push_back(i);
    return pcd->SelectByIndex(indices);
}

void DrawCorrespondence(std::shared_ptr<open3d::geometry::PointCloud> source, 
                        std::shared_ptr<open3d::geometry::PointCloud> target,
                        open3d::pipelines::registration::CorrespondenceSet const& correspondences) {
    std::vector<std::pair<int, int> > corres_lines; 
    for (const auto& c : correspondences) { corres_lines.emplace_back(std::make_pair(c[0], c[1])); } 
    auto lineset = open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(*source, *target, corres_lines); 
    lineset->PaintUniformColor({ 1, 0, 0 }); 
    //source->PaintUniformColor({ 0, 1, 0 });
    //target->PaintUniformColor({ 0, 1, 0 });
    //open3d::visualization::DrawGeometries({ source, target, lineset }, "correspondence line set");
    open3d::visualization::DrawGeometries({ lineset }, "correspondence line set");
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

// R_target2source * R_source2origin = R_target2origin   ===>   R_target2source = R_target2origin * R_source2origin.T()
Eigen::Matrix3d FindRotationMatrix(Eigen::Matrix3d const& rot_max_s2o, Eigen::Matrix3d const& rot_max_t2o) {
    return rot_max_t2o * rot_max_s2o.transpose();
}

// https://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix
Eigen::Matrix4d CreateQuaternion(Eigen::Matrix3d const& rot_mat, Eigen::Vector3d const& trans_vec) {
    Eigen::Matrix4d Quaternion;
    Quaternion.setIdentity();
    Quaternion.block<3, 3>(0, 0) = rot_mat;
    Quaternion.block<3, 1>(0, 3) = trans_vec;
    return Quaternion;
}

int main(int argc, char* argv[]) {
    assert(argc == 3);

    // PCDs
    auto p_source = open3d::io::CreatePointCloudFromFile(argv[1], "xyz");
    auto p_target = open3d::io::CreatePointCloudFromFile(argv[2], "xyz");
    // estimate normals
    double voxel_size = 0.01 * p_source->GetAxisAlignedBoundingBox().GetExtent().minCoeff();
    if (!p_source->HasNormals()) p_source->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size, 30));
    if (!p_target->HasNormals()) p_target->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size, 30));

    // draw original PCDs
    open3d::visualization::DrawGeometries({ p_source, p_target }, "original PCDs");

    // filter z > 5% max to remove ground plane noise
    p_source = FilterPCDByZ(p_source, 0.05 * p_source->GetAxisAlignedBoundingBox().GetExtent().z());
    p_target = FilterPCDByZ(p_target, 0.05 * p_target->GetAxisAlignedBoundingBox().GetExtent().z());

    // draw filtered PCDs
    open3d::visualization::DrawGeometries({ p_source, p_target }, "filtered PCDs");

    // get rough transformation matrix from bounding box
    auto p_source_bbox = std::make_shared<open3d::geometry::OrientedBoundingBox>(p_source->GetOrientedBoundingBox());
    auto p_target_bbox = std::make_shared<open3d::geometry::OrientedBoundingBox>(p_target->GetOrientedBoundingBox());
    open3d::visualization::DrawGeometries({ p_source_bbox, p_target_bbox }, "bbox");
    auto rot_mat = FindRotationMatrix(p_source_bbox->R_, p_target_bbox->R_);  // rot_mat * source_bbox.R_ = target_bbox.R_
    auto trans_vec = p_target_bbox->center_ - p_source_bbox->center_;
    //DrawRegistrationResult(p_source, p_target, rot_mat, p_source_bbox->center_, trans_vec);  // rotate relative to bbox's center
    auto g_transformation = CreateQuaternion(rot_mat, trans_vec);
    std::cout << "-Rough registration based on OrientedBoundingBox\ntransformation matrix:\n" << g_transformation << std::endl;
    DrawRegistrationResult(p_source, p_target, g_transformation);

    /*
    // preprocess
    auto source_pcd_fpfh_pair = PreProcessPCD(p_source, voxel_size);
    auto target_pcd_fpfh_pair = PreProcessPCD(p_target, voxel_size);

    // global registration RANSAC
    double distance_threshold = voxel_size * 1.5;

    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checkers;
    auto correspondence_checker_edge_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.95);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    auto correspondence_checker_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(0.2617993878);  // 15 degrees
    correspondence_checkers.push_back(correspondence_checker_edge_length);
    correspondence_checkers.push_back(correspondence_checker_distance);
    correspondence_checkers.push_back(correspondence_checker_normal);

    auto g_reg_result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
        *(source_pcd_fpfh_pair.first), *(target_pcd_fpfh_pair.first), *(source_pcd_fpfh_pair.second), *(target_pcd_fpfh_pair.second), distance_threshold,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
        4, correspondence_checkers, open3d::pipelines::registration::RANSACConvergenceCriteria(10000, 500));
    printf("-RANSAC\n\tinlier_rmse: %f, fitness: %f, correspondence_set size: %zu\n", g_reg_result.inlier_rmse_, g_reg_result.fitness_, g_reg_result.correspondence_set_.size());
    auto g_transformation = g_reg_result.transformation_;
    std::cout << "transformation matrix:\n" << g_transformation << std::endl;
    // draw result
    DrawRegistrationResult(p_source, p_target, g_transformation);
    // draw correspondence
    if (!g_reg_result.correspondence_set_.empty()) DrawCorrespondence(p_source, p_target, g_reg_result.correspondence_set_);
    */

   // FIXME: use transformation matrix from OrientedBoundingBox make the registration result worse...
   g_transformation = Eigen::Matrix4d::Identity();

    // fine registration ICP
    double threshold = voxel_size * 0.5;
    auto reg_result = open3d::pipelines::registration::RegistrationICP(*p_source, *p_target, threshold, g_transformation, 
        open3d::pipelines::registration::TransformationEstimationPointToPoint(false), open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 2000));
    printf("-ICP point to point\n\tinlier_rmse: %f, fitness: %f, correspondence_set size: %zu\n", reg_result.inlier_rmse_, reg_result.fitness_, reg_result.correspondence_set_.size());
    auto transformation = reg_result.transformation_;
    std::cout << "transformation matrix:\n" << transformation << std::endl;
    // draw result
    DrawRegistrationResult(p_source, p_target, transformation);
    // draw correspondence
    if (!reg_result.correspondence_set_.empty()) DrawCorrespondence(p_source, p_target, reg_result.correspondence_set_);

    // fine registration point-to-plane ICP
    auto reg_result_p2l = open3d::pipelines::registration::RegistrationICP(*p_source, *p_target, threshold, g_transformation, 
        open3d::pipelines::registration::TransformationEstimationPointToPlane(), open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 2000));
    printf("-ICP point to plane\n\tinlier_rmse: %f, fitness: %f, correspondence_set size: %zu\n", reg_result_p2l.inlier_rmse_, reg_result_p2l.fitness_, reg_result_p2l.correspondence_set_.size());
    auto transformation_p2l = reg_result_p2l.transformation_;
    std::cout << "transformation matrix:\n" << transformation_p2l << std::endl;
    // draw result
    DrawRegistrationResult(p_source, p_target, transformation_p2l);
    // draw correspondence
    if (!reg_result_p2l.correspondence_set_.empty()) DrawCorrespondence(p_source, p_target, reg_result_p2l.correspondence_set_);

    // generate mesh
    auto base_mesh = TriangleMesh::GenMesh(p_source, TriangleMesh::BallPivot);
    auto compared_mesh = TriangleMesh::GenMesh(p_target, TriangleMesh::BallPivot);

    auto mcr = MeshCompare(base_mesh, compared_mesh);
    auto res_mesh = open3d::geometry::TriangleMesh(mcr.vertices, mcr.triangles);

    // show mesh compare result
    auto max_distance = *std::max_element(mcr.vertex_distances.begin(), mcr.vertex_distances.begin());
    printf("max_distance: %f\n", max_distance);
    fflush(stdout);

    open3d::visualization::ColorMapHot colormap;
    res_mesh.vertex_colors_.resize(res_mesh.vertices_.size());
    for (auto i = 0; i < res_mesh.vertices_.size(); i++) {
        res_mesh.vertex_colors_[i] = colormap.GetColor(mcr.vertex_distances[i] / max_distance);
    }
    auto p_res_mesh = std::make_shared<open3d::geometry::TriangleMesh>(std::move(res_mesh));
    open3d::visualization::DrawGeometries({ p_res_mesh }, "result mesh");


    return 0;
}