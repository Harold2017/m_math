#include "m_mesh_split.h"

void PaintMesh(open3d::geometry::TriangleMesh& mesh,
	const Eigen::Vector3d& color) {
	mesh.vertex_colors_.resize(mesh.vertices_.size());
	for (size_t i = 0; i < mesh.vertices_.size(); i++) {
		mesh.vertex_colors_[i] = color;
	}
}

int main(int argc, char* argv[]) {
	assert(argc == 2);  // argv[1]: mesh_file
	auto mesh = open3d::io::CreateMeshFromFile(argv[1], false);

	/*
	// by open3d crop function
	auto origin_bounding_box = mesh->GetAxisAlignedBoundingBox();
	auto diff = origin_bounding_box.GetMaxBound() - origin_bounding_box.GetMinBound();
	auto new_low_bound = Eigen::Vector3d(origin_bounding_box.GetMinBound().x(), origin_bounding_box.GetMinBound().y(), origin_bounding_box.GetMinBound().z() + diff.z() / 2);
	auto crop_bounding_box = open3d::geometry::AxisAlignedBoundingBox(new_low_bound, origin_bounding_box.GetMaxBound());
	auto mesh_1 = mesh->Crop(crop_bounding_box);
	PaintMesh(*mesh, Eigen::Vector3d(0.0, 1.0, 0.0));
	PaintMesh(*mesh_1, Eigen::Vector3d(1.0, 0.0, 0.0));
	open3d::visualization::DrawGeometries({ mesh_1, mesh }, "mesh", 1920, 1080);
	open3d::visualization::DrawGeometries({ mesh_1 }, "cropped mesh", 1920, 1080);
	*/

	///*
	// by HM
	open3d::geometry::TriangleMesh l_mesh, r_mesh;
	auto diff = mesh->GetMaxBound() - mesh->GetMinBound();
	auto plane_center = cv::Point3d(diff.x() / 2, diff.y() / 2, diff.z() / 2);
	auto plane_normal = cv::Point3d(1, 1, 1);
	plane_normal = plane_normal / cv::norm(plane_normal);
	M_MATH::MeshCut(*mesh, plane_center, plane_normal, l_mesh, r_mesh);
	PaintMesh(l_mesh, Eigen::Vector3d(0.0, 1.0, 0.0));
	PaintMesh(r_mesh, Eigen::Vector3d(1.0, 0.0, 0.0));
	std::shared_ptr<open3d::geometry::TriangleMesh> p_l_mesh, p_r_mesh;
	p_l_mesh.reset(&l_mesh);
	p_r_mesh.reset(&r_mesh);
	printf("left mesh vertices size: %llu, triangles size: %llu\n", p_l_mesh->vertices_.size(), p_l_mesh->triangles_.size());
	printf("right mesh vertices size: %llu, triangles size: %llu\n", p_r_mesh->vertices_.size(), p_r_mesh->triangles_.size());
	open3d::visualization::DrawGeometries({ p_l_mesh, p_r_mesh }, "mesh", 1920, 1080);
	open3d::visualization::DrawGeometries({ p_l_mesh }, "left mesh", 1920, 1080);
	open3d::visualization::DrawGeometries({ p_r_mesh }, "right mesh", 1920, 1080);
	//*/

	return 0;
}