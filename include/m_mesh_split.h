//
// Created by Harold on 2021/2/18.
//

#ifndef M_MATH_M_MESH_SPLIT_H
#define M_MATH_M_MESH_SPLIT_H

#include <opencv2/core.hpp>
#include <open3d/Open3D.h>
#include <unordered_map>
#include <omp.h>

namespace M_MATH {
	struct TmpMesh {
		std::vector<Eigen::Vector3d> vertices;
		std::vector<Eigen::Vector3i> triangles;
		std::unordered_map<size_t, size_t> vertices_idx;  // global : local
	};

	// https://gdbooks.gitbooks.io/3dcollisions/content/Chapter2/static_aabb_plane.html
	bool BoundPlaneIntersect(open3d::geometry::OrientedBoundingBox const& bound,
		Eigen::Vector3d const& plane_center,
		Eigen::Vector3d const& plane_normal) {
		auto bound_center = bound.GetCenter();  // AABB center
		auto bound_extents = bound.GetMaxBound() - bound_center;  // positive extents
		// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
		auto r = bound_extents.x() * std::abs(plane_normal.x()) + bound_extents.y() * std::abs(plane_normal.y()) + bound_extents.z() * std::abs(plane_normal.z());
		// Compute distance of box center from plane
		auto s = (bound_center - plane_center).dot(plane_normal);
		// Intersection occurs when distance s falls within [-r,+r] interval
		return std::abs(s) <= r;
	}

	bool LinePlaneIntersect(Eigen::Vector3d const& p0,
		Eigen::Vector3d const& p1,
		Eigen::Vector3d const& plane_center,
		Eigen::Vector3d const& plane_normal,
		Eigen::Vector3d& intersect_point,
		double epsilon = 1e-6) {  //std::numeric_limits<double>::epsilon()
		auto line_dir = (p1 - p0).normalized();
		auto d = plane_normal.dot(line_dir);
		if (std::abs(d) < epsilon)
			return false;
		auto t = (plane_normal.dot(plane_center) - plane_normal.dot(p0)) / d;
		intersect_point = p0 + line_dir * t;
		return true;
	}

	/*
	 *          |  /| v1
	 *          | / |
	 *          |/  |
	 *       i0 |   |
	 *         /|   |
	 *        / |   |
	 *    v0 /__|___| v2
	 *          | i1
	 */
	bool TrianglePlaneIntersect(std::vector<Eigen::Vector3d> const& vertices,
		std::vector<Eigen::Vector3i> const& triangles,
		size_t triangle_idx,
		Eigen::Vector3d const& plane_center,
		Eigen::Vector3d const& plane_normal,
		TmpMesh& l_tmp_mesh,
		TmpMesh& r_tmp_mesh) {
		auto triangle = triangles[triangle_idx];

		auto is_triangle_vertices_in_mesh = [](Eigen::Vector3i triangle, TmpMesh const& mesh) {
			return std::array<bool, 3>{mesh.vertices_idx.find(triangle.x()) != mesh.vertices_idx.end(),
				mesh.vertices_idx.find(triangle.y()) != mesh.vertices_idx.end(),
				mesh.vertices_idx.find(triangle.z()) != mesh.vertices_idx.end()};
		};

		// triangle vertices all in one side mesh
		auto bls = is_triangle_vertices_in_mesh(triangle, l_tmp_mesh);
		if (bls[0] == bls[1] && bls[1] == bls[2]) {
			if (bls[0])
				l_tmp_mesh.triangles.push_back(Eigen::Vector3i(l_tmp_mesh.vertices_idx[triangle.x()], l_tmp_mesh.vertices_idx[triangle.y()], l_tmp_mesh.vertices_idx[triangle.z()]));
			else
				r_tmp_mesh.triangles.push_back(Eigen::Vector3i(r_tmp_mesh.vertices_idx[triangle.x()], r_tmp_mesh.vertices_idx[triangle.y()], r_tmp_mesh.vertices_idx[triangle.z()]));
			return false;
		}

		// find lonely vertex
		int v0 = 0;
		if (bls[0] != bls[1])
			v0 = bls[0] != bls[2] ? 0 : 1;
		else
			v0 = 2;

		int v1 = v0 + 1;
		if (v1 == 3) v1 = 0;
		int v2 = v0 - 1;
		if (v2 == -1) v2 = 2;

		// get insert points
		Eigen::Vector3d i0, i1;
		LinePlaneIntersect(vertices[triangle(v0)], vertices[triangle(v1)], plane_center, plane_normal, i0);
		LinePlaneIntersect(vertices[triangle(v0)], vertices[triangle(v2)], plane_center, plane_normal, i1);

		auto Nl = l_tmp_mesh.vertices.size();
		auto Nr = r_tmp_mesh.vertices.size();
		l_tmp_mesh.vertices.push_back(i0);
		l_tmp_mesh.vertices.push_back(i1);
		r_tmp_mesh.vertices.push_back(i0);
		r_tmp_mesh.vertices.push_back(i1);

		// create new triangles: [v0, i0, i1], [i0, v1, v2], [v2, i1, i0]
		bls[v0] ? l_tmp_mesh.triangles.push_back(Eigen::Vector3i(l_tmp_mesh.vertices_idx[triangle(v0)], Nl, Nl + 1)) : r_tmp_mesh.triangles.push_back(Eigen::Vector3i(r_tmp_mesh.vertices_idx[triangle(v0)], Nr, Nr + 1));
		if (bls[v1]) {
			l_tmp_mesh.triangles.push_back(Eigen::Vector3i(Nl, l_tmp_mesh.vertices_idx[triangle(v1)], l_tmp_mesh.vertices_idx[triangle(v2)]));
		}
		else {
			r_tmp_mesh.triangles.push_back(Eigen::Vector3i(Nr, r_tmp_mesh.vertices_idx[triangle(v1)], r_tmp_mesh.vertices_idx[triangle(v2)]));
		}
		if (bls[v2]) {
			l_tmp_mesh.triangles.push_back(Eigen::Vector3i(l_tmp_mesh.vertices_idx[triangle(v2)], Nl + 1, Nl));
		}
		else {
			r_tmp_mesh.triangles.push_back(Eigen::Vector3i(r_tmp_mesh.vertices_idx[triangle(v2)], Nr + 1, Nr));
		}

		return true;
	}

	bool MeshSplit(open3d::geometry::TriangleMesh const& mesh,
		cv::Point3d const& plane_center,
		cv::Point3d const& plane_normal,
		open3d::geometry::TriangleMesh& l_mesh,
		open3d::geometry::TriangleMesh& r_mesh) {
		auto pc = Eigen::Vector3d(plane_center.x, plane_center.y, plane_center.z);
		auto pn = Eigen::Vector3d(plane_normal.x, plane_normal.y, plane_normal.z);
		// 1. check whether bounding box intersect
		if (!BoundPlaneIntersect(mesh.GetOrientedBoundingBox(), pc, pn))
			return false;

		TmpMesh l_tmp_mesh, r_tmp_mesh;

		// 2. seperate vertices
		auto& vertices = mesh.vertices_;
		auto N = vertices.size();
		for (size_t i = 0; i < N; i++) {
			if ((vertices[i] - pc).dot(pn) >= 0) {
				l_tmp_mesh.vertices.push_back(vertices[i]);
				l_tmp_mesh.vertices_idx.insert({ i, l_tmp_mesh.vertices.size() - 1 });
			}
			else {
				r_tmp_mesh.vertices.push_back(vertices[i]);
				r_tmp_mesh.vertices_idx.insert({ i, r_tmp_mesh.vertices.size() - 1 });
			}
		}

		// if one of mesh's vertices is empty, no intersect
		if (l_tmp_mesh.vertices.empty() || r_tmp_mesh.vertices.empty())
			return false;

		// 3. seperate triangles and cut tirangles which intersected with plane
		auto& triangles = mesh.triangles_;
		auto M = triangles.size();
		for (auto i = 0; i < M; i++)
			TrianglePlaneIntersect(vertices, triangles, i, pc, pn, l_tmp_mesh, r_tmp_mesh);

		// 4. assign result to l_mesh, r_mesh
		l_mesh = open3d::geometry::TriangleMesh(l_tmp_mesh.vertices, l_tmp_mesh.triangles);
		r_mesh = open3d::geometry::TriangleMesh(r_tmp_mesh.vertices, r_tmp_mesh.triangles);

		/*
		// 5. remove duplicated
		l_mesh.RemoveDuplicatedVertices();
		l_mesh.RemoveDuplicatedTriangles();
		r_mesh.RemoveDuplicatedVertices();
		r_mesh.RemoveDuplicatedTriangles();
		*/

		return true;
	}
}

#endif M_MATH_M_MESH_SPLIT_H
