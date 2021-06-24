//
// Created by Harold on 2021/06/24.
//

#ifndef M_MATH_M_MESH_VERTICES_FILTER_H
#define M_MATH_M_MESH_VERTICES_FILTER_H

#include <open3d/Open3D.h>

namespace M_MATH {
    namespace details
	{
		/**
		 * @brief check whether point is inside the polygon
		 * 
		 * @param pt 
		 * @param polygon_pts 
		 * @return true 
		 * @return false 
		 */
		bool IsPtInsidePolygon(Eigen::Vector2d const &pt, std::vector<Eigen::Vector2d> const &polygon_pts)
		{
			bool inside = false;
			auto ptx = pt.x(), pty = pt.y();
			Eigen::Vector2d verti, vertj;
			for (int i = 0, j = polygon_pts.size() - 1; i < polygon_pts.size(); j = i++)
			{
				verti = polygon_pts[i], vertj = polygon_pts[j];
				if ((verti.y() > pty) != (vertj.y() > pty) &&
					ptx < (vertj.x() - verti.x()) * (pty - verti.y()) / (vertj.y() - verti.y()) + verti.x())
					inside = !inside;
			}
			return inside;
		}

		bool IsPtInsideCircle(Eigen::Vector2d const &pt, Eigen::Vector2d const &center, double radius_sqr)
		{
			return (pt - center).squaredNorm() < radius_sqr;
		}

		/**
		 * @brief FilterVerticesByTopView
		 * 
		 * @param mesh 
		 * @param predict 
		 * @return std::vector<size_t> 
		 */
		std::vector<size_t> FilterVerticesByTopView(open3d::geometry::TriangleMesh const &mesh, std::function<bool(Eigen::Vector2d const &pt)> predict)
		{
			std::vector<size_t> indices;
			auto const &points = mesh.vertices_;
			indices.reserve(points.size());
			Eigen::Vector2d point{0, 0};
			for (size_t idx = 0; idx < points.size(); idx++)
			{
				point(0) = points[idx].x();
				point(1) = points[idx].y();
				if (predict(point))
					indices.push_back(idx);
			}
			return indices;
		}
	}

	std::vector<size_t> VerticesInsidePolygonROI(open3d::geometry::TriangleMesh const &mesh, std::vector<Eigen::Vector2d> const &polygon_pts)
	{
		return details::FilterVerticesByTopView(mesh, [&](Eigen::Vector2d const &pt)
												{ return details::IsPtInsidePolygon(pt, polygon_pts); });
	}

	std::vector<size_t> VerticesInsideCircleROI(open3d::geometry::TriangleMesh const &mesh, Eigen::Vector2d const &center, double radius)
	{
		return details::FilterVerticesByTopView(mesh, [=](Eigen::Vector2d const &pt)
												{ return details::IsPtInsideCircle(pt, center, radius * radius); });
	}

	/**
	 * @brief Get the sub-mesh from vertices indices
	 * 
	 * @param mesh 
	 * @param indices 
	 * @return std::shared_ptr<open3d::geometry::TriangleMesh> 
	 */
	std::shared_ptr<open3d::geometry::TriangleMesh> GetSubMeshFromVertices(open3d::geometry::TriangleMesh const &mesh, std::vector<size_t> const &indices)
	{
		return mesh.SelectByIndex(indices);
	}
}

#endif //M_MATH_M_MESH_VERTICES_FILTER_H