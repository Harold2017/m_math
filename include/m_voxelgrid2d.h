//
// Created by Harold on 2021/6/4.
//

#ifndef M_MATH_M_VOXELGRID2D_HPP
#define M_MATH_M_VOXELGRID2D_HPP

#include <opencv2/core.hpp>

namespace M_MATH {
	template<typename T>
	struct Voxel {
		static_assert(std::is_floating_point<T>::value, "T should be floating point");

		bool m_is_occupied;
		T m_intensity;
		std::vector<size_t> m_pt_ids;
	};

	template<typename T>
	class VoxelGrid {
		static_assert(std::is_floating_point<T>::value, "T should be floating point");

	public:
		VoxelGrid(size_t rows, size_t cols, T vx, T vy);
		~VoxelGrid() = default;

		cv::Mat CreateImg(std::vector<cv::Point_<T>> const& pts, bool is_binary, cv::Point_<T> const& origin);

	private:
		std::vector<std::vector<Voxel<T>>> m_voxels;
		size_t m_rows, m_cols;
		T m_vx, m_vy;
		cv::Point_<T> m_origin;
	};


	template<typename T>
	VoxelGrid<T>::VoxelGrid(size_t rows, size_t cols, T vx, T vy) : m_rows(rows), m_cols(cols), m_vx(vx), m_vy(vy), m_origin(T{}, T{}) {
		assert(rows > 1 && cols > 1 && vx > 0 && vy > 0);
		m_voxels = std::vector<std::vector<Voxel<T>>>(m_rows, std::vector<Voxel<T>>(m_cols));
	}

	template<typename T>
	cv::Mat VoxelGrid<T>::CreateImg(std::vector<cv::Point_<T>> const& pts, bool is_binary, cv::Point_<T> const& origin) {
		auto ivx = T(1.0) / m_vx;
		auto ivy = T(1.0) / m_vy;
		m_origin = origin;
		for (auto i = 0; i < pts.size(); i++) {
			auto ix = static_cast<size_t>(std::floor((pts[i].x - origin.x) * ivx));  // col
			auto iy = static_cast<size_t>(std::floor((pts[i].y - origin.y) * ivy));  // row
			m_voxels[iy][ix].m_is_occupied = true;
			m_voxels[iy][ix].m_pt_ids.push_back(i);
		}

		cv::Mat img = cv::Mat::zeros(m_rows, m_cols, CV_8UC1);
		if (is_binary) {
			for (auto i = 0; i < m_rows; i++)
				for (auto j = 0; j < m_cols; j++)
					if (m_voxels[i][j].m_is_occupied)
						img.at<uchar>(i, j) = 255;
		}
		else {  // compute and use intensity as pixel value
			size_t max_pts_in_voxel = 0;
			for (auto i = 0; i < m_rows; i++)
				for (auto j = 0; j < m_cols; j++)
					max_pts_in_voxel = max_pts_in_voxel < m_voxels[i][j].m_pt_ids.size() ? m_voxels[i][j].m_pt_ids.size() : max_pts_in_voxel;
			for (auto i = 0; i < m_rows; i++)
				for (auto j = 0; j < m_cols; j++) {
					auto intensity = T(255 * m_voxels[i][j].m_pt_ids.size()) / T(max_pts_in_voxel);
					m_voxels[i][j].m_intensity = intensity;
					img.at<uchar>(i, j) = static_cast<uchar>(intensity);
				}
		}
		return img;
	}
}

#endif //M_MATH_M_VOXELGRID2D_HPP