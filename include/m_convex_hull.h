//
// Created by Harold on 2021/6/3.
//

#ifndef M_MATH_M_CONVEX_HULL_H
#define M_MATH_M_CONVEX_HULL_H

#include <vector>
#include <Eigen/Dense>
#include <algorithm>

#include "m_pt_in_polygon.h"

namespace M_MATH {
	namespace details {
		// https://www.geeksforgeeks.org/convex-hull-set-2-graham-scan/
		// https://www.cs.cmu.edu/~quake/robust.html
		double orient2d(Eigen::Vector2d const& p1, Eigen::Vector2d const& p2, Eigen::Vector2d const& p3) {
			return (p2(1)- p1(1)) * (p3(0) - p2(0)) - (p2(0) - p1(0)) * (p3(1) - p2(1));
		}
	}

	// Graham Scan
	// http://cgm.cs.mcgill.ca/~athens/cs601/
	std::vector<Eigen::Vector2d> ConvexHull(std::vector<Eigen::Vector2d> const& points) {
		// find four extreme points
		auto left = points[0];
		auto top = points[0];
		auto right = points[0];
		auto bottom = points[0];
		for (auto i = 0; i < points.size(); i++) {
			auto p = points[i];
			if (p(0) < left(0)) left = p;
			if (p(0) > right(0)) right = p;
			if (p(1) < top(1)) top = p;
			if (p(1) > bottom(1)) bottom = p;
		}
		// filter out points that are inside the resulting quadrilateral
		auto cull = std::vector<Eigen::Vector2d>{ left, top, right, bottom };
		auto filtered = cull;
		for (auto i = 0; i < points.size(); i++) {
			if (!IsPtInsidePolygon(points[i], cull)) filtered.push_back(points[i]);
		}

		// convex hull
		std::sort(filtered.begin(), filtered.end(), [](Eigen::Vector2d const& p1, Eigen::Vector2d const& p2) { return p1(0) == p2(0) ? p1(1) < p2(1) : p1(0) < p2(0); });  // compare by x
		auto N = filtered.size();
		std::vector<Eigen::Vector2d> lower, upper;
		lower.reserve(N); upper.reserve(N);

		for (int i = 0; i < N; i++) {
			while (lower.size() >= 2 && details::orient2d(lower[lower.size() - 2], lower[lower.size() - 1], filtered[i]) <= 0)
				lower.pop_back();
			lower.push_back(filtered[i]);
		}

		for (int i = N - 1; i >= 0; i--) {
			while (upper.size() >= 2 && details::orient2d(upper[upper.size() - 2], upper[upper.size() - 1], filtered[i]) <= 0)
				upper.pop_back();
			upper.push_back(filtered[i]);
		}

		upper.pop_back();
		lower.pop_back();

		lower.insert(lower.end(), upper.begin(), upper.end());
		lower.shrink_to_fit();

		return lower;
	}
}

#endif //M_MATH_M_CONVEX_HULL_H