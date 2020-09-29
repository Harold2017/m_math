//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_M_GEO_H
#define M_MATH_M_GEO_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_2.h>
#include "m_curvefit.h"

namespace M_MATH {
    namespace GEO {
        typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
        typedef K::Point_2 Point_2;
        typedef CGAL::Polygon_2<K> Polygon_2;

        // area of a group of points
        template<typename T, typename IT>
        T Area(IT const begin, IT const end) {
            std::vector<Point_2> pts, vertices;
            pts.reserve(std::distance(begin, end));
            vertices.reserve(std::distance(begin, end));
            for (IT it = begin; it != end; ++it)
                pts.emplace_back((*it).x, (*it).y);
            CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(vertices));
            vertices.shrink_to_fit();
            Polygon_2 pgn(vertices.begin(), vertices.end());
            return pgn.area();
        }

        // histogram of points height
        // need sort points first
        // hist size is N - 1, not N: N points with N - 1 bins
        template<typename IT, typename T, size_t N = 20>
        std::vector<unsigned> HistOfHeight(IT const begin, IT const end, T max, T min) {
            static_assert(N > 1, "Hist: at least 1 sets: [min, max]");
            double deltaZ = (max - min) / (N - 1);
            std::vector<unsigned> hist(N - 1, 0);
            size_t cnt = 1;
            for (auto it = begin; it != end; ++it)
                // [min, min + deltaZ) -> hist[0]
                if ((*it).z < min + deltaZ)
                    ++hist[0];
                    // [max - deltaZ, max] -> hist[N-1]
                else if ((*it).z >= max - deltaZ) {
                    ++hist[N-2];
                }
                    // [min + deltaZ : deltaZ : max - deltaZ)
                else {
                    if (((*it).z >= min + cnt * deltaZ) && ((*it).z < min + double(cnt + 1) * deltaZ))
                        ++hist[cnt];
                    else {
                        ++cnt;
                        ++hist[cnt];
                    }
                }
            return hist;
        }

        // height ratio (distribution), certain point height in whole points set
        template<typename IT, typename T, size_t N = 20>
        double HeightRatio(IT const begin, IT const end, T max, T min, double z) {
            // hist size is N - 1, not N: N points with N - 1 bins
            static auto hist = HistOfHeight<N>(begin, end, max, min);
            double deltaZ = (max - min) / (N - 1);
            if (z == min)
                return 1.0;
            else if (z == max)
                return 0.0;
            else {
                double num;
                unsigned sum;
                num = (z - min) / deltaZ;
                auto quotient = int(num);
                auto remainder = num - quotient;
                if (remainder < 0.5)
                    sum = std::accumulate(hist.begin(), hist.begin() + quotient + 1, 0u);
                else
                    sum = std::accumulate(hist.begin(), hist.begin() + quotient + 2, 0u);
                return 1.0 - double(sum) / double(std::distance(begin, end));
            }
        }

        // Height distribution curve, 0%: highest point, 100%: lowest point
        template<typename IT, typename T, size_t N = 20>
        M_MATH::CurveFit HeightDistCurve(IT const begin, IT const end, T max, T min) {
            // 0% - 100%
            auto Y = linspace(max, min, N);
            std::vector<double> Z(N);
            for (auto i = 0; i < N; i++)
                Z[i] = HeightRatio(begin, end, max, min, Y[i]);
            M_MATH::CurveFit cf;
            cf.polyfit(Z.data(), Y.data(), N, 5);
            return cf;
        }
    }
}

#endif //M_MATH_M_GEO_H
