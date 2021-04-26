//
// Created by Harold on 2021/4/26.
//

#ifndef M_MATH_M_MR_H
#define M_MATH_M_MR_H

#include <numeric>
#include <cmath>
#include <vector>
#include <ostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "m_integration_hm.h"
#include "m_hist_hm.h"
#include "utils.h"
#include "m_interp_hm.h"

// debug
//#define DEBUG_DRAW

#ifdef DEBUG_DRAW
#include <opencv2/highgui.hpp>

#define BOARD_SIZE 800
#endif // DEBUG_DRAW

namespace M_MATH {
    /**
     * \class Mr
     *
     * \brief material ratio related helper functions
     *
     * Height vs. Mr(c)\n
     * highest point(0%), lowest point(100%)\n
     * input cv::Point3_<T>\n
     * \note to get S/V parameters, need ensure material curve is "S" shape
     */
    template<typename T>
    class Mr {
        static_assert(std::is_floating_point<T>::value, "T should be floating point");
    public:
        Mr() = default;

        /// n = height/deltaZ, should > 1
        template<typename IT, typename GetterFp>
        void Init(IT begin, IT end, GetterFp gfp, size_t n = 20);

    public:
        T Sk, Smr1, Smr2, Spk, Svk, A1, A2;

        private:
        T min, max;
        size_t N;
        size_t pts_size;
        std::vector<size_t> hist;
        std::vector<cv::Point_<T>> material_pts;
        M_MATH::Integration<T> i_QAG;

    public:
        /// percentage to z
        double MaterialCurve(double p);

        /// according to above smc curve to obtain other parameters
        /// Sk, Smr1, Smr2, Spk, Svk should be calculated only if the area material ratio curve is "S" shaped
        template<size_t NX = 100>
        void CalculateParams() { Sk_Smr1_Smr2_Spk_Svk_A1_A2<NX>(); }

        /// for final Vv(p) should multiply const K, which is used by converting result to ml/mm2
        double Vv(double p);

        /// for final Vm(p) should multiply const K, which is used by converting result to ml/mm2
        double Vm(double p);

        friend std::ostream& operator<<(std::ostream& os, Mr const& mr) {
            return os << "Sk: " << mr.Sk << '\n'
                      << "Smr1: " << mr.Smr1 << '\n'
                      << "Smr2: " << mr.Smr2 << '\n'
                      << "Spk: "  << mr.Spk << '\n'
                      << "Svk: "  << mr.Svk << '\n'
                      << "A1: "  << mr.A1 << '\n'
                      << "A2: "  << mr.A2 << '\n';
        }

    private:
        double SMR(double z);

        /// Sk, Smr1, Smr2, Spk, Svk should be calculated only if the area material ratio curve is "S" shaped
        template<size_t NX = 100>
        void Sk_Smr1_Smr2_Spk_Svk_A1_A2();
    };

    template<typename T>
    template<typename IT, typename GetterFp>
    void Mr<T>::Init(const IT begin, const IT end, GetterFp gfp, size_t n) {
        pts_size = std::distance(begin, end);
        N = (n < 4096) ? n : 4096;
        min = std::numeric_limits<T>::max();
        max = std::numeric_limits<T>::lowest();
        T z;
        for (IT it = begin; it != end; ++it) {
            z = gfp(*it);
            if (z < min) min = z;
            if (max < z) max = z;
        }

        // N = min(height/deltaZ, 4096), VK (p129) uses detalZ as input
        hist = M_MATH::Histogram<T>(begin, end, gfp, min, max, N);

        // Sxp = Smc(2.5%) - Smc(50%)
        // Sxp = MaterialCurve(0.025) - MaterialCurve(0.5);
    }

    template<typename T>
    double Mr<T>::SMR(double z) {
        double deltaZ = (max - min) / N;
        if (z == min)
            return 1.0;
        else if (z == max)
            return 0.0;
        else {
            double num;
            size_t sum;
            num = (z - min) / deltaZ;
            auto quotient = int(num);
            auto remainder = num - quotient;
            if (remainder < 0.5)
                sum = std::accumulate(hist.begin(), hist.begin() + quotient + 1, size_t(0));
            else
                sum = std::accumulate(hist.begin(), hist.begin() + quotient + 2, size_t(0));
            return 1.0 - double(sum) / double(pts_size);
        }
    }

    // based on interpolation
    template<typename T>
    double Mr<T>::MaterialCurve(double p) {
        if (material_pts.empty()) {
            // 0% - 100%
            auto Y = linspace(max, min, N);
            for (auto i = 0; i < N; i++)
                material_pts.emplace_back(SMR(Y[i]), Y[i]);
        }
        if (p == 0.0)
            return max;
        else if (p == 1.0)
            return min;
        else
            return M_MATH::Interpolation::Interpolate(material_pts.begin(), material_pts.end(), p);
    }

    template<typename T>
    template<size_t NX>
    void Mr<T>::Sk_Smr1_Smr2_Spk_Svk_A1_A2() {
        // prepare smc curve
        MaterialCurve(0.0);
        auto X = linspace(0.0, 1.0, NX);
        std::vector<cv::Point_<T>> vp;
        vp.reserve(X.size());
        for (auto e : X)
            if (e == 0.0)
                vp.emplace_back(e, max);
            else if (e == 1.0)
                vp.emplace_back(e, min);
            else
                vp.emplace_back(e, MaterialCurve(e));

#ifdef DEBUG_DRAW
        cv::Mat board = cv::Mat(BOARD_SIZE, BOARD_SIZE, CV_8UC3);
        std::vector<cv::Point> dpts;
        dpts.reserve(vp.size());
        for (auto const& p : vp)
            dpts.emplace_back(p.x * BOARD_SIZE, BOARD_SIZE - p.y);  // image (0, 0) is at left-up
        cv::polylines(board, dpts, false, cv::Scalar(0, 255, 0), 1, 8, 0);
#endif //DEBUG_DRAW

        // prepare smr curve
        auto deltaX = 4 * vp.size() / 10;
        std::vector<cv::Point_<T>> tmp_pts;
        tmp_pts.reserve(deltaX);
        
        cv::Vec<T, 4> line;
        T k{}, b{};

        auto it = vp.begin();
        auto min_slope = std::numeric_limits<T>::max();
        for (auto i = vp.begin(); i != vp.end() - deltaX; ++i) {
            tmp_pts.clear();
            for (auto j = 0; j < deltaX; ++j)
                tmp_pts.push_back(*(i + j));
            cv::fitLine(tmp_pts, line, cv::DIST_L2, 0, 0.01, 0.01);
            k = line[1] / line[0];
            if (abs(k) < abs(min_slope)) {
                min_slope = k;
                it = i;
                b = line[3] - (line[1] / line[0]) * line[2];
            }
        }

        tmp_pts.clear();
        for (auto j = 0; j < deltaX; ++j)
            tmp_pts.push_back(*(it + j));
        k = min_slope;

#ifdef DEBUG_DRAW
        cv::line(board, cv::Point(0, BOARD_SIZE - b), cv::Point(BOARD_SIZE, BOARD_SIZE - k - b), cv::Scalar(255, 0, 0));
#endif // DEBUG_DRAW

        // Sk (y at 0 - y at 1 = b - (k + b) = -k)
        Sk = -k;

        // Spk
        auto Smr1_y = b;
        tmp_pts.clear();
        for (auto _it = vp.begin(); _it != vp.end(); ++_it)
            if ((*_it).y - Smr1_y >= 0)
                tmp_pts.push_back(*_it);
            else break;
        cv::fitLine(tmp_pts, line, cv::DIST_L2, 0, 0.01, 0.01);
        auto Spk_k = line[1] / line[0];
        auto Spk_b = line[3] - Spk_k * line[2];
        Spk = Spk_b - b;
        Smr1 = tmp_pts[tmp_pts.size()-1].x;

#ifdef DEBUG_DRAW
        cv::line(board, cv::Point(BOARD_SIZE * tmp_pts[0].x, BOARD_SIZE - Spk_k * tmp_pts[0].x - Spk_b), cv::Point(BOARD_SIZE * tmp_pts[tmp_pts.size() - 1].x, BOARD_SIZE - Spk_k * tmp_pts[tmp_pts.size() - 1].x - Spk_b), cv::Scalar(255, 0, 0));
#endif //DEBUG_DRAW

        // Svk
        auto Smr2_y = k + b;
        auto Smr2_it_x = std::find_if(vp.begin(), vp.end(), [=](cv::Point_<T> const& p) { return p.y <= Smr2_y; });
        tmp_pts.clear();
        for (auto i = Smr2_it_x; i != vp.end(); ++i)
            tmp_pts.push_back(*i);
        cv::fitLine(tmp_pts, line, cv::DIST_L2, 0, 0.01, 0.01);
        auto Svk_k = line[1] / line[0];
        auto Svk_b = line[3] - Svk_k * line[2];
        Svk = k + b - (Svk_k + Svk_b);
        Smr2 = (*Smr2_it_x).x;

#ifdef DEBUG_DRAW
        cv::line(board, cv::Point(BOARD_SIZE * tmp_pts[0].x, BOARD_SIZE - Svk_k * tmp_pts[0].x - Svk_b), cv::Point(BOARD_SIZE * tmp_pts[tmp_pts.size() - 1].x, BOARD_SIZE - Svk_k * tmp_pts[tmp_pts.size() - 1].x - Svk_b), cv::Scalar(255, 0, 0));
#endif //DEBUG_DRAW

        A1 = Smr1 * Spk / 2;
        A2 = (1.0 - Smr2) * Svk / 2;

#ifdef DEBUG_DRAW
        cv::imshow("board", board);
        cv::waitKey();
#endif //DEBUG_DRAW
    }

    template<typename T>
    double Mr<T>::Vv(double p) {
        auto f = [&](double x) {
            return MaterialCurve(x);
        };
        double result = i_QAG.integrate(f, p, 1.0);
        return (1.0-p) * MaterialCurve(p) - result;
    }

    template<typename T>
    double Mr<T>::Vm(double p) {
        auto f = [&](double x) {
            return MaterialCurve(x);
        };
        double result = i_QAG.integrate(f, 0.0, p);
        return result - p * MaterialCurve(p);
    }
}

#endif //M_MATH_M_MR_H