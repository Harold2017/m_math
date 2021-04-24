//
// Created by Harold on 2021/4/24.
//

#ifndef M_MATH_M_LINEFIT_H
#define M_MATH_M_LINEFIT_H

#include <Eigen/Core>
#include <Eigen/SVD>
//#include <Eigen/QR>
#include <Eigen/LU>
#include <vector>
#include <numeric>
#include <random>

namespace M_MATH {
    // y = kx + b
    template<typename T, typename IT, typename GetterFpX, typename GetterFpY>
    bool LineFitLeastSquares(IT begin, IT end, GetterFpX gfpx, GetterFpY gfpy, T& k, T& b) {
        auto N = std::distance(begin, end);
        Eigen::Matrix<T, Eigen::Dynamic, 2> A(N, 2);
        Eigen::Matrix<T, Eigen::Dynamic, 1> B(N, 1);

        for (auto i = 0; i < N; i++) {
            A.row(i) = Eigen::Matrix<T, 2, 1>(gfpx(*(begin + i)), 1);
            B(i) = gfpy(*(begin + i));
        }

        Eigen::Matrix<T, Eigen::Dynamic, 1> solution = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
        //Eigen::Matrix<T, Eigen::Dynamic, 1> solution = A.colPivHouseholderQr().solve(B);
        if (solution.size() == 2) {
            k = solution(0);
            b = solution(1);
            return true;
        }
        return false;
    }

    template<typename T, typename IT, typename GetterFpX, typename GetterFpY>
    bool least_squares_a(IT begin, IT end, GetterFpX gfpx, GetterFpY gfpy, T& k, T& b) {
        auto N = std::distance(begin, end);
        T A{}, B{}, C{}, D{};
        for (auto it = begin; it != end; it++) {
            auto x = gfpx(*it);
            auto y = gfpy(*it);
            A += x * x;
            B += x;
            C += x * y;
            D += y;
        }

        auto temp = (N * A - B * B);
        if (temp) {
            k = (N * C - B * D) / temp;
            b = (A * D - B * C) / temp;
            return true;
        }
        else
            return false;
    }

    template<typename T, typename IT, typename GetterFpX, typename GetterFpY>
    bool least_squares_b(IT begin, IT end, GetterFpX gfpx, GetterFpY gfpy, T& k, T& b) {
        auto N = std::distance(begin, end);
        Eigen::Matrix<T, Eigen::Dynamic, 2> A(N, 2);
        Eigen::Matrix<T, Eigen::Dynamic, 1> B(N, 1);

        for (auto i = 0; i < N; i++) {
            A.row(i) = Eigen::Matrix<T, 2, 1>(gfpx(*(begin + i)), 1);
            B(i) = gfpy(*(begin + i));
        }

        Eigen::FullPivLU<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> LU(A.transpose() * A);
        if (!LU.isInvertible())
            return false;
        Eigen::Matrix<T, Eigen::Dynamic, 1> solution = LU.inverse() * A.transpose() * B;
        if (solution.size() == 2) {
            k = solution(0);
            b = solution(1);
            return true;
        }
        return false;
    }

    // y = w1 + w2 * x
    // loss(w) = (1/2N) * Sum(pow((w1 + w2 * x - y), 2))
    // partial differential on w1: w1 = w1 - alpha * (1/N) * Sum(pow((w1 + w2 * x - y), 2))
    // partial differential on w2: w2 = w2 - alpha * (1/N) * Sum((w1 + w2 * x - y) * x)
    template<typename T, typename IT, typename GetterFpX, typename GetterFpY>
    bool LineFitGradientDescent(IT begin, IT end, GetterFpX gfpx, GetterFpY gfpy, T& k, T& b, double alpha = 0.1, int const num_iterations = 100, double min_loss = 0.001) {
        auto N = std::distance(begin, end);
        double loss = std::numeric_limits<double>::max(), loss_pre{};

        size_t it = 0;
        double w1 = 0, w2 =0;
        double g1, g2;
        double x, y;

        do {
            // the partial derivative of w1, w2
            g1 = 0, g2 = 0;
            for (auto i = 0; i < N; i++) {
                x = gfpx(*(begin + i));
                y = gfpy(*(begin + i));
                g1 = g1 + w1 + w2 * x - y;
                g2 = g2 + (w1 + w2 * x - y) * x;
            }
            // update w1, w2
            w1 = w1 - alpha * g1 / N;
            w2 = w2 - alpha * g2 / N;

            // update loss
            loss_pre = loss;
            // MSE
            loss = 0;
            for (auto i = 0; i < N; i++) {
                x = gfpx(*(begin + i));
                y = gfpy(*(begin + i));
                loss += pow(abs(w1 + x * w2 - y), 2);
            }
            loss /= (2 * N);
        } while (abs(loss - loss_pre) >= min_loss && it++ < num_iterations);

        k = w2;
        b = w1;
        return loss != std::numeric_limits<double>::max() && !isinf(loss);
    }

    // ax + by + c = 0
    namespace details {
        class RANSACResult {
        public:
            RANSACResult() : fitness_(0), inlier_rmse_(0) {}
            ~RANSACResult() {}

        public:
            double fitness_;
            double inlier_rmse_;
        };

        // distance = |ax + by + c| / sqrt(a2 + b2)
        RANSACResult EvaluateRANSACBasedOnDistance(
            const std::vector<Eigen::Vector2d>& points,
            const Eigen::Vector3d line_model,
            std::vector<size_t>& inliers,
            double distance_threshold,
            double error) {
            RANSACResult result;

            for (size_t idx = 0; idx < points.size(); ++idx) {
                Eigen::Vector3d point(points[idx](0), points[idx](1), 1);
                double distance = std::abs(line_model.dot(point));

                if (distance < distance_threshold) {
                    error += distance;
                    inliers.emplace_back(idx);
                }
            }

            size_t inlier_num = inliers.size();
            if (inlier_num == 0) {
                result.fitness_ = 0;
                result.inlier_rmse_ = 0;
            }
            else {
                result.fitness_ = (double)inlier_num / (double)points.size();
                result.inlier_rmse_ = error / std::sqrt((double)inlier_num);
            }
            //std::cout << "(" << line_model(0) << ", " << line_model(1) << ", " << line_model(2) << "); " << inlier_num << ", " << result.fitness_ << ", " << result.inlier_rmse_ << std::endl;
            return result;
        }

        bool least_squares(std::vector<Eigen::Vector2d> const& points, std::vector<size_t> inliers, double& k, double& b) {
            auto N = inliers.size();
            Eigen::MatrixXd A(N, 2);
            Eigen::VectorXd B(N);

            for (auto i = 0; i < N; i++) {
                A.row(i) = Eigen::Vector2d(points[inliers[i]](0), 1);
                B(i) = points[inliers[i]](1);
            }

            Eigen::VectorXd solution = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
            if (solution.size() == 2) {
                k = solution(0);
                b = solution(1);
                return true;
            }
            return false;
        }
    };

    Eigen::Vector2d LineFitRANSAC(std::vector<Eigen::Vector2d> const& points,
        const double distance_threshold /* = 0.01 */,
        const int ransac_n /* = 2 */,
        const int num_iterations /* = 100 */) {
        Eigen::Vector2d  res;
        details::RANSACResult result;
        double error = 0;

        // Initialize the line model ax + by + c = 0.
        Eigen::Vector3d line_model = Eigen::Vector3d(0, 0, 0);
        // Initialize the best plane model.
        Eigen::Vector3d best_line_model = Eigen::Vector3d(0, 0, 0);

        // Initialize consensus set.
        std::vector<size_t> inliers;

        size_t num_points = points.size();
        std::vector<size_t> indices(num_points);
        std::iota(std::begin(indices), std::end(indices), 0);

        std::random_device rd;
        std::mt19937 rng(rd());

        // Return if ransac_n is less than the required plane model parameters.
        if (ransac_n < 2) {
            //std::cerr << "ransac_n should be set to higher than or equal to 2." << std::endl;
            return res;
        }
        if (num_points < size_t(ransac_n)) {
            //std::cerr << "There must be at least 'ransac_n' points." << std::endl;
            return res;
        }

        for (int itr = 0; itr < num_iterations; itr++) {
            for (int i = 0; i < ransac_n; ++i) {
                std::swap(indices[i], indices[rng() % num_points]);
            }
            inliers.clear();
            for (int idx = 0; idx < ransac_n; ++idx) {
                inliers.emplace_back(indices[idx]);
            }

            // Fit model to num_model_parameters randomly selected points among the inliers.
            double k = 0, b = 0;
            details::least_squares(points, inliers, k, b);  // ax + by + c = 0 / kx -y + b = 0
            line_model = Eigen::Vector3d(k, -1, b);
            if (line_model.isZero(0)) {
                continue;
            }

            error = 0;
            inliers.clear();
            auto this_result = details::EvaluateRANSACBasedOnDistance(points, line_model, inliers, distance_threshold, error);
            if (this_result.fitness_ > result.fitness_ ||
                (this_result.fitness_ == result.fitness_ &&
                    this_result.inlier_rmse_ < result.inlier_rmse_)) {
                result = this_result;
                best_line_model = line_model;
            }
        }

        // Find the final inliers using best_line_model.
        inliers.clear();
        for (size_t idx = 0; idx < points.size(); ++idx) {
            Eigen::Vector3d point(points[idx](0), points[idx](1), 1);
            double distance = std::abs(best_line_model.dot(point));

            if (distance < distance_threshold) {
                inliers.emplace_back(idx);
            }
        }

        //std::cout << inliers.size() << std::endl;

        double k = 0, b = 0;
        details::least_squares(points, inliers, k, b);
        res = Eigen::Vector2d(k, b);

        return res;
    }
}

#endif //M_MATH_M_LINEFIT_H