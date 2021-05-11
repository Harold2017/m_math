//
// Created by Harold on 2021/5/11.
//

#include "m_robust_gaussian_regression.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main() {
	unsigned N = 200;
	int board_size = 800;
	double A = board_size / 2;
	Eigen::VectorXd data(N);
	for (auto i = 0; i < N; ++i)
		data(i) = A * sin((EIGEN_PI / 2 + EIGEN_PI / N * i)) + 0.1 * A * rand() / (RAND_MAX / 2) - 1;

	auto res = M_MATH::RGR2_1D(data);

	cv::Mat board = cv::Mat(board_size, board_size, CV_8UC3);
	std::vector<cv::Point> dpts, rpts;
	dpts.reserve(N);
	rpts.reserve(N);
	for (auto i = 0; i < N; i++) {
		dpts.emplace_back(board_size / N * i, A - data(i));  // image (0, 0) is at left-up
		rpts.emplace_back(board_size / N * i, A - res(i));
	}
	cv::polylines(board, dpts, false, cv::Scalar(0, 255, 0), 1, 8, 0);
	cv::polylines(board, rpts, false, cv::Scalar(255, 255, 255), 1, 8, 0);
	cv::imshow("board", board);
	cv::waitKey();

    return 0;
}