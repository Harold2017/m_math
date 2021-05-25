//
// Created by Harold on 2021/5/12.
//

#include "m_gaussian_filter.h"
#include "m_filters_opencv.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

int main() {
	unsigned N = 200;
	int board_size = 800;
	double A = board_size / 2;
	std::vector<double> data(N);
	for (auto i = 0; i < N; ++i)
		data[i] = A * sin((M_PI / 2 + M_PI / N * i)) + 0.1 * A * rand() / (RAND_MAX / 2) - 1;

	double lambdac = 0.8;
	double dx = 0.25;
	int M = (int)(2 * lambdac / dx + 1);

	cv::Mat in(1, N, CV_64F), out(1, N, CV_64F);
	for (auto i = 0; i < N; i++) in.at<double>(0, i) = data[i];
	M_MATH::Gaussian1D(in, out, M);

	// FIXME: seems wrong
	auto kernel = M_MATH::GaussianKernel(M, lambdac, dx);
	auto res = M_MATH::Convolution1DWithEndEffect(kernel, data, lambdac, dx);

	cv::Mat board = cv::Mat(board_size, board_size, CV_8UC3);
	std::vector<cv::Point> dpts, rpts, kpts, gpts;
	dpts.reserve(N);
	rpts.reserve(N);
	kpts.reserve(N);
	gpts.reserve(N);
	for (auto i = 0; i < N; i++) {
		dpts.emplace_back(board_size / N * i, A - data[i]);  // image (0, 0) is at left-up
		rpts.emplace_back(board_size / N * i, A - res[i]);
		kpts.emplace_back(board_size / N * i, A - kernel[i]);
		gpts.emplace_back(board_size / N * i, A - out.at<double>(0, i));
	}
	cv::polylines(board, dpts, false, cv::Scalar(0, 255, 0), 1, 8, 0);
	cv::polylines(board, rpts, false, cv::Scalar(255, 255, 255), 1, 8, 0);
	//cv::polylines(board, kpts, false, cv::Scalar(255, 0, 0), 1, 8, 0);
	cv::polylines(board, gpts, false, cv::Scalar(0, 0, 255), 1, 8, 0);
	cv::imshow("board", board);
	cv::waitKey();

    return 0;
}