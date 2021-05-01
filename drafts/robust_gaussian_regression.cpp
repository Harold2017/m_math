//
// Created by Harold on 2021/5/1.
//

#include <Eigen/Core>

#include <iostream>

Eigen::MatrixXd RGR0_2D(Eigen::MatrixXd const& data) {
	auto ny = data.rows();
	auto nx = data.cols();
	double constant = sqrt(log(2) / 2 / EIGEN_PI / EIGEN_PI);
	double lambdax = 0.25;
	double lambday = 0.25;
	double dx = 0.01;
	double dy = 0.01;

	int iterationNo = 0;
	Eigen::MatrixXd delta = Eigen::MatrixXd::Ones(ny, nx);
	double CBx = 1;
	double CB_old = 1;

	Eigen::MatrixXd w4(ny, nx);
	Eigen::MatrixXd r4(ny, nx);

	while (CBx > 0.00000003) {
		for (auto kx = 0; kx < nx; ++kx)
			for (auto ky = 0; ky < ny; ++ky) {
				auto px = Eigen::VectorXd::LinSpaced(nx, 0, nx);
				auto py = Eigen::VectorXd::LinSpaced(ny, 0, ny);
				Eigen::VectorXd v1 = (-0.5 * ((py.array() - ky) * dy / constant / lambday)).array().pow(2);
				Eigen::RowVectorXd v2 = (-0.5 * ((px.array() - kx) * dx / constant / lambday)).array().pow(2).transpose();
				Eigen::MatrixXd S = (1 / sqrt(2 * EIGEN_PI) / pow(constant, 2) / lambdax / lambday) * v1 * v2;
				Eigen::MatrixXd SMOD = S / S.sum();
				w4(ky, kx) = (SMOD * data * delta).sum() / (SMOD * delta).sum();
			}
		r4 = data - w4;
		double CB_next = 4.4 * r4.array().abs().sum() / r4.size();
		CBx = abs((CB_old - CB_next) / CB_old);
		for (auto i = 0; i < nx; i++)
			for (auto j = 0; j < ny; j++) {
				if (abs(r4(j, i) / CB_next) < 1)
					delta(j, i) = pow(1 - pow(r4(j, i) / CB_next, 2), 2);
				else
					delta(j, i) = 0;
			}
		CB_old = CB_next;
		++iterationNo;
		std::cout << "CBx: " << CBx << ", iteration no: " << iterationNo << std::endl;
		if (iterationNo > 9)
			break;
	}
	return w4;
}

int main() {
	Eigen::MatrixXd data(15, 15);
	data.setRandom();
	//std::cout << data << std::endl;
	auto res = RGR0_2D(data);
	std::cout << res << std::endl;

	return 0;
}