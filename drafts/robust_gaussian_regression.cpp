//
// Created by Harold on 2021/5/1.
//

#include <Eigen/Dense>

#include <iostream>

// TODO: verify
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

// FIXME: incorrect
// w(k,p)= Ax2 + Bx+C where x = (k-p)dx
Eigen::VectorXd RGR2_1D(Eigen::VectorXd const& data) {
	auto n = data.size();
	double constant = sqrt(log(2) / 2 / EIGEN_PI / EIGEN_PI);
	double lambdac = 0.8;
	double dx = 0.001;
	Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(n, 0, n) * dx;

	int iterationNo = 0;
	Eigen::VectorXd delta = Eigen::VectorXd::Ones(n);
	double CBx = 1;
	double CB_old = 1;

	Eigen::Matrix3d M;
	Eigen::Vector3d Q;
	Eigen::Vector3d P;
	Eigen::VectorXd w1(n);
	Eigen::VectorXd r1(n);

	while (CBx > 0.00000003) {
		for (auto k = 0; k < n; k++) {
			auto p = Eigen::VectorXd::LinSpaced(n, 0, n);
			Eigen::VectorXd S = (1/sqrt(2 * EIGEN_PI * EIGEN_PI)/constant/lambdac) * (-0.5 * ((k - p.array()) * dx / constant / lambdac)).array().pow(2);
			S /= S.sum();
			x = (k - p.array()) * dx;
			M(0, 0) = (delta.dot(S) * ((x.array().pow(0)))).sum();  // A0
			M(0, 1) = (delta.dot(S) * ((x.array().pow(1)))).sum();  // A1
			M(0, 2) = (delta.dot(S) * ((x.array().pow(2)))).sum();  // A2
			M(1, 0) = M(0, 1);  // A1
			M(1, 1) = M(0, 2);  // A2
			M(1, 2) = (delta.dot(S) * ((x.array().pow(3)))).sum();  // A3
			M(2, 0) = M(0, 2);  // A2
			M(2, 1) = M(1, 2);  // A3
			M(2, 2) = (delta.dot(S) * ((x.array().pow(4)))).sum();  // A4

			Q(0) = (delta.dot(data) * S).sum();  // F0
			Q(1) = (delta.dot(data) * S).dot(x);  // F1
			Q(2) = (delta.dot(data) * S).dot(Eigen::VectorXd(x.array().pow(2)));  // F2

			P = M.inverse() * Q;
			w1(k) = P(0);
		}

		r1 = data - w1;
		double CB_next = 4.4 * r1.array().abs().sum() / r1.size();
		CBx = abs((CB_old - CB_next) / CB_old);
		for (auto j = 0; j < n; j++)
			if (abs(r1(j) / CB_next) < 1)
				delta(j) = pow(1 - pow(r1(j) / CB_next, 2), 2);
			else
				delta(j) = 0;
		CB_old = CB_next;
		++iterationNo;
		std::cout << "CBx: " << CBx << ", iteration no: " << iterationNo << std::endl;
		if (iterationNo > 29)
			break;
	}
	return w1;
}

int main() {
	/*
	Eigen::MatrixXd data(15, 15);
	data.setRandom();
	auto res = RGR0_2D(data);
	std::cout << res << std::endl;
	*/

	Eigen::VectorXd data1(100);
	data1.setRandom();
	std::cout << data1.transpose() <<'\n' << std::endl;
	auto res1 = RGR2_1D(data1);
	std::cout << res1.transpose() << std::endl;

	return 0;
}