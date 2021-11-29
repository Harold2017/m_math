//
// Created by Harold on 2021/11/29.
//

#include "m_eigen_io.hpp"

using namespace M_MATH;

int main()
{
    std::string filename = "matrix.dat";
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(6, 3);
    std::cout << "A:\n" << A << '\n' << std::endl;
    write_binary(filename, A);

    Eigen::MatrixXd B;
    read_binary(filename, B);
    std::cout << "B:\n" << B << '\n' << std::endl;

    std::cout << "A == B: " << std::boolalpha << (A == B) << std::endl;

    return std::remove(filename.c_str());
}