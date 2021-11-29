//
// Created by Harold on 2021/11/29.
//

#include <iostream>
#include "m_cv_io.hpp"

using namespace M_MATH;

int main()
{
    std::string filename = "matrix.dat";
    cv::Mat A(6, 3, CV_32FC3);
    cv::randu(A, cv::Scalar(-10), cv::Scalar(10));
    std::cout << "A:\n" << A << '\n' << std::endl;
    write_binary(filename, A);

    cv::Mat B;
    read_binary(filename, B);
    std::cout << "B:\n" << B << '\n' << std::endl;

    std::cout << "A == B: " << std::boolalpha << (cv::sum(A != B) == cv::Scalar(0)) << std::endl;

    return std::remove(filename.c_str());
}