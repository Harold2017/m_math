//
// Created by Harold on 2021/9/15.
//

#include "m_arc.hpp"
#include "io/arg_parser_opencv.h"
#include <iostream>

int main(int argc, char* argv[]) {
    if (!(M_ARG_PARSER::OptionExists(argc, argv, "-p1") && M_ARG_PARSER::OptionExists(argc, argv, "-p2") && M_ARG_PARSER::OptionExists(argc, argv, "-p3"))) {
        std::cerr << "wrong arguments, require -p1, -p2, -p3, three points" << std::endl;
        exit(1);
    }

    auto p1 = M_ARG_PARSER::ParseAsOpenCVPoint<float>(argc, argv, "-p1", cv::Point2f());
    auto p2 = M_ARG_PARSER::ParseAsOpenCVPoint<float>(argc, argv, "-p2", cv::Point2f());
    auto p3 = M_ARG_PARSER::ParseAsOpenCVPoint<float>(argc, argv, "-p3", cv::Point2f());

    auto arc = M_MATH::GetArc(p1, p2, p3);

    std::cout << "Arc throught 3 points <p1, p2, p3>:\n"
              << "\tcenter: " << arc.center << '\n'
              << "\tstarting angle in radians: " << arc.starting_angle << '\n'
              << "\tangle in radians: " << arc.arc_angle << '\n'
              << "\tradius: " << arc.radius << '\n'
              << "\tlength: " << arc.arc_angle * arc.radius
              << std::endl;

    return 0;
}