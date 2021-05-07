//
// Created by Harold on 2021/5/6.
//

#include "../utils/io/arg_parser.h"
#include "../utils/io/arg_parser_opencv.h"
#include "../utils/io/arg_parser_eigen.h"

#include <cassert>

using namespace M_ARG_PARSER;

int main() {
    // actually argv is not const and NULL terminated
    // but here since not change argv, just use const char* as argv[x] for simplification
    char* argv[] = { "program name", "-s", "string", "-i", "1", "-f", "0.5", "-d", "2.0", "-eigen", "[1, 2, 3]", "-cv", "[3, 2, 1]", NULL };
    int argc = sizeof(argv) / sizeof(char*) - 1;

    assert(OptionExists(argc, argv, "-s"));
    assert(OptionExists(argc, argv, "-i"));
    assert(OptionExists(argc, argv, "-f"));
    assert(OptionExists(argc, argv, "-d"));
    assert(OptionExists(argc, argv, "-eigen"));
    assert(OptionExists(argc, argv, "-cv"));

    assert(OptionExistsAny(argc, argv, {"-f"}));

    assert(ParseAsString(argc, argv, "-s", "") == "string");
    assert(ParseAsInt(argc, argv, "-i", 0) == 1);
    assert(ParseAsFloat(argc, argv, "-f", 0.f) == 0.5f);
    assert(ParseAsDouble(argc, argv, "-d", 0.0) == 2.0);

    Eigen::VectorXd vec(3);
    vec << 1, 2, 3;
    assert(ParseAsEigenVectorXd(argc, argv, "-eigen", Eigen::VectorXd::Zero(3)) == vec);

    assert(ParseAsOpenCVVec3d(argc, argv, "-cv", cv::Vec3d(0, 0, 0)) == cv::Vec3d(3, 2, 1));

    return 0;
}