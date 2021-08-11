//
// Created by Harold on 2021/8/11.
//

#include <iostream>
#include <open3d/Open3D.h>
#include <fstream>
#include "../utils/io/arg_parser.h"
#include "../utils/io/filesystem.h"

struct condition {
    bool valid = false;
    char arg;
    std::string expr;
    double val; 
};

static std::string const expr[] = { ">", "<", ">=", "<=", "==" };

condition ParseCondition(std::string s) {
    if (s.empty())
        return {};
    condition c;
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
    c.arg = s[0];
    for (auto op : expr) {
        auto pos = s.find(op);
        if (pos != std::string::npos) {
            auto substr = s.substr(pos + op.length());
            // >=, <=
            if (substr.find("=") != std::string::npos) {
                op += "=";
                substr = substr.substr(1);
            }
            c.expr = op;
            c.val = std::stod(substr);
            c.valid = true;
            break;
        }
    }
    return c;
}

bool Filter(Eigen::Vector3d const& pt, condition const& c) {
    double arg;
    if (c.arg == 'x')
        arg = pt.x();
    if (c.arg == 'y')
        arg = pt.y();
    if (c.arg == 'z')
        arg = pt.z();
    
    if (c.expr == ">")
        return arg > c.val;
    if (c.expr == "<")
        return arg < c.val;
    if (c.expr == ">=")
        return arg >= c.val;
    if (c.expr == "<=")
        return arg <= c.val;
    if (c.expr == "==")
        return arg == c.val;
    return false;
}

// -i: input pcd filepath (.xyz/.xyzn)
// -f: filter condition ("z > 0.1")
// -o: output pcd filepath (.xyz/.xyzn)
int main(int argc, char* argv[]) {
    // pcd file io
    auto in_file = M_ARG_PARSER::ParseAsString(argc, argv, "-i", "");
    if (in_file.empty()) {
        std::cerr << "invalid input filename: " << in_file << std::endl;
        exit(1);
    }
    auto out_file = M_ARG_PARSER::ParseAsString(argc, argv, "-o", "");
    if (out_file.empty()) {
        std::cerr << "invalid output filename: " << in_file << std::endl;
        exit(1);
    }
    auto in_ext = M_FILESYSTEM::GetFileExtensionLower(in_file);
    auto p_source = open3d::io::CreatePointCloudFromFile(in_file, in_ext);
    if (!p_source) {
        std::cerr << "can not read in file: " << in_file << std::endl;
        exit(1);
    }

    // read in filter condition
    auto f_con = M_ARG_PARSER::ParseAsString(argc, argv, "-f", "");
    auto con = ParseCondition(f_con);
    if (!con.valid) {
        std::cerr << "invalid condition: " << f_con << std::endl;
        exit(1);
    }

    // filter
    open3d::geometry::PointCloud pcd;
    pcd.points_.reserve(p_source->points_.size());
    std::copy_if(p_source->points_.cbegin(), p_source->points_.cend(), std::back_inserter(pcd.points_), [=](Eigen::Vector3d const& pt) { return Filter(pt, con); });
    
    // write output pcd file
    auto ret = open3d::io::WritePointCloud(out_file, pcd);
    if (!ret) {
        std::cerr << "fail to write output file: " << out_file << std::endl;
        exit(1);
    }

    return 0;
}