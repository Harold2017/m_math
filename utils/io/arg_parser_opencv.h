//
// Created by Harold on 2021/5/6.
//

#ifndef M_MATH_IO_ARG_PARSER_OPENCV_H
#define M_MATH_IO_ARG_PARSER_OPENCV_H

#include <opencv2/core.hpp>
#include "arg_parser.h"

namespace M_ARG_PARSER {
    cv::Vec3d ParseAsOpenCVVec3d(int argc, char** argv, std::string const& option, cv::Vec3d const& default_value /* = cv::Vec3d(0, 0, 0)*/) {
        std::string str = ParseAsString(argc, argv, option, "");
        if (str.empty() || (!(str.front() == '(' && str.back() == ')') &&
                            !(str.front() == '[' && str.back() == ']') &&
                            !(str.front() == '{' && str.back() == '}') &&
                            !(str.front() == '<' && str.back() == '>')))
            return default_value;
        std::string parsed_str = str.substr(1, str.length() - 2);
        std::vector<std::string> tokens;
        std::string::size_type pos = 0, new_pos = 0, last_pos = 0;
        while (pos != std::string::npos) {
            pos = parsed_str.find_first_of(",", last_pos);
            new_pos = (pos == std::string::npos ? parsed_str.length() : pos);
            if (new_pos != last_pos)
                tokens.push_back(parsed_str.substr(last_pos, new_pos - last_pos));
            last_pos = new_pos + 1;
        }
        cv::Vec3d vec;
        for (size_t i = 0; i < 3; i++) {
            char* end;
            errno = 0;
            double d = std::strtod(tokens[i].c_str(), &end);
            if (errno == ERANGE || *end != '\0')
                return default_value;
            vec(i) = d;
        }
        return vec;
    }
}

#endif //M_MATH_IO_ARG_PARSER_OPENCV_H