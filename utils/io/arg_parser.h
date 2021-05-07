//
// Created by Harold on 2021/5/6.
//

#ifndef M_MATH_IO_ARG_PARSER_H
#define M_MATH_IO_ARG_PARSER_H

#include <string>
#include <algorithm>
#include <vector>
#include <cstdlib>
#include <cerrno>
#include <climits>

namespace M_ARG_PARSER {
    bool OptionExists(int argc, char** argv, std::string const& option) {
        return std::find(argv, argv + argc, option) != argv + argc;
    }

    bool OptionExistsAny(int argc, char** argv, std::vector<std::string> const& options) {
        for (auto const& op : options)
            if (OptionExists(argc, argv, op))
                return true;
        return false;
    }

    std::string ParseAsString(int argc, char** argv, std::string const& option, std::string const& default_value /* = ""*/) {
        char** it = std::find(argv, argv + argc, option);
        return (it != argv + argc && ++it != argv + argc) ? std::string(*it) : default_value;
    }

    int ParseAsInt(int argc, char** argv, std::string const& option, int const& default_value /* = 0*/) {
        std::string str = ParseAsString(argc, argv, option, "");
        if (str.empty()) return default_value;
        //return std::stoi(str);  // stoi will throw out_of_range exception if errno == ERANGE, so use std::strtol instead to avoid exception
        errno = 0;
        char* end;
        long l = std::strtol(str.c_str(), &end, 0);
        return (errno == ERANGE || l < INT_MIN || INT_MAX < l || *end != '\0') ? default_value : static_cast<int>(l);
    }

    float ParseAsFloat(int argc, char** argv, std::string const& option, float const& default_value /* = 0.f*/) {
        std::string str = ParseAsString(argc, argv, option, "");
        if (str.empty()) return default_value;
        errno = 0;
        char* end;
        float f = std::strtof(str.c_str(), &end);
        return (errno == ERANGE || *end != '\0') ? default_value : f;
    }

    double ParseAsDouble(int argc, char** argv, std::string const& option, double const& default_value /* = 0.0*/) {
        std::string str = ParseAsString(argc, argv, option, "");
        if (str.empty()) return default_value;
        errno = 0;
        char* end;
        double d = std::strtod(str.c_str(), &end);
        return (errno == ERANGE || *end != '\0') ? default_value : d;
    }
}

#endif //M_MATH_IO_ARG_PARSER_H