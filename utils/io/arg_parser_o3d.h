//
// Created by Harold on 2021/6/24.
//

#ifndef M_MATH_IO_ARGPARSER_O3D_H
#define M_MATH_IO_ARGPARSER_O3D_H

#include "arg_parser.h"

#include <open3d/Open3D.h>

namespace M_ARG_PARSER {
    Eigen::VectorXd Str2VectorXd(std::string str, Eigen::VectorXd const &default_value)
    {
        str.erase(std::remove_if(str.begin(), str.end(), [](unsigned char x)
                                { return std::isspace(x); }),
                str.end());
        if (str.empty() || (!(str.front() == '(' && str.back() == ')') &&
                            !(str.front() == '[' && str.back() == ']') &&
                            !(str.front() == '{' && str.back() == '}') &&
                            !(str.front() == '<' && str.back() == '>')))
            return default_value;
        std::string parsed_str = str.substr(1, str.length() - 2);
        std::vector<std::string> tokens;
        std::string::size_type pos = 0, new_pos = 0, last_pos = 0;
        while (pos != std::string::npos)
        {
            pos = parsed_str.find_first_of(",", last_pos);
            new_pos = (pos == std::string::npos ? parsed_str.length() : pos);
            if (new_pos != last_pos)
                tokens.push_back(parsed_str.substr(last_pos, new_pos - last_pos));
            last_pos = new_pos + 1;
        }
        Eigen::VectorXd vec(tokens.size());
        for (size_t i = 0; i < tokens.size(); i++)
        {
            char *end;
            errno = 0;
            double d = std::strtod(tokens[i].c_str(), &end);
            if (errno == ERANGE || *end != '\0')
                return default_value;
            vec(i) = d;
        }
        return vec;
    }

    std::vector<Eigen::Vector2d> ParsePolyROI(std::string const &str)
    {
        std::vector<Eigen::Vector2d> res;
        if (str.empty() || (!(str.front() == '(' && str.back() == ')') &&
                            !(str.front() == '[' && str.back() == ']') &&
                            !(str.front() == '{' && str.back() == '}') &&
                            !(str.front() == '<' && str.back() == '>')))
            return res;
        std::string parsed_str = str.substr(1, str.length() - 2);
        std::vector<std::string> tokens;
        std::string::size_type pos = 0, new_pos = 0, last_pos = 0;
        while (pos != std::string::npos)
        {
            pos = parsed_str.find_first_of(";", last_pos);
            new_pos = (pos == std::string::npos ? parsed_str.length() : pos);
            if (new_pos != last_pos)
                tokens.push_back(parsed_str.substr(last_pos, new_pos - last_pos));
            last_pos = new_pos + 1;
        }
        for (auto token : tokens)
            res.push_back(Str2VectorXd(token, Eigen::Vector2d::Zero()));
        return res;
    }

    std::pair<Eigen::Vector2d, double> ParseCircleROI(std::string const &str)
    {
        std::pair<Eigen::Vector2d, double> res;
        if (str.empty() || (!(str.front() == '(' && str.back() == ')') &&
                            !(str.front() == '[' && str.back() == ']') &&
                            !(str.front() == '{' && str.back() == '}') &&
                            !(str.front() == '<' && str.back() == '>')))
            return res;
        std::string parsed_str = str.substr(1, str.length() - 2);
        std::vector<std::string> tokens;
        std::string::size_type pos = 0, new_pos = 0, last_pos = 0;
        while (pos != std::string::npos)
        {
            pos = parsed_str.find_first_of(";", last_pos);
            new_pos = (pos == std::string::npos ? parsed_str.length() : pos);
            if (new_pos != last_pos)
                tokens.push_back(parsed_str.substr(last_pos, new_pos - last_pos));
            last_pos = new_pos + 1;
        }
        res.first = Str2VectorXd(tokens[0], Eigen::Vector2d::Zero());
        char *end;
        res.second = std::strtod(tokens[1].c_str(), &end);
        return res;
    }
}

#endif //M_MATH_IO_ARGPARSER_O3D_H