//
// Created by Harold on 2021/11/29.
//

#ifndef M_MATH_M_CV_IO_HPP
#define M_MATH_M_CV_IO_HPP

#include <fstream>
#include <iomanip>
#include <opencv2/core.hpp>

//                 | C1   C2   C3   C4
// CV_8U  | uchar  | 0    8    16   24 |  -0 %8
// CV_8S  | char   | 1    9    17   25 |  -1 %8
// CV_16U |        | 2    10   18   26 |  -2 %8
// CV_16S | short  | 3    11   19   27 |  -3 %8
// CV_32S | int    | 4    12   20   28 |  -4 %8
// CV_32F | float  | 5    13   21   29 |  -5 %8
// CV_64F | double | 6    14   22   30 |  -6 %8

namespace M_MATH
{
    void write_binary(std::string const& filename, cv::Mat const& matrix)
    {
        int rows = matrix.rows, cols = matrix.cols, step = matrix.step, type = matrix.type();

        std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
        out.write((char*) (&rows), sizeof(int));
        out.write((char*) (&cols), sizeof(int));
        out.write((char*) (&step), sizeof(int));
        out.write((char*) (&type), sizeof(int));
        out.write((char*) matrix.data, rows * step * sizeof(uchar));
        out.close();
    }

    void read_binary(std::string const& filename, cv::Mat& matrix)
    {
        int rows = 0, cols = 0, step = 0, type = 0;

        std::ifstream in(filename, std::ios::in | std::ios::binary);
        in.read((char*) (&rows), sizeof(int));
        in.read((char*) (&cols), sizeof(int));
        in.read((char*) (&step), sizeof(int));
        in.read((char*) (&type), sizeof(int));
        matrix = cv::Mat(rows, cols, type);
        in.read((char*) matrix.data, rows * step * sizeof(uchar));
        in.close();
    }
}

#endif //M_MATH_M_CV_IO_HPP