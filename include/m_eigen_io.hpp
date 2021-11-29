//
// Created by Harold on 2021/11/29.
//

#ifndef M_MATH_M_EIGEN_IO_HPP
#define M_MATH_M_EIGEN_IO_HPP

#include <fstream>
#include <iomanip>
#include <open3d/Open3D.h>

namespace M_MATH
{
    template<typename Derived>
    void write_binary(std::string const& filename, Eigen::PlainObjectBase<Derived> const& matrix)
    {
        using Index = Derived::Index;
        using Scalar = Derived::Scalar;

        Index rows = matrix.rows(), cols = matrix.cols();

        std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
        out.write((char*) (&rows), sizeof(Index));
        out.write((char*) (&cols), sizeof(Index));
        out.write((char*) matrix.data(), rows * cols * sizeof(Scalar) );
        out.close();
    }

    template<typename Derived>
    void read_binary(std::string const& filename, Eigen::PlainObjectBase<Derived>& matrix)
    {
        using Index = Derived::Index;
        using Scalar = Derived::Scalar;

        Index rows = 0, cols = 0;

        std::ifstream in(filename, std::ios::in | std::ios::binary);
        in.read((char*) (&rows), sizeof(Index));
        in.read((char*) (&cols), sizeof(Index));
        matrix.resize(rows, cols);
        in.read((char*) matrix.data(), rows * cols * sizeof(Scalar));
        in.close();
    }
}

#endif //M_MATH_M_EIGEN_IO_HPP