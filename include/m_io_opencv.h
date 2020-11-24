//
// Created by Harold on 2020/11/24.
//

#ifndef M_MATH_M_IO_OPENCV_H
#define M_MATH_M_IO_OPENCV_H

#include <opencv2/core.hpp>

namespace M_MATH {
    void WriteToYAML(cv::Mat const& mat,
                     std::string const& filename,
                     std::string const& matName) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        fs << matName << mat;
        fs.release();
    }

    void ReadFromYAML(cv::Mat &mat,
                      std::string const& filename,
                      std::string const& matName) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        fs[matName] >> mat;
        fs.release();
    }
}

#endif //M_MATH_M_IO_OPENCV_H
