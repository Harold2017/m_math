//
// Created by Harold on 2021/6/28.
//

#ifndef M_MATH_M_AREA_AUTO_COVARIANCE_H
#define M_MATH_M_AREA_AUTO_COVARIANCE_H

#include <opencv2/core.hpp>
#include <omp.h>

namespace M_MATH {
    template<typename T>
    cv::Mat_<T> AreaAutoCovariance(cv::Mat_<T> const& I) {
        static_assert(std::is_floating_point<T>::value, "T should be floating point");
        auto ny = I.rows;
        auto nx = I.cols;
        cv::Mat_<T> AACV(ny, nx, T{});

#pragma omp parallel for collapse(2)
        for (auto ty = 0; ty < ny; ty++)
            for (auto tx = 0; tx < nx; tx++) {
                auto tmp = T{};
                for (auto j = 0; j < ny - ty; j++)
                    for (auto i = 0; i < nx - tx; i++)
                        if (j + ty - 1 < 0 || i + tx - 1 < 0) continue;
                        else tmp = tmp + I.at<T>(j, i) * I.at<T>(j + ty - 1, i + tx - 1);  // I(-1, -1) is invalid, which means AACV row_0 and col_0 are invalid
                AACV.at<T>(ty, tx) = tmp;
            }
        AACV /= (nx * ny);

        // translate to remove invalid row_0 and col_0
        //cv::Mat trans_mat = (cv::Mat_<T>(2,3) << 1, 0, -1, 0, 1, -1);
        //cv::warpAffine(AACV, AACV, trans_mat, AACV.size());

        // copy to remove invalid row_0 and col_0
        AACV(cv::Rect(1, 1, I.cols - 1, I.rows - 1)).copyTo(AACV(cv::Rect(0, 0, I.cols - 1, I.rows - 1)));
        AACV.row(I.rows - 1) = 0;
        AACV.col(I.cols - 1) = 0;
        return AACV;
    }
}

#endif //M_MATH_M_AREA_AUTO_COVARIANCE_H