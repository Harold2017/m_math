//
// Created by Harold on 2020/10/19.
//

#ifndef M_MATH_M_OPENCV_UTILS_H
#define M_MATH_M_OPENCV_UTILS_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace M_MATH {
    // all use float inside
    inline cv::Mat ToMat(size_t rows, size_t cols, float *data) {
        return cv::Mat(rows, cols, CV_32FC1, data);
    }

    inline cv::Mat ToMat(size_t rows, size_t cols, double *data) {
        cv::Mat out;
        cv::Mat(rows, cols, CV_64FC1, data).convertTo(out, CV_32FC1);
        return out;
    }

    // vector<Point3> to Mat (n X 3)
    template<typename T>
    inline cv::Mat ToMat(std::vector<cv::Point3_<T>> const& pts) {
        return cv::Mat(pts).reshape(1);
    }

    // copy data from array to mat
    inline cv::Mat ToMatCopy(size_t rows, size_t cols, float *data) {
        cv::Mat out(rows, cols, CV_32FC1);
        std::memcpy(out.data, data, rows * cols * sizeof(float));
        return out;
    }

    inline cv::Mat ToMatCopy(size_t rows, size_t cols, double *data) {
        cv::Mat out(rows, cols, CV_64FC1);
        std::memcpy(out.data, data, rows * cols * sizeof(double));
        out.convertTo(out, CV_32FC1);
        return out;
    }

    template<typename T>
    inline cv::Mat ToMatCopy(size_t rows, size_t cols, std::vector<T> const& vec) {
        cv::Mat_<T> res(rows, cols);
        memcpy(res.data, vec.data(), vec.size() * sizeof(T));
        return res;
    }

    inline void ToVec(cv::Mat const& in, std::vector<float> & out) {
        if (in.isContinuous()) {
            // array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
            out.assign((float*)in.data, (float*)in.data + in.total()*in.channels());
        } else {
            for (int i = 0; i < in.rows; ++i) {
                out.insert(out.end(), in.ptr<float>(i), in.ptr<float>(i)+in.cols*in.channels());
            }
        }
    }

    // Mat (n X 3) to vector<Point3>
    inline void ToVec(cv::Mat const& in, std::vector<cv::Point3f> & out) {
        in.reshape(3, in.rows * in.cols).copyTo(out);
    }

    // in should in [0, 1]
    inline cv::Mat To8U(cv::Mat const& in) {
        double min, max;
        cv::minMaxLoc(in, &min, &max);
        cv::Mat out;
        if (min != max)
            in.convertTo(out, CV_8U, 255.0/(max-min),-255.0*min/(max-min));
        else
            if (min == 0)
                out = cv::Mat::zeros(in.rows, in.cols, CV_8U);
            else
                out = cv::Mat::ones(in.rows, in.cols, CV_8U);
        return out;
    }

    bool IsEqual(const cv::Mat Mat1, const cv::Mat Mat2)
    {
        if( Mat1.dims == Mat2.dims &&
            Mat1.size == Mat2.size &&
            Mat1.elemSize() == Mat2.elemSize())
        {
            if( Mat1.isContinuous() && Mat2.isContinuous())
            {
                return 0 == memcmp( Mat1.ptr(), Mat2.ptr(), Mat1.total()*Mat1.elemSize());
            }
            else
            {
                // a null terminated list of pointers
                const cv::Mat* arrays[] = {&Mat1, &Mat2, nullptr};
                uchar* ptrs[2];
                cv::NAryMatIterator it( arrays, ptrs, 2);
                for(unsigned int p = 0; p < it.nplanes; p++, ++it)
                    if( 0 != memcmp( it.ptrs[0], it.ptrs[1], it.size*Mat1.elemSize()) )
                        return false;

                return true;
            }
        }

        return false;
    }

    template <typename T>
    cv::Mat PlotLineSeries(std::vector<T>& vals, int YRange[2])
    {
        auto it = std::minmax_element(vals.begin(), vals.end());
        float scale = 1. / std::ceil(*it.second - *it.first);
        float bias = *it.first;
        int rows = YRange[1] - YRange[0] + 1;
        cv::Mat image = cv::Mat::zeros(rows, vals.size(), CV_8UC3);
        image.setTo(0);
        for (int i = 0; i < (int)vals.size() - 1; i++)
            cv::line(image, cv::Point(i, rows - 1 - (vals[i] - bias) * scale * YRange[1]), cv::Point(i + 1, rows - 1 - (vals[i + 1] - bias) * scale * YRange[1]), cv::Scalar(255, 0, 0), 1);

        return image;
    }
}

#endif //M_MATH_M_OPENCV_UTILS_H
