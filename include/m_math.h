//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_M_MATH_H
#define M_MATH_M_MATH_H

#include <cmath>
#include <algorithm>
#include <utility>

namespace M_MATH {
    // sum
    // or use std::accumulate(begin, end, T{});
    template<typename IT>
    inline double sum(IT const begin, IT const end) {
        double s = 0;
        for (IT it = begin; it != end; ++it)
            s += *it;
        return s;
    }
    // e.g. sum += p.x, gfp = [](T const& p) { return p.x; }
    template<typename IT, typename GetterFp>
    inline double sum(IT const begin, IT const end, GetterFp gfp) {
        double s = 0;
        for (IT it = begin; it != end; ++it)
            s += gfp(*it);
        return s;
    }

    template<typename T>
    struct plus_const: public std::unary_function<T, T> {
        T addend;

        explicit plus_const(T const& data): addend(data) { }
        T operator()(T const& x) const { return x + addend; }
    };

    // add const and store in the original series
    template<typename T, typename IT>
    void add(IT begin, IT end, T addend) {
        std::transform(begin, end, begin, plus_const<T>(addend));
    }
    // add two equal-length series
    template<typename T, typename IT>
    void add(IT begin, IT end, IT begin_addend, IT begin_res) {
        std::transform(begin, end, begin_addend, begin_res, std::plus<T>());
    }

    // minus const and store in the original series
    template<typename T, typename IT>
    void minus(IT begin, IT end, T minuend) {
        std::transform(begin, end, begin, plus_const<T>(-minuend));
    }
    // diff two equal-length series
    template<typename T, typename IT>
    void minus(IT begin, IT end, IT begin_minuend, IT begin_res) {
        std::transform(begin, end, begin_minuend, begin_res, std::minus<T>());
    }

    // mean
    template<typename IT>
    inline double mean(IT const begin, IT const end) {
        return sum(begin, end) / (double)(std::distance(begin, end));
    }
    template<typename IT, typename GetterFp>
    inline double mean(IT const begin, IT const end, GetterFp gfp) {
        return sum(begin, end, gfp) / (double)(std::distance(begin, end));
    }

    // central moment, var: 2nd order
    template<typename IT>
    inline double central_moment(IT const begin, IT const end, int order) {
        double m = mean(begin, end);
        double sd = 0;
        for (IT it = begin; it != end; ++it)
            sd += pow((*it - m), order);
        return sd / double(std::distance(begin, end));
    }
    // central moment, var: 2nd order
    template<typename IT, typename GetterFp>
    inline double central_moment(IT const begin, IT const end, int order, GetterFp gfp) {
        double m = mean(begin, end, gfp);
        double sd = 0;
        for (IT it = begin; it != end; ++it)
            sd += pow((gfp(*it) - m), order);
        return sd / double(std::distance(begin, end));
    }

    // origin moment
    template<typename IT>
    inline double origin_moment(IT const begin, IT const end, int order) {
        double s = 0;
        for (IT it = begin; it != end; ++it)
            s += pow(*it, order);
        return s / double(std::distance(begin, end));
    }
    // origin moment
    template<typename IT, typename GetterFp>
    inline double origin_moment(IT const begin, IT const end, int order, GetterFp gfp) {
        double s = 0;
        for (IT it = begin; it != end; ++it)
            s += pow(gfp(*it), order);
        return s / double(std::distance(begin, end));
    }

    // variance, for sample variance, need sd / (len - 1) (unbiased)
    template<typename IT>
    inline double var(IT const begin, IT const end) {
        double m = mean(begin, end);
        double sd = 0;
        for (IT it = begin; it != end; ++it)
            sd += ((*it - m) * (*it - m));
        auto len = std::distance(begin, end);
        return sd / double(len > 1 ? len - 1 : len);
    }
    // variance, for sample variance, need sd / (len - 1) (unbiased)
    template<typename IT, typename GetterFp>
    inline double var(IT const begin, IT const end, GetterFp gfp) {
        double m = mean(begin, end, gfp);
        double sd = 0;
        for (IT it = begin; it != end; ++it)
            sd += ((gfp(*it) - m) * (gfp(*it) - m));
        auto len = std::distance(begin, end);
        return sd / double(len > 1 ? len - 1 : len);
    }

    // standard variance
    template<typename IT>
    inline double std_var(IT const begin, IT const end) {
        return sqrt(var(begin, end));
    }
    // standard variance
    template<typename IT, typename GetterFp>
    inline double std_var(IT const begin, IT const end, GetterFp gfp) {
        return sqrt(var(begin, end, gfp));
    }

    // skewness
    template<typename IT>
    inline double skewness(IT const begin, IT const end) {
        return central_moment(begin, end, 3) / pow(std_var(begin, end), 3);
    }
    // skewness
    template<typename IT, typename GetterFp>
    inline double skewness(IT const begin, IT const end, GetterFp gfp) {
        return central_moment(begin, end, 3, gfp) / pow(std_var(begin, end, gfp), 3);
    }

    // kurtosis
    template<typename IT>
    inline double kurtosis(IT const begin, IT const end) {
        return central_moment(begin, end, 4) / pow(var(begin, end), 2);
    }
    // kurtosis
    template<typename IT, typename GetterFp>
    inline double kurtosis(IT const begin, IT const end, GetterFp gfp) {
        return central_moment(begin, end, 4, gfp) / pow(var(begin, end, gfp), 2);
    }

    // min max element
    // std::minmax_element(begin, end);

    // peak to peak value
    template<typename IT>
    inline double peak_to_peak_value(IT const begin, IT const end) {
        auto const minmax = std::minmax_element(begin, end);
        return (double)(*minmax.second) - (double)(*minmax.first);
    }
    // peak to peak value
    template<typename IT, typename LessFp>
    inline double peak_to_peak_value(IT const begin, IT const end, LessFp lfp) {
        auto const minmax = std::minmax_element(begin, end, lfp);
        return (double)(*minmax.second) - (double)(*minmax.first);
    }

    // series minus mean, in-place
    template<typename IT>
    inline void diff_mean(IT begin, IT end) {
        double m = mean(begin, end);
        for (IT it = begin; it != end; ++it)
            *it -= m;
    }
    // series minus mean, in-place
    template<typename IT, typename GetterFp>
    inline void diff_mean(IT begin, IT end, GetterFp gfp) {
        double m = mean(begin, end, gfp);
        for (IT it = begin; it != end; ++it)
            gfp(*it) -= m;
    }

    // trim series: if data > max, data = max; if data < min, data = min
    template<typename IT, typename T>
    inline void trim(IT begin, IT end, T min, T max) {
        for (IT it = begin; it != end; ++it)
            if (*it < min)
                *it = min;
            else if (*it > max)
                *it = max;
    }
    // trim series: if data > max, data = max
    template<typename IT, typename T>
    inline void trim_upper(IT begin, IT end, T max) {
        for (IT it = begin; it != end; ++it)
            if (*it > max)
                *it = max;
    }
    // trim series: if data < min, data = min
    template<typename IT, typename T>
    inline void trim_lower(IT begin, IT end, T min) {
        for (IT it = begin; it != end; ++it)
            if (*it < min)
                *it = min;
    }

    // 5 points
    template<typename IN_IT, typename OUT_IT>
    inline void smooth_linear(IN_IT const in_begin, IN_IT const in_end, OUT_IT out_begin) {
        auto N = std::distance(in_begin, out_begin);
        IN_IT in = in_begin;
        OUT_IT out = out_begin;
        // if N < 5, no smooth
        if (N < 5) {
            for (; in != in_end; ++in)
                *out = *in;
        } else {
            *out = (3.0 * (*in) + 2.0 * (*(in+1)) + (*(in+2)) - (*(in+4))) / 5.0;
            *(out+1) = ( 4.0 * (*in) + 3.0 * (*(in+1)) + 2 * (*(in+2)) + (*(in+3)) ) / 10.0;
            for (auto i = 2; i < N - 3; i++)
                *(out+i) = (*(in+i-2) + *(in+i-1) + *(in+i) + *(in+i+1) + *(in+i+2)) / 5.0;
            *(out+N-2) = (4.0 * (*(in+N-1)) + 3.0 * (*(in+N-2)) + 2 * (*(in+N-3)) + (*(in+N-4))) / 10.0;
            *(out+N-1) = (3.0 * (*(in+N-1)) + 2.0 * (*(in+N-2)) + (*(in+N-3)) - (*(in+N-5))) / 5.0;
        }
    }

    // 5 points
    template<typename IN_IT, typename OUT_IT>
    inline void smooth_quadratic(IN_IT const in_begin, IN_IT const in_end, OUT_IT out_begin) {
        auto N = std::distance(in_begin, out_begin);
        IN_IT in = in_begin;
        OUT_IT out = out_begin;
        // if N < 5, no smooth
        if (N < 5) {
            for (; in != in_end; ++in)
                *out = *in;
        } else {
            *out = (31.0 * (*in) + 9.0 * (*(in+1)) - 3.0 * (*(in+2)) - 5.0 * (*(in+3)) + 3.0 * (*(in+4))) / 35.0;
            *(out+1) = (9.0 * (*in) + 13.0 * (*(in+1)) + 12 * (*(in+2)) + 6.0 * (*(in+3)) - 5.0 * (*(in+4))) / 35.0;
            for (auto i = 2; i <= N - 3; i++)
            {
                *(out+i) = (- 3.0 * (*(in+i-2) + *(in+i+2)) +
                            12.0 * (*(in+i-1) + *(in+i+1)) + 17 * (*(in+i))) / 35.0;
            }
            *(out+N-2) = ( 9.0 * (*(in+N-1)) + 13.0 * (*(in+N-2)) + 12.0 * (*(in+N-3)) + 6.0 * (*(in+N-4)) - 5.0 * (*(in+N-5)) ) / 35.0;
            *(out+N-1) = ( 31.0 * (*(in+N-1)) + 9.0 * (*(in+N-2)) - 3.0 * (*(in+N-3)) - 5.0 * (*(in+N-4)) + 3.0 * (*(in+N-5))) / 35.0;
        }
    }

    // 5 points
    template<typename IN_IT, typename OUT_IT>
    inline void smooth_cubic(IN_IT const in_begin, IN_IT const in_end, OUT_IT out_begin) {
        auto N = std::distance(in_begin, out_begin);
        IN_IT in = in_begin;
        OUT_IT out = out_begin;
        // if N < 5, no smooth
        if (N < 5) {
            for (; in != in_end; ++in)
                *out = *in;
        } else {
            *out = (69.0 * (*in) + 4.0 * (*(in+1)) - 6.0 * (*(in+2)) + 4.0 * (*(in+3)) - (*(in+4))) / 70.0;
            *(out+1) = (2.0 * (*in) + 27.0 * (*(in+1)) + 12.0 * (*(in+2)) - 8.0 * (*(in+3)) + 2.0 * (*(in+4))) / 35.0;
            for (auto i = 2; i <= N - 3; i++)
            {
                *(out+i) = (-3.0 * (*(in+i-2) + *(in+i+2))+ 12.0 * (*(in+i-1) + *(in+i+1)) + 17.0 * (*(in+i))) / 35.0;
            }
            *(out+N-2) = (2.0 * (*(in+N-5)) - 8.0 * (*(in+N-4)) + 12.0 * (*(in+N-3)) + 27.0 * (*(in+N-2)) + 2.0 * (*(in+N-1))) / 35.0;
            *(out+N-1) = (- (*(in+N-5)) + 4.0 * (*(in+N-4)) - 6.0 * (*(in+N-3)) + 4.0 * (*(in+N-2)) + 69.0 * (*(in+N-1))) / 70.0;
        }
    }

    // nth order difference: m points -> m-n points
    template<typename IN_IT, typename OUT_IT>
    inline void diff(IN_IT in_begin, IN_IT in_end, OUT_IT out_begin, size_t n) {
        IN_IT j = in_begin;
        OUT_IT k = out_begin;
        for (auto i = 0; i < n; ++i)
            for (; j != (in_end-n); ++j, ++k)
                *k = *(j+1) - *j;
    }
}

#endif //M_MATH_M_MATH_H
