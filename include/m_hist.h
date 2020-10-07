//
// Created by Harold on 2020/10/7.
//

#ifndef M_MATH_M_HIST_H
#define M_MATH_M_HIST_H

#include <gsl/gsl_histogram.h>
#include <gsl/gsl_histogram2d.h>
#include <vector>
#include <utility>

namespace M_MATH {
    template<typename T>
    class Histogram1D {
    public:
        // [min, max), it: <x>
        template<typename IT>
        Histogram1D(IT begin, IT end, T min, T max, size_t N);
        ~Histogram1D() { gsl_histogram_free(h); gsl_histogram_pdf_free(p); };
        // counts in every bin
        std::vector<size_t> counts();
        // bins with [lower, upper] boundaries
        std::vector<std::pair<T, T>> bins();
        // probability distribution, r in [0, 1]
        double pdf_sample(double r);
        // standard deviation of the histogrammed variable
        double sigma() { return gsl_histogram_sigma(h); }
        // mean of the histogrammed variable
        double mean() { return gsl_histogram_mean(h); }
        // sum of all bin values
        double sum() { return gsl_histogram_sum(h); }
        // find x in which bin
        size_t find(double x) { size_t idx; gsl_histogram_find(h, x, &idx); return idx; }

    private:
        gsl_histogram *h;
        gsl_histogram_pdf *p;
    };

    template<typename T>
    class Histogram2D {
    public:
        // [min, max), it: <x, y>
        template<typename IT>
        Histogram2D(IT begin, IT end, T xmin, T xmax, T ymin, T ymax, size_t xdim, size_t ydim);
        ~Histogram2D() { gsl_histogram2d_free(h); gsl_histogram2d_pdf_free(p); };
        // counts in every bin
        std::vector<size_t> counts();
        // bins with [lower, upper] boundaries, <xrange, yrange>
        std::pair<std::vector<std::pair<T, T>>, std::vector<std::pair<T, T>>> bins();
        // probability distribution <x, y>, r1, r2 in [0, 1]
        std::pair<double, double> pdf_sample(double r1, double r2);
        // standard deviation of the histogrammed x and y variables
        std::pair<double, double> sigma() { return std::make_pair(gsl_histogram2d_xsigma(h), gsl_histogram2d_ysigma(h)); }
        // mean of the histogrammed x and y variables
        std::pair<double, double> mean() { return std::make_pair(gsl_histogram2d_xmean(h), gsl_histogram2d_ymean(h)); }
        // sum of all bin values
        double sum() { return gsl_histogram2d_sum(h); }
        // covariance of the histogrammed x and y variables
        double cov() { return gsl_histogram2d_cov(h); }
        // find x in which bin, <x, y>
        std::pair<size_t, size_t> find(double x, double y) { size_t xidx, yidx; gsl_histogram2d_find(h, x, y, &xidx, &yidx); return std::make_pair(xidx, yidx); }
    private:
        gsl_histogram2d *h;
        gsl_histogram2d_pdf *p;
    };

    template<typename T>
    template<typename IT>
    Histogram1D<T>::Histogram1D(IT const begin, IT const end, T min, T max, size_t N) : h(nullptr), p(nullptr) {
        h = gsl_histogram_alloc(N);
        gsl_histogram_set_ranges_uniform(h, min, max);
        for (auto it = begin; it != end; ++it)
            gsl_histogram_increment(h, *it);
    }

    template<typename T>
    std::vector<size_t> Histogram1D<T>::counts() {
        std::vector<size_t> res(h->n);
        for (auto i = 0; i < h->n; ++i)
            res[i] = gsl_histogram_get(h, i);
        return res;
    }

    template<typename T>
    std::vector<std::pair<T, T>> Histogram1D<T>::bins() {
        std::vector<std::pair<T, T>> res(h->n);
        double lower, upper;
        for (auto i = 0; i < h->n; ++i) {
            gsl_histogram_get_range(h, i, &lower, &upper);
            res[i] = std::make_pair(lower, upper);
        }
        return res;
    }

    template<typename T>
    double Histogram1D<T>::pdf_sample(double r) {
        if (p == nullptr) {
            p = gsl_histogram_pdf_alloc(h->n);
            gsl_histogram_pdf_init(p, h);
        }
        return gsl_histogram_pdf_sample(p, r);
    }

    template<typename T>
    template<typename IT>
    Histogram2D<T>::Histogram2D(IT begin, IT end,
                                T xmin, T xmax,
                                T ymin, T ymax,
                                size_t xdim, size_t ydim) : h(nullptr), p(nullptr) {
        h = gsl_histogram2d_alloc(xdim, ydim);
        gsl_histogram2d_set_ranges_uniform(h, xmin, xmax, ymin, ymax);
        for (auto it = begin; it != end; ++it)
            gsl_histogram2d_increment(h, (*it).x, (*it).y);
    }

    template<typename T>
    std::vector<size_t> Histogram2D<T>::counts() {
        std::vector<size_t> res((h->nx) * (h->ny));
        for (auto i = 0; i < h->nx; ++i)
            for (auto j = 0; j < h->ny; ++j)
                res[j * (h->nx) + i] = gsl_histogram2d_get(h, i, j);
        return res;
    }

    template<typename T>
    std::pair<std::vector<std::pair<T, T>>, std::vector<std::pair<T, T>>> Histogram2D<T>::bins() {
        std::vector<std::pair<T, T>> xrange(h->nx), yrange(h->ny);
        double lower, upper;
        for (auto i = 0; i < h->nx; ++i) {
            gsl_histogram2d_get_xrange(h, i, &lower, &upper);
            xrange[i] = std::make_pair(lower, upper);
        }
        for (auto i = 0; i < h->ny; ++i) {
            gsl_histogram2d_get_yrange(h, i, &lower, &upper);
            yrange[i] = std::make_pair(lower, upper);
        }
        return std::make_pair(xrange, yrange);
    }

    template<typename T>
    std::pair<double, double> Histogram2D<T>::pdf_sample(double r1, double r2) {
        if (p == nullptr) {
            p = gsl_histogram2d_pdf_alloc(h->nx, h->ny);
            gsl_histogram2d_pdf_init(p, h);
        }
        double x, y;
        gsl_histogram2d_pdf_sample(p, r1, r2, &x, &y);
        return std::make_pair(x, y);
    }
}

#endif //M_MATH_M_HIST_H
