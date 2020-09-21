//
// Created by Harold on 2020/9/21.
//

#ifndef M_MATH_M_FILTER_H
#define M_MATH_M_FILTER_H

#include <gsl/gsl_filter.h>
#include <gsl/gsl_vector.h>

namespace M_MATH {
    class GaussianFilter {
    public:
        enum EndpointType {
            NO_PAD,
            ZERO_PAD,
            VALUE_PAD
        };
    public:
        static bool filter(double const* x,
                           size_t length,
                           size_t window_size,
                           double alpha,
                           EndpointType type,
                           size_t n_length,
                           double* n_x,
                           size_t order);
    private:
        static gsl_filter_end_t EndpointType2GSL(EndpointType);
    };

    gsl_filter_end_t GaussianFilter::EndpointType2GSL(GaussianFilter::EndpointType type) {
        switch (type) {
            case NO_PAD:
                return GSL_FILTER_END_TRUNCATE;
            case ZERO_PAD:
                return GSL_FILTER_END_PADZERO;
            case VALUE_PAD:
                return GSL_FILTER_END_PADVALUE;
        }
        // default is no pad
        // no padding is performed, and the windows are simply truncated as the end points are approached
        return GSL_FILTER_END_TRUNCATE;
    }

    // filter with nth order Gaussian filter, default order = 0
    bool GaussianFilter::filter(const double *x, size_t length, size_t window_size, double alpha,
                                GaussianFilter::EndpointType type, size_t n_length, double *n_x,
                                size_t order = 0) {
        // workspace
        gsl_filter_gaussian_workspace *gauss_p = gsl_filter_gaussian_alloc(window_size);
        if (gauss_p == nullptr)
            return false;
        // xx, yy
        gsl_vector *xx = gsl_vector_alloc(length);
        gsl_vector *yy = gsl_vector_alloc(length);
        for (auto i = 0; i < length; ++i) {
            gsl_vector_set(xx, i, x[i]);
        }

        /* print kernel to debug
        // kernel
        gsl_vector *k = gsl_vector_alloc(window_size);
        // compute kernel without normalization
        gsl_filter_gaussian_kernel(alpha, 0, 0, k);
        // print kernel
        printf("%s", "kernel: ");
        for (auto i = 0; i < window_size; ++i)
            printf("%e ", gsl_vector_get(k, i));
        printf("\n");
        gsl_vector_free(k);
         */

        // apply filter
        if (gsl_filter_gaussian(EndpointType2GSL(type), alpha, order, xx, yy, gauss_p) != 0) {
            gsl_vector_free(xx);
            gsl_vector_free(yy);
            gsl_filter_gaussian_free(gauss_p);
            return false;
        }

        for (auto i = 0; i < n_length; ++i) {
            n_x[i] = gsl_vector_get(yy, i);
        }

        // clean up
        gsl_vector_free(xx);
        gsl_vector_free(yy);
        gsl_filter_gaussian_free(gauss_p);

        return true;
    }
}

#endif //M_MATH_M_FILTER_H
