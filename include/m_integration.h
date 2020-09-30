//
// Created by Harold on 2020/9/30.
//

#ifndef M_MATH_M_INTEGRATION_H
#define M_MATH_M_INTEGRATION_H

#include <gsl/gsl_integration.h>

namespace M_MATH {
    class IntegrationQAG {
    public:
        typedef double (*func_type)(double x, void* params);
    public:
        IntegrationQAG() : IntegrationQAG(1000, 1e-6) { }

        IntegrationQAG(size_t workspace_sz, double accuracy)
                : workspace_size(workspace_sz), accuracy_bound(accuracy), F() {
            workspace = gsl_integration_workspace_alloc(workspace_size);
        }

        ~IntegrationQAG() {
            gsl_integration_workspace_free(workspace);
        }

        void bind_func(func_type f, void* params) {
            F.function = f;
            F.params = params;
        }

        void integrate(double x0, double x1, double* result, double* error) {
            // default use key = 1: GSL_INTEG_GAUSS15
            gsl_integration_qag(&F, x0, x1, 0, accuracy_bound, workspace_size, 1, workspace, result, error);
        }
    private:
        gsl_function F;
        size_t workspace_size;
        double accuracy_bound;
        gsl_integration_workspace* workspace;
    };

    // change lambda function to function ptr
    struct Lambda {
        template<typename Tret, typename T>
        static Tret lambda_ptr_exec(double x, void* data) {
            return (Tret) (*(T*)fn<T>())(x, data);
        }

        template<typename Tret = void, typename Tfp = Tret(*)(double, void*), typename T>
        static Tfp ptr(T& t) {
            fn<T>(&t);
            return (Tfp) lambda_ptr_exec<Tret, T>;
        }

        template<typename T>
        static void* fn(void* new_fn = nullptr) {
            static void* fn;
            if (new_fn != nullptr)
                fn = new_fn;
            return fn;
        }
    };
}

#endif //M_MATH_M_INTEGRATION_H
