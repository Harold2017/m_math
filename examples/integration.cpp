//
// Created by Harold on 2020/9/30.
//

#include <iostream>
#include "m_integration.h"

using namespace M_MATH;

int main() {
    // f: ax2 + bx + c
    struct f_params{
        double a, b, c;
    };
    auto f = [](double x, void* params) {
        auto params_ = *(f_params*) params;
        return params_.a * x * x + params_.b * x + params_.c;
    };
    auto integration = IntegrationQAG();
    double result, error;
    auto ps = f_params{1, 2, 3};
    integration.bind_func(f, (void*)(&ps));
    integration.integrate(0, 10, &result, &error);

    std::cout << "result: " << result << ", error: " << error << std::endl;

    double a = 1, b = 2, c = 3;

    auto g = [=](double x, void*) {
        return a * x * x + b * x + c;
    };
    integration.bind_func(Lambda::ptr<double>(g), nullptr);
    integration.integrate(0, 10, &result, &error);

    std::cout << "result: " << result << ", error: " << error << std::endl;

    return 0;
}
