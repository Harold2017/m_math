//
// Created by Harold on 2020/10/16.
//

#include "m_integration_hm.h"

int main() {
    auto f = [](double const x) { return std::exp(x); };
    M_MATH::Integration<double> integration;
    std::cout << integration.integrate(f, 0., 1.) << std::endl;

    return 0;
}
