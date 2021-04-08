//
// Created by Harold on 2021/4/8.
//

#include <benchmark/benchmark.h>
#include "m_curvefit_opencv.h"
#include "m_curvefit_eigen.h"
#include "utils.h"

static void BM_CurveFitCV(benchmark::State& state) {
    auto x = linspace(0.1, 2.0, 100);
    auto y = linspace(1.0, 20.0, 100);
    for (auto _ : state) {
        auto cf = M_MATH::CurveFitCV();
        cf.polyfit(x.data(), y.data(), x.size(), 10);
    }
}

static void BM_CurveFitEigen(benchmark::State& state) {
    auto x = linspace(0.1, 2.0, 100);
    auto y = linspace(1.0, 20.0, 100);
    for (auto _ : state) {
        auto cf = M_MATH::CurveFitEigen();
        cf.polyfit(x.data(), y.data(), x.size(), 10);
    }
}

BENCHMARK(BM_CurveFitCV);
BENCHMARK(BM_CurveFitEigen);
