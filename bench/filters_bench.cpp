//
// Created by Harold on 2020/12/16.
//

#include <benchmark/benchmark.h>
#include "m_filters_opencv.h"
#include "m_filters_opencv_1.h"

class FiltersFixture : public benchmark::Fixture {
public:
    void SetUp(benchmark::State& state) override {
        in.create(4096, 4096, CV_32FC1);
        cv::randu(in, cv::Scalar(0.f), cv::Scalar(1.f));
    }

    void TearDown(benchmark::State const& state) override {

    }

public:
    cv::Mat in, out;
};

BENCHMARK_DEFINE_F(FiltersFixture, LP_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::LowPass2D(in_, out, state.range(0) / 2);
    }
}

BENCHMARK_DEFINE_F(FiltersFixture, HP_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::HighPass2D(in_, out, state.range(0) / 2);
    }
}

BENCHMARK_DEFINE_F(FiltersFixture, BP_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::BandPass2D(in_, out, 0, state.range(0) / 2);
    }
}

BENCHMARK_DEFINE_F(FiltersFixture, BR_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::BandReject2D(in_, out, 0, state.range(0) / 2);
    }
}

BENCHMARK_DEFINE_F(FiltersFixture, LP_Without_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::Filter(in_, out, state.range(0) / 2, 0, M_MATH::FilterType::ILP);
    }
}

BENCHMARK_DEFINE_F(FiltersFixture, HP_Without_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::Filter(in_, out, state.range(0) / 2, 0, M_MATH::FilterType::IHP);
    }
}

BENCHMARK_DEFINE_F(FiltersFixture, BP_Without_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::Filter(in_, out, 0, state.range(0) / 2, M_MATH::FilterType::IBP);
    }
}

BENCHMARK_DEFINE_F(FiltersFixture, BR_Without_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto in_ = in(cv::Rect(0, 0, state.range(0), state.range(0)));
        state.ResumeTiming();
        M_MATH::Filter(in_, out, 0, state.range(0) / 2, M_MATH::FilterType::IBR);
    }
}

BENCHMARK_REGISTER_F(FiltersFixture, LP_Mask)
    -> Arg(256)
    -> Arg(512)
    -> Arg(1024)
    -> Arg(2048)
    -> Arg(4096);

BENCHMARK_REGISTER_F(FiltersFixture, HP_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FiltersFixture, BP_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FiltersFixture, BR_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FiltersFixture, LP_Without_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FiltersFixture, HP_Without_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FiltersFixture, BP_Without_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FiltersFixture, BR_Without_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

/*
Run on (4 X 2900 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x2)
  L1 Instruction 32 KiB (x2)
  L2 Unified 256 KiB (x2)
  L3 Unified 3072 KiB (x1)
Load Average: 2.09, 2.10, 2.46
-------------------------------------------------------------------------
Benchmark                               Time             CPU   Iterations
-------------------------------------------------------------------------
FiltersFixture/LP_Mask/256             2076776 ns      2066796 ns          284
FiltersFixture/LP_Mask/512             8225615 ns      8171787 ns           80
FiltersFixture/LP_Mask/1024           45743651 ns     45558500 ns           16
FiltersFixture/LP_Mask/2048          202392831 ns    201793500 ns            4
FiltersFixture/LP_Mask/4096         1161259028 ns   1154758000 ns            1
FiltersFixture/HP_Mask/256             1534420 ns      1533544 ns          430
FiltersFixture/HP_Mask/512             7559517 ns      7555138 ns           94
FiltersFixture/HP_Mask/1024           42517564 ns     42472917 ns           12
FiltersFixture/HP_Mask/2048          181987840 ns    181896333 ns            3
FiltersFixture/HP_Mask/4096         1118164636 ns   1115492000 ns            1
FiltersFixture/BP_Mask/256             1542323 ns      1541227 ns          352
FiltersFixture/BP_Mask/512            18119083 ns     17039674 ns           92
FiltersFixture/BP_Mask/1024          114041147 ns    101810111 ns            9
FiltersFixture/BP_Mask/2048          260086239 ns    257117000 ns            2
FiltersFixture/BP_Mask/4096         1130854175 ns   1125727000 ns            1
FiltersFixture/BR_Mask/256             1539259 ns      1538798 ns          446
FiltersFixture/BR_Mask/512             8299300 ns      8241582 ns           91
FiltersFixture/BR_Mask/1024           45583716 ns     45397500 ns           16
FiltersFixture/BR_Mask/2048          196343080 ns    195646500 ns            4
FiltersFixture/BR_Mask/4096         1116359629 ns   1115087000 ns            1
FiltersFixture/LP_Without_Mask/256     7039282 ns      7035524 ns           82
FiltersFixture/LP_Without_Mask/512    27446720 ns     27427208 ns           24
FiltersFixture/LP_Without_Mask/1024  143725865 ns    143686250 ns            4
FiltersFixture/LP_Without_Mask/2048  596720025 ns    596077000 ns            1
FiltersFixture/LP_Without_Mask/4096 2477242454 ns   2472872000 ns            1
FiltersFixture/HP_Without_Mask/256     6603402 ns      6596204 ns          108
FiltersFixture/HP_Without_Mask/512    27424030 ns     27419292 ns           24
FiltersFixture/HP_Without_Mask/1024  129137397 ns    128858167 ns            6
FiltersFixture/HP_Without_Mask/2048  607451692 ns    606518000 ns            1
FiltersFixture/HP_Without_Mask/4096 2556473872 ns   2547210000 ns            1
FiltersFixture/BP_Without_Mask/256     1855771 ns      1855200 ns          360
FiltersFixture/BP_Without_Mask/512     8803146 ns      8794429 ns           70
FiltersFixture/BP_Without_Mask/1024   53371904 ns     53313200 ns           10
FiltersFixture/BP_Without_Mask/2048  233032251 ns    232472667 ns            3
FiltersFixture/BP_Without_Mask/4096 1283963780 ns   1282238000 ns            1
FiltersFixture/BR_Without_Mask/256     1844059 ns      1843967 ns          359
FiltersFixture/BR_Without_Mask/512     8723653 ns      8719826 ns           69
FiltersFixture/BR_Without_Mask/1024   52529271 ns     52374429 ns           14
FiltersFixture/BR_Without_Mask/2048  240792577 ns    239819333 ns            3
FiltersFixture/BR_Without_Mask/4096 1748417193 ns   1672702000 ns            1
 */