//
// Created by Harold on 2021/4/9.
//

#include <benchmark/benchmark.h>
#include "m_plane_fit.hpp"
#include "m_plane_fit_eigen.h"

class PlaneFitFixture : public benchmark::Fixture {
public:
    void SetUp(benchmark::State& state) override {
        srand((unsigned int)time(0));
        pts.clear();
        pts1.clear();
        pts.reserve(state.range(0));
        pts1.reserve(state.range(0));
        for (auto i = 0; i < state.range(0); ++i) {
            auto pt = Eigen::Vector3d::Random();
            pts1.push_back(pt);
            pts.push_back(cv::Point3d(pt.x(), pt.y(), pt.z()));
        }
    }

    void TearDown(benchmark::State const& state) override {
        pts.clear();
        pts1.clear();
    }

public:
    std::vector<cv::Point3d> pts;
    std::vector<Eigen::Vector3d> pts1;
};

BENCHMARK_DEFINE_F(PlaneFitFixture, PFCV_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        M_MATH::PlaneFit(pts);
    }
}

BENCHMARK_DEFINE_F(PlaneFitFixture, PFEIGEN_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        M_MATH::PlaneFit(pts1);
    }
}

BENCHMARK_REGISTER_F(PlaneFitFixture, PFCV_Mask)
    -> Arg(256)
    -> Arg(512)
    -> Arg(1024)
    -> Arg(2048)
    -> Arg(4096);

BENCHMARK_REGISTER_F(PlaneFitFixture, PFEIGEN_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

/*
Run on (8 X 3600 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 8192 KiB (x1)
----------------------------------------------------------------------------
Benchmark                                  Time             CPU   Iterations
----------------------------------------------------------------------------
PlaneFitFixture/PFCV_Mask/256           4636 ns         4604 ns       149333
PlaneFitFixture/PFCV_Mask/512           7114 ns         7150 ns        89600
PlaneFitFixture/PFCV_Mask/1024         12169 ns        11998 ns        56000
PlaneFitFixture/PFCV_Mask/2048         23392 ns        23542 ns        29867
PlaneFitFixture/PFCV_Mask/4096         45101 ns        45516 ns        15448
PlaneFitFixture/PFEIGEN_Mask/256        3518 ns         3449 ns       194783
PlaneFitFixture/PFEIGEN_Mask/512        6397 ns         6417 ns       112000
PlaneFitFixture/PFEIGEN_Mask/1024      12572 ns        12556 ns        56000
PlaneFitFixture/PFEIGEN_Mask/2048      32899 ns        32959 ns        21333
PlaneFitFixture/PFEIGEN_Mask/4096      62255 ns        61654 ns        11151
*/