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

BENCHMARK_DEFINE_F(PlaneFitFixture, PFEIGENSVD_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        M_MATH::PlaneFitSVD(pts1);
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

BENCHMARK_REGISTER_F(PlaneFitFixture, PFEIGENSVD_Mask)
    ->Arg(256)
    ->Arg(512)
    ->Arg(1024)
    ->Arg(2048)
    ->Arg(4096);

/*
Run on (8 X 3600 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 8192 KiB (x1)
-------------------------------------------------------------------------------
Benchmark                                     Time             CPU   Iterations
-------------------------------------------------------------------------------
PlaneFitFixture/PFCV_Mask/256              4633 ns         4604 ns       149333
PlaneFitFixture/PFCV_Mask/512              7088 ns         7115 ns       112000
PlaneFitFixture/PFCV_Mask/1024            12288 ns        12277 ns        56000
PlaneFitFixture/PFCV_Mask/2048            23418 ns        22949 ns        32000
PlaneFitFixture/PFCV_Mask/4096            45104 ns        44504 ns        15448
PlaneFitFixture/PFEIGEN_Mask/256           3437 ns         3453 ns       203636
PlaneFitFixture/PFEIGEN_Mask/512           6432 ns         6417 ns       112000
PlaneFitFixture/PFEIGEN_Mask/1024         12557 ns        12556 ns        56000
PlaneFitFixture/PFEIGEN_Mask/2048         32752 ns        32884 ns        20907
PlaneFitFixture/PFEIGEN_Mask/4096         58404 ns        59291 ns         8960
PlaneFitFixture/PFEIGENSVD_Mask/256       11210 ns        11230 ns        64000
PlaneFitFixture/PFEIGENSVD_Mask/512       19676 ns        19496 ns        34462
PlaneFitFixture/PFEIGENSVD_Mask/1024      44540 ns        44504 ns        15448
PlaneFitFixture/PFEIGENSVD_Mask/2048      89925 ns        89979 ns         7467
*/