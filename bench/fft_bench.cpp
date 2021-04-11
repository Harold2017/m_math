//
// Created by Harold on 2021/4/12.
//

#include <benchmark/benchmark.h>
#include "m_fft_opencv.h"
#include "m_fft_eigen.hpp"
#include <opencv2/core/eigen.hpp>

class FFTFixture : public benchmark::Fixture {
public:
    void SetUp(benchmark::State& state) override {
        srand((unsigned int)time(0));
        auto N = state.range(0);
        pts = cv::Mat(N, N, CV_32FC1);
        pts1 = Eigen::MatrixXf::Random(N, N);
        cv::eigen2cv(pts1, pts);
    }

    void TearDown(benchmark::State const& state) override {
    }

public:
    cv::Mat pts, out;
    Eigen::MatrixXf pts1;
};

BENCHMARK_DEFINE_F(FFTFixture, FFTCV_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        M_MATH::ForwardFFT(pts, out);
    }
}

BENCHMARK_DEFINE_F(FFTFixture, FFTEIGEN_Mask)(benchmark::State& state) {
    for (auto _ : state) {
        M_MATH::ForwardFFT(pts1);
    }
}

BENCHMARK_DEFINE_F(FFTFixture, FFTEIGEN_Mask_OMP)(benchmark::State& state) {
    for (auto _ : state) {
        M_MATH::ForwardFFT_OMP(pts1);
    }
}

BENCHMARK_REGISTER_F(FFTFixture, FFTCV_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FFTFixture, FFTEIGEN_Mask)
        -> Arg(256)
        -> Arg(512)
        -> Arg(1024)
        -> Arg(2048)
        -> Arg(4096);

BENCHMARK_REGISTER_F(FFTFixture, FFTEIGEN_Mask_OMP)
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
Load Average: 2.57, 3.14, 2.92
-------------------------------------------------------------------------------
Benchmark                                     Time             CPU   Iterations
-------------------------------------------------------------------------------
FFTFixture/FFTCV_Mask/256             589051 ns       583399 ns         1297
FFTFixture/FFTCV_Mask/512            2057893 ns      2054978 ns          313
FFTFixture/FFTCV_Mask/1024          11559522 ns     11521076 ns           66
FFTFixture/FFTCV_Mask/2048          55861082 ns     55739000 ns            9
FFTFixture/FFTCV_Mask/4096         234159352 ns    233953667 ns            3
FFTFixture/FFTEIGEN_Mask/256         2501553 ns      2500071 ns          269
FFTFixture/FFTEIGEN_Mask/512        11335397 ns     11316651 ns           63
FFTFixture/FFTEIGEN_Mask/1024       59493094 ns     59348455 ns           11
FFTFixture/FFTEIGEN_Mask/2048      284279181 ns    283801000 ns            2
FFTFixture/FFTEIGEN_Mask/4096     1316980038 ns   1313545000 ns            1
FFTFixture/FFTEIGEN_Mask_OMP/256     3923713 ns      3922028 ns          176
FFTFixture/FFTEIGEN_Mask_OMP/512    17648613 ns     17556412 ns           34
FFTFixture/FFTEIGEN_Mask_OMP/1024   82441030 ns     82301889 ns            9
FFTFixture/FFTEIGEN_Mask_OMP/2048  316862660 ns    315979500 ns            2
FFTFixture/FFTEIGEN_Mask_OMP/4096 1372252860 ns   1368940000 ns            1
 */