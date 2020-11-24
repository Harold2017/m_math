//
// Created by Harold on 2020/11/24.
//

#include "m_io_opencv.h"
#include "m_opencv_utils.h"

using namespace M_MATH;

int main() {
    int N = 100;
    std::vector<float> vz;
    vz.reserve(N);
    for (auto i = 0; i < N; ++i)
        vz.emplace_back(i);
    auto mat = ToMat(10, 10, vz.data());

    std::string filename = "mat.yaml";
    std::string matName = "mat";
    WriteToYAML(mat, filename, matName);

    cv::Mat mat1;
    ReadFromYAML(mat1, filename, matName);

    assert(IsEqual(mat, mat1));

    std::vector<float> vz1;
    ToVec(mat1, vz1);

    for (auto i = 0; i < N; ++i)
        assert(vz[i] == vz1[i]);

    return 0;
}
