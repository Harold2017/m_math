//
// Created by Harold on 2020/9/23.
//

#include <iostream>
#include "m_peak.h"

int main() {
    float signal[14] = {0,-2,1,3,0,1,0,5,1,-3,1,1,7,1};
    std::vector<float> in(signal, signal + sizeof(signal) / sizeof(float));
    std::vector<int> out;

    M_MATH::Peak1D::find_peaks(in, out);

    std::cout << "Maxima found:" << std::endl;

    for(int i : out)
        std::cout << in[i]<<" ";
    std::cout << std::endl;

    return 0;
}
