//
// Created by Harold on 2020/10/16.
//

#ifndef M_MATH_M_HIST_HM_H
#define M_MATH_M_HIST_HM_H

#include <vector>

namespace M_MATH {
    template<typename T, typename IT, typename GetterFP>
    std::vector<size_t> Histogram(IT const begin, IT const end, GetterFP gfp, T min, T max, size_t N = 20) {
        T deltaZ = (max - min) / N;
        std::vector<size_t> hist(N, 0);
        for (auto it = begin; it != end; ++it)
            // [min, min + deltaZ) -> hist[0]
            if (gfp(*it) < min + deltaZ) ++hist[0];
                // [max - deltaZ, max] -> hist[N-1]
            else if (gfp(*it) >= max - deltaZ) ++hist[N-1];
                // [min + deltaZ : deltaZ : max - deltaZ)
            else ++hist[int((gfp(*it) - min) / deltaZ)];
        return hist;
    }
}

#endif //M_MATH_M_HIST_HM_H
