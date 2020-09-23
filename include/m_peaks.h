//
// Created by Harold on 2020/9/17.
//

#ifndef M_MATH_M_PEAKS_H
#define M_MATH_M_PEAKS_H

#include <type_traits>
#include <vector>

namespace M_MATH {
    class Peak2D {
    public:
        // compare with its 8 neighbors
        template<typename T, typename GetterFp>
        static std::pair<int,int> counts_vp(T const& array, size_t row, size_t col, GetterFp fp) {
            int cnt_v = 0 ;
            int cnt_p = 0 ;

            for (int i : { -1, 0, 1 })
                for (int j : { -1, 0, 1 }) {
                    cnt_v += fp(array[row + i][col + j]) > fp(array[row][col]);  // array[row][col] is valley
                    cnt_p += fp(array[row + i][col + j]) < fp(array[row][col]);  // array[row][col] is peak
                }

            return {cnt_v, cnt_p} ;
        }

        // rows >= 2, cols >= 2
        // not consider plateau
        template <typename T, typename GetterFp>
        static auto find_peaks_and_valleys(T const& array, size_t rows, size_t cols, GetterFp fp)
        -> std::pair<std::vector<std::pair<size_t, size_t>>, std::vector<std::pair<size_t, size_t>>> {
            std::vector<std::pair<size_t, size_t>> peaks, valleys;
            for(auto row = 1 ; row < (rows - 1) ; ++row)
                for(auto col = 1 ; col < (cols - 1) ; ++col)
                {
                    const auto cnts = counts_vp(array, row, col, fp) ;
                    if(cnts.first == 8)  // lower than its 8 neighbors
                        valleys.emplace_back(row, col);
                    else if(cnts.second == 8)  // higher than its 8 neighbors
                        peaks.emplace_back(row, col);
                }
            return std::make_pair(peaks, valleys);
        }

        // rows >= 2, cols >= 2
        // not consider plateau
        template <typename T, typename GetterFp>
        static auto find_peaks(T const& array, size_t rows, size_t cols, GetterFp fp)
        -> std::vector<std::pair<size_t, size_t>> {
            std::vector<std::pair<size_t, size_t>> peaks;
            for(auto row = 1 ; row < (rows - 1) ; ++row)
                for(auto col = 1 ; col < (cols - 1) ; ++col)
                {
                    const auto cnts = counts_vp(array, row, col, fp) ;
                    if(cnts.second == 8)  // higher than its 8 neighbors
                        peaks.emplace_back(row, col);
                }
            return peaks;
        }

        // rows >= 2, cols >= 2
        // not consider plateau
        template <typename T, typename GetterFp>
        static auto find_valleys(T const& array, size_t rows, size_t cols, GetterFp fp)
        -> std::vector<std::pair<size_t, size_t>> {
            std::vector<std::pair<size_t, size_t>> valleys;
            for(auto row = 1 ; row < (rows - 1) ; ++row)
                for(auto col = 1 ; col < (cols - 1) ; ++col)
                {
                    const auto cnts = counts_vp(array, row, col, fp) ;
                    if(cnts.first == 8)  // lower than its 8 neighbors
                        valleys.emplace_back(row, col);
                }
            return valleys;
        }
    };
}

#endif //M_MATH_M_PEAKS_H
