//
// Created by Harold on 2020/12/30.
//

#ifndef M_MATH_M_INTERP_HM_H
#define M_MATH_M_INTERP_HM_H

#include <algorithm>

namespace M_MATH {
    class Interpolation {
    public:
        /**
         * linear interpolation, requires input pts are sorted and without duplicates
         * @tparam T
         * @param pts
         * @param x
         * @return
         */
        template<typename T, typename IT>
        static T Interpolate(IT begin, IT end, T x);
    };

    template<typename T, typename IT>
    T Interpolation::Interpolate(IT begin, IT end, T x) {
        auto less = [](decltype(*begin) const& p, T x) {
            return p.x < x;
        };
        auto it = std::lower_bound(begin, end, x, less);
        // out of range
        if (it == end)
            return (*(end - 1)).y;
        if (it == begin && x <= (*begin).x)
            return (*begin).y;
        // interpolation
        return (*(it - 1)).y + (x - (*(it - 1)).x) / ((*it).x - (*(it - 1)).x) * ((*it).y - (*(it - 1)).y);
    }
}

#endif //M_MATH_M_INTERP_HM_H
