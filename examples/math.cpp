//
// Created by Harold on 2020/9/16.
//

#include <vector>
#include "m_math.h"
#include "point.h"

using namespace M_MATH;

int main() {
    size_t N = 10;

    std::vector<int> vi;
    vi.reserve(N);

    std::vector<Point2D<float>> vp;
    vp.reserve(N);
    for (int i = 0; i < N; ++i) {
        vi.emplace_back(i);
        vp.emplace_back(i, i);
    }

    auto gfp = [](Point2D<float> const& p) -> double { return p.y; };

    // sum
    assert(45 == sum(vi.begin(), vi.end()));
    assert(45 == sum(vp.begin(), vp.end(), gfp));

    // add
    auto vii = vi;
    add(vii.begin(), vii.end(), 1);
    assert(55 == sum(vii.begin(), vii.end()));
    auto vpp = vp;
    add<Point2D<float>>(vpp.begin(), vpp.end(), vpp.begin(), vpp.begin());
    assert(90 == sum(vpp.begin(), vpp.end(), gfp));

    // minus
    minus(vii.begin(), vii.end(), 1);
    assert(45 == sum(vii.begin(), vii.end()));
    minus<Point2D<float>>(vpp.begin(), vpp.end(), vpp.begin(), vpp.begin());
    assert(0 == sum(vpp.begin(), vpp.end(), gfp));

    // mean
    assert(4.5 == mean(vi.begin(), vi.end()));
    assert(4.5 == mean(vp.begin(), vp.end(), gfp));

    return 0;
}
