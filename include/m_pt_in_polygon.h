//
// Created by Harold on 2021/5/28.
//

#ifndef M_MATH_M_PT_IN_POLYGON_H
#define M_MATH_M_PT_IN_POLYGON_H

#include <vector>
#include <Eigen/Dense>

namespace M_MATH {
    // reference: https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    // explainations: https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
    bool IsPtInsidePolygon(Eigen::Vector2d const& pt, std::vector<Eigen::Vector2d> const& polygon_pts) {
        bool inside = false;
        auto ptx = pt.x(), pty = pt.y();
        Eigen::Vector2d verti, vertj;
        for (int i = 0, j = polygon_pts.size() - 1; i < polygon_pts.size(); j = i++) {
            verti = polygon_pts[i], vertj = polygon_pts[j];
            if ((verti.y() > pty) != (vertj.y() > pty) &&
                ptx < (vertj.x() - verti.x()) * (pty - verti.y()) / (vertj.y() - verti.y()) + verti.x())
                inside = !inside;
        }
        return inside;
    }
}

#endif //M_MATH_M_PT_IN_POLYGON_H