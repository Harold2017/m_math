//
// Created by Harold on 2021/9/15.
//

#ifndef M_MATH_M_ARC_H
#define M_MATH_M_ARC_H

#include <opencv2/core.hpp>

namespace M_MATH {
    template<typename T>
    struct Arc {
        cv::Point_<T> center;
        T starting_angle;
        T arc_angle;
        T radius;
    };

    template<typename T>
    struct Circle {
        cv::Point_<T> center;
        T radius;
    };

    template<typename T>
    Circle<T> GetCircle(cv::Point_<T> const& pt1,
                        cv::Point_<T> const& pt2,
                        cv::Point_<T> const& pt3) {
        auto A = pt1.x * (pt2.y - pt3.y) - pt1.y * (pt2.x - pt3.x) + pt2.x * pt3.y - pt3.x * pt2.y;
        auto B = (pt1.dot(pt1)) * (pt3.y - pt2.y) + (pt2.dot(pt2)) * (pt1.y - pt3.y) + (pt3.dot(pt3)) * (pt2.y - pt1.y);
        auto C = (pt1.dot(pt1)) * (pt2.x - pt3.x) + (pt2.dot(pt2)) * (pt3.x - pt1.x) + (pt3.dot(pt3)) * (pt1.x - pt2.x);
        auto D = (pt1.dot(pt1)) * (pt3.x * pt2.y - pt2.x * pt3.y) + pt2.dot(pt2) * (pt1.x * pt3.y - pt3.x * pt1.y) + (pt3.dot(pt3)) * (pt2.x * pt1.y - pt1.x * pt2.y);
        auto center = cv::Point_<T>( -B / (2 * A), -C / (2 * A) );
        auto radius = std::sqrt((B * B + C * C - 4 * A * D) / (4 * A * A));
        return Circle<T>{ center, radius };
    }

    // in radians
    template<typename T>
    T GetAngle(cv::Point_<T> const& pt1,
               cv::Point_<T> const& vertex,
               cv::Point_<T> const& pt2) {
        auto c = cv::norm(pt1 - vertex);
        auto b = cv::norm(pt1 - pt2);
        auto a = cv::norm(vertex - pt2);
        return std::acos((a * a + c * c - b * b) / (2 * a * c));  // [0, PI]
    }

    // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
    template<typename T>
    bool IsPointInsideTriangle(cv::Point_<T> const& p,
                               cv::Point_<T> const& t1,
                               cv::Point_<T> const& t2,
                               cv::Point_<T> const& t3) {
        auto s = t1.y * t3.x - t1.x * t3.y + (t3.y - t1.y) * p.x + (t1.x - t3.x) * p.y;
        auto t = t1.x * t2.y - t1.y * t2.x + (t1.y - t2.y) * p.x + (t2.x - t1.x) * p.y;

        if ((s < 0) != (t < 0))
            return false;

        auto A = -t2.y * t3.x + t1.y * (t3.x - t2.x) + t1.x * (t2.y - t3.y) + t2.x * t3.y;
        return A < 0 ? (s <= 0 && s + t >= A) : (s >= 0 && s + t <= A);
    }

    template<typename T>
    Arc<T> GetArc(cv::Point_<T> const& pt1,
                  cv::Point_<T> const& pt2,
                  cv::Point_<T> const& pt3) {
        auto circle = GetCircle(pt1, pt2, pt3);
        auto starting_angle = static_cast<T>(cv::fastAtan2(pt1.y - circle.center.y, pt1.x - circle.center.x) * CV_2PI / 360.);
        auto arc_angle = GetAngle(pt1, circle.center, pt3);
        if (GetAngle(pt1, pt2, pt3) > static_cast<T>(CV_PI / 2.))  // angle<pt1, pt2, pt3> > 90 degrees means the arc angle > 180 degrees
            arc_angle = CV_2PI - arc_angle;
        return Arc<T>{ circle.center,
                       starting_angle,
                       arc_angle,
                       circle.radius };
    }
}

#endif //M_MATH_M_ARC_H