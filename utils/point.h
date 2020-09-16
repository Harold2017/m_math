//
// Created by Harold on 2020/9/16.
//

#ifndef M_MATH_POINT_H
#define M_MATH_POINT_H

#include <ostream>
#include <cmath>
#include "utils.h"

template<typename T,
        typename = EnableIfFloatingPoint<T>>
struct Point2D {
    T x, y;
    Point2D() = default;
    Point2D(const T x, const T y): x(x), y(y) { }
    friend std::ostream& operator<<(std::ostream& os, const Point2D& point) {
        return os << "(" << point.x << ", " << point.y << ")";
    }
    friend Point2D operator-(Point2D const& p1, Point2D const& p2) {
        return Point2D(p1.x-p2.x, p1.y-p2.y);
    }
    friend Point2D operator+(Point2D const& p1, Point2D const& p2) {
        return Point2D(p1.x+p2.x, p1.y+p2.y);
    }
    Point2D& operator-=(Point2D const& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }
    Point2D& operator+=(Point2D const& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
    bool operator==(Point2D const& rhs) {
        return x == rhs.x && y == rhs.y;
    }
};

template<typename T,
        typename = EnableIfFloatingPoint<T>>
struct Point3D {
    T x, y, z;
    Point3D() = default;
    Point3D(T x, T y, T z): x(x), y(y), z(z) { }
    Point3D normalized() {
        auto n = std::sqrt(x * x + y * y + z * z);
        return Point3D(x/n, y/n, z/n);
    }
    friend std::ostream& operator<<(std::ostream& os, const Point3D& point) {
        return os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    }
    friend T dot(const Point3D<T>& p1, const Point3D<T>& p2) {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    }
    friend Point3D<T> cross(const Point3D<T>& p1, const Point3D<T>& p2) {
        return Point3D<T>(p1.y * p2.z - p1.z * p2.y, p1.z * p2.x - p1.x * p2.z, p1.x * p2.y - p1.y * p2.x);
    }
};

#endif //M_MATH_POINT_H
