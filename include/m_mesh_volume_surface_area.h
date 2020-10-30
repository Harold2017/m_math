//
// Created by Harold on 2020/10/30.
//

#ifndef M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
#define M_MATH_M_MESH_VOLUME_SURFACE_AREA_H

#include <cmath>

namespace M_MATH {

    // A = sqrt(s * (s-a) * (s-b) * (s-c)), where s = (a+b+c)/2
    double AreaOfTriangle(double p1X, double p1Y, double p1Z,
                          double p2X, double p2Y, double p2Z,
                          double p3X, double p3Y, double p3Z) {
        double ax = p2X - p1X;
        double ay = p2Y - p1Y;
        double az = p2Z - p1Z;
        double bx = p3X - p1X;
        double by = p3Y - p1Y;
        double bz = p3Z - p1Z;
        double cx = ay*bz - az*by;
        double cy = az*bx - ax*bz;
        double cz = ax*by - ay*bx;

        return 0.5 * sqrt(cx*cx + cy*cy + cz*cz);
    }

    // V = p1.Dot(p2.Cross(p3)) / 6.0;
    double SignedVolumeOfTriangle(double p1X, double p1Y, double p1Z,
                                  double p2X, double p2Y, double p2Z,
                                  double p3X, double p3Y, double p3Z) {
        double v321 = p3X*p2Y*p1Z;
        double v231 = p2X*p3Y*p1Z;
        double v312 = p3X*p1Y*p2Z;
        double v132 = p1X*p3Y*p2Z;
        double v213 = p2X*p1Y*p3Z;
        double v123 = p1X*p2Y*p3Z;
        return (double)(1.0/6.0)*(-v321 + v231 + v312 - v132 - v213 + v123);
    }

    // require closed mesh with no intersecting/overlapping triangles
    void MeshSurface(double *X, double *Y, double *Z,
                     int *numT,
                     int *V1, int *V2, int *V3,
                     double *Area) {
        *Area = 0.0;
        for (auto n = 0; n < *numT; n++) {
            *Area += AreaOfTriangle(X[V1[n]], Y[V1[n]], Z[V1[n]],
                                    X[V2[n]], Y[V2[n]], Z[V2[n]],
                                    X[V3[n]], Y[V3[n]], Z[V3[n]]);
        }
    }

    // require closed mesh with no intersecting/overlapping triangles
    void MeshVolume(double *X, double *Y, double *Z, int *numT, int *V1, int *V2, int *V3, double *Volume) {
        int n;
        *Volume=0;
        for (n=0; n<*numT; n++) {
            *Volume = *Volume +
                    SignedVolumeOfTriangle(X[V1[n]], Y[V1[n]], Z[V1[n]],
                                           X[V2[n]], Y[V2[n]], Z[V2[n]],
                                           X[V3[n]], Y[V3[n]], Z[V3[n]]);
        }
        *Volume = fabs(*Volume);
    }
}

#endif //M_MATH_M_MESH_VOLUME_SURFACE_AREA_H
