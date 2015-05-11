#include "mas/mesh/meshbv_intersector.h"
#include "mas/bvtree/bvtree.h"

#include <unordered_map>
#include <list>
#include <algorithm>

namespace mas {
namespace mesh {

// TriangleIntersector implementation
TriangleIntersector::TriangleIntersector() :
        epsilon(0) {
}

TriangleIntersector::TriangleIntersector(double eps) :
        epsilon(eps) {
}

void TriangleIntersector::setEpsilon(double eps) {
    this->epsilon = eps;
}

double TriangleIntersector::getEpsilon() const {
    return epsilon;
}

/*
 *
 * This file contains C implementation of algorithms for performing two and
 * three-dimensional triangle-triangle intersection test The algorithms and
 * underlying theory are described in
 *
 * "Fast and Robust Triangle-Triangle Overlap Test Using Orientation
 * Predicates" P. Guigue - O. Devillers
 *
 * Journal of Graphics Tools, 8(1), 2003
 *
 * Several geometric predicates are defined. Their parameters are all points.
 * Each point is an array of two or three double precision floating point
 * numbers. The geometric predicates implemented in this file are:
 *
 * int tri_tri_overlap_test_3d(p1,q1,r1,p2,q2,r2) int
 * tri_tri_overlap_test_2d(p1,q1,r1,p2,q2,r2)
 *
 * int tri_tri_intersection_test_3d(p1,q1,r1,p2,q2,r2,
 * coplanar,source,target)
 *
 * is a version that computes the segment of intersection when the triangles
 * overlap (and are not coplanar)
 *
 * each function returns 1 if the triangles (including their boundary)
 * intersect, otherwise 0
 *
 *
 * Other information are available from the Web page
 * http://jgt.akpeters.com/papers/GuigueDevillers03/
 *
 */

int TriangleIntersector::check_min_max(const Vector3d& p1, const Vector3d& q1,
        const Vector3d& r1, const Vector3d& p2, const Vector3d& q2,
        const Vector3d& r2) const {

    Vector3d v1, v2, v, N1;

    v1.subtract(p2, q1);
    v2.subtract(p1, q1);
    N1.subtract(v1, v2);
    v1.subtract(q2, q1);
    if (v1.dot(N1) > 0.0)
        return 0;

    v1.subtract(p2, p1);
    v2.subtract(r1, p1);
    N1.subtract(v1, v2);
    v1.subtract(r2, p1);
    if (v1.dot(N1) > 0.0)
        return 0;
    else
        return 1;
}

int TriangleIntersector::construct_intersection(const Vector3d& p1,
        const Vector3d& q1, const Vector3d& r1, const Vector3d& p2,
        const Vector3d& q2, const Vector3d& r2,
        std::vector<Point3d>& pnts) const {

    Vector3d v1, v2, v, N, N1, N2;

    // get normals of triangles 1&  2, scale doesn't matter

    v1.subtract(p2, r2);
    v2.subtract(q2, r2);
    N2.cross(v1, v2);
    v1.subtract(q1, p1);
    v2.subtract(r1, p1);
    N1.cross(v1, v2);

    // v1.subtract(q1, p1); // same as above
    v2.subtract(r2, p1);
    N.cross(v1, v2);

    v.subtract(p2, p1);

    if (v.dot(N) > 0.0) {
        v1.subtract(r1, p1);
        N.cross(v1, v2);
        if (v.dot(N) <= 0.0) {

            v2.subtract(q2, p1);
            N.cross(v1, v2);
            if (v.dot(N) > 0.0) {
                v1.subtract(p1, p2);
                v2.subtract(p1, r1);
                double alpha = v1.dot(N2) / v2.dot(N2);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[0].subtract(p1, v1);
                v1.subtract(p2, p1);
                v2.subtract(p2, r2);
                alpha = v1.dot(N1) / v2.dot(N1);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[1].subtract(p2, v1);
            } else {
                v1.subtract(p2, p1);
                v2.subtract(p2, q2);
                double alpha = v1.dot(N1) / v2.dot(N1);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[0].subtract(p2, v1);
                v1.subtract(p2, p1);
                v2.subtract(p2, r2);
                alpha = v1.dot(N1) / v2.dot(N1);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[1].subtract(p2, v1);
            }
        } else {
            return 0;
        }
    } else {
        v2.subtract(q2, p1);
        N.cross(v1, v2);
        if (v.dot(N) < 0.0) {
            return 0;
        } else {

            v1.subtract(r1, p1);
            N.cross(v1, v2);
            if (v.dot(N) >= 0.0) {
                v1.subtract(p1, p2);
                v2.subtract(p1, r1);
                double alpha = v1.dot(N2) / v2.dot(N2);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[0].subtract(p1, v1);
                v1.subtract(p1, p2);
                v2.subtract(p1, q1);
                alpha = v1.dot(N2) / v2.dot(N2);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[1].subtract(p1, v1);
            } else {
                v1.subtract(p2, p1);
                v2.subtract(p2, q2);
                double alpha = v1.dot(N1) / v2.dot(N1);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[0].subtract(p2, v1);
                v1.subtract(p1, p2);
                v2.subtract(p1, q1);
                alpha = v1.dot(N2) / v2.dot(N2);
                v1.scale(alpha, v2);
                pnts.push_back(Point3d());
                pnts[1].subtract(p1, v1);
            }
        }
    }

    if (pnts[1].distance(pnts[2]) > epsilon) {
        return 2;
    }
    pnts.pop_back();  // remove last
    return 1;

}

int TriangleIntersector::tri_tri_inter_3d(const Vector3d& p1,
        const Vector3d& q1, const Vector3d& r1, const Vector3d& p2,
        const Vector3d& q2, const Vector3d& r2, double dp2, double dq2,
        double dr2, std::vector<Point3d>& pnts) const {
    if (dp2 > 0.0) {
        if (dq2 > 0.0)
            return construct_intersection(p1, r1, q1, r2, p2, q2, pnts);
        else if (dr2 > 0.0)
            return construct_intersection(p1, r1, q1, q2, r2, p2, pnts);
        else
            return construct_intersection(p1, q1, r1, p2, q2, r2, pnts);
    } else if (dp2 < 0.0) {
        if (dq2 < 0.0)
            return construct_intersection(p1, q1, r1, r2, p2, q2, pnts);
        else if (dr2 < 0.0)
            return construct_intersection(p1, q1, r1, q2, r2, p2, pnts);
        else
            return construct_intersection(p1, r1, q1, p2, q2, r2, pnts);
    } else {
        if (dq2 < 0.0) {
            if (dr2 >= 0.0)
                return construct_intersection(p1, r1, q1, q2, r2, p2, pnts);
            else
                return construct_intersection(p1, q1, r1, p2, q2, r2, pnts);
        } else if (dq2 > 0.0) {
            if (dr2 > 0.0)
                return construct_intersection(p1, r1, q1, p2, q2, r2, pnts);
            else
                return construct_intersection(p1, q1, r1, q2, r2, p2, pnts);
        } else {
            if (dr2 > 0.0)
                return construct_intersection(p1, q1, r1, r2, p2, q2, pnts);
            else if (dr2 < 0.0)
                return construct_intersection(p1, r1, q1, r2, p2, q2, pnts);
            else {
                // coplanar
                return 0; // coplanar_tri_tri_3d (p1, q1, r1, p2, q2, r2, N1,
                          // N2);
            }
        }
    }
}

int TriangleIntersector::intersectTriangleTriangle(const Vector3d& p1,
        const Vector3d& q1, const Vector3d& r1, const Vector3d& p2,
        const Vector3d& q2, const Vector3d& r2,
        std::vector<Point3d>& pnts) const {

    // Compute distance signs of p1, q1 and r1
    // to the plane of triangle(p2,q2,r2)
    Vector3d v1, v2, N1, N2;

    v1.subtract(p2, r2);
    v2.subtract(q2, r2);
    N2.cross(v1, v2);

    v1.subtract(p1, r2);
    double dp1 = v1.dot(N2);
    v1.subtract(q1, r2);
    double dq1 = v1.dot(N2);
    v1.subtract(r1, r2);
    double dr1 = v1.dot(N2);

    // //epsilon test
    // if(dp1 < epsilon && -dp1 < epsilon)
    // dp1 = 0;
    // if(dq1 < epsilon && -dq1 < epsilon)
    // dq1 = 0;
    // if(dr1 < epsilon && -dr1 < epsilon)
    // dr1 = 0;

    if (((dp1 * dq1) > 0.0) && ((dp1 * dr1) > 0.0))
        return 0;

    // Compute distance signs of p2, q2 and r2
    // to the plane of triangle(p1,q1,r1)
    v1.subtract(q1, p1);
    v2.subtract(r1, p1);
    N1.cross(v1, v2);

    v1.subtract(p2, r1);
    double dp2 = v1.dot(N1);
    v1.subtract(q2, r1);
    double dq2 = v1.dot(N1);
    v1.subtract(r2, r1);
    double dr2 = v1.dot(N1);

    // //epsilon test
    // if(dp2 < epsilon && -dp2 < epsilon)
    // dp2 = 0;
    // if(dq2 < epsilon && -dq2 < epsilon)
    // dq2 = 0;
    // if(dr2 < epsilon && -dr2 < epsilon)
    // dr2 = 0;

    if (((dp2 * dq2) > 0.0) && ((dp2 * dr2) > 0.0))
        return 0;

    // Permutation in a canonical form of T1's vertices

    if (dp1 > 0.0) {
        if (dq1 > 0.0)
            return tri_tri_inter_3d(r1, p1, q1, p2, r2, q2, dp2, dr2, dq2, pnts);
        else if (dr1 > 0.0)
            return tri_tri_inter_3d(q1, r1, p1, p2, r2, q2, dp2, dr2, dq2, pnts);

        else
            return tri_tri_inter_3d(p1, q1, r1, p2, q2, r2, dp2, dq2, dr2, pnts);
    } else if (dp1 < 0.0) {
        if (dq1 < 0.0)
            return tri_tri_inter_3d(r1, p1, q1, p2, q2, r2, dp2, dq2, dr2, pnts);
        else if (dr1 < 0.0)
            return tri_tri_inter_3d(q1, r1, p1, p2, q2, r2, dp2, dq2, dr2, pnts);
        else
            return tri_tri_inter_3d(p1, q1, r1, p2, r2, q2, dp2, dr2, dq2, pnts);
    } else {
        if (dq1 < 0.0) {
            if (dr1 >= 0.0)
                return tri_tri_inter_3d(q1, r1, p1, p2, r2, q2, dp2, dr2, dq2,
                        pnts);
            else
                return tri_tri_inter_3d(p1, q1, r1, p2, q2, r2, dp2, dq2, dr2,
                        pnts);
        } else if (dq1 > 0.0) {
            if (dr1 > 0.0)
                return tri_tri_inter_3d(p1, q1, r1, p2, r2, q2, dp2, dr2, dq2,
                        pnts);
            else
                return tri_tri_inter_3d(q1, r1, p1, p2, q2, r2, dp2, dq2, dr2,
                        pnts);
        } else {
            if (dr1 > 0.0)
                return tri_tri_inter_3d(r1, p1, q1, p2, q2, r2, dp2, dq2, dr2,
                        pnts);
            else if (dr1 < 0.0)
                return tri_tri_inter_3d(r1, p1, q1, p2, r2, q2, dp2, dr2, dq2,
                        pnts);
            else {
                // triangles are co-planar

                // coplanar.value = 1;
                return 0;     // coplanar_tri_tri3d (p1, q1, r1, p2, q2, r2, N1,
                              // N2);
            }
        }
    }
}

// triangle-ray intersection code
// http://jgt.akpeters.com/papers/MollerTrumbore97/code.html

/**
 * Determines the bary centric coordinates of a ray hitting a triangle.
 *
 * @param v0
 * The first vertex.
 * @param v1
 * The second vertex.
 * @param v2
 * The third vertex.
 * @param pos
 * The ray's origin.
 * @param dir
 * The ray's direction.
 * @param duv
 * The resulting coordinates of the projection in the space of the ray and
 * vector with t being the distance along the vector and u/v being
 * barycentric coordinates.
 */
int TriangleIntersector::intersectTriangleLine(const Point3d& v0,
        const Point3d& v1, const Point3d& v2, const Point3d& pos,
        const Vector3d& dir, Vector3d& duv) const {

    Vector3d edge0, edge1, pvec, tvec;

    edge0.subtract(v1, v0);
    edge1.subtract(v2, v0);

    /* begin calculating determinant - also used to calculate U parameter */
    pvec.cross(dir, edge1);

    /* if determinant is near zero, ray lies in plane of triangle */
    double det = edge0.dot(pvec);

    if (det > -epsilon && det < epsilon)
        return 0;

    double inv_det = 1.0 / det;

    /* calculate distance from vert0 to ray origin */
    tvec.subtract(pos, v0);

    /* calculate U parameter and test bounds */
    duv.y = tvec.dot(pvec) * inv_det;
    if (duv.y < 0.0 || duv.y > 1.0)
        return 0;

    /* prepare to test V parameter */
    pvec.cross(tvec, edge0);

    /* calculate V parameter and test bounds */
    duv.z = dir.dot(pvec) * inv_det;
    if (duv.z < 0.0 || duv.y + duv.z > 1.0)
        return 0;

    /* calculate t, ray intersects triangle */
    duv.x = edge1.dot(pvec) * inv_det;

    return 1;
}

/**
 * Finds the nearest distance between a point and a triangle.
 *
 * @param v0
 * The first vertex.
 * @param v1
 * The second vertex.
 * @param v2
 * The third vertex.
 * @param p
 * The point to measure from.
 * @param closest
 * The closest point to p on the triangle.
 * @param duv
 * The barycentric coordinates of the nearest point where u and v are the
 * weights for vertices 1 and 2 respectively.
 * @return The distance from point p to the nearest point on triangle v0, v1
 * and v2.
 */
double TriangleIntersector::nearestpoint(const Point3d& v0, const Point3d& v1,
        const Point3d& v2, const Point3d& p, Point3d& closest,
        Vector3d& duv) const {

    Vector3d pvec, edge0, edge1;
    pvec.subtract(v0, p);
    edge0.subtract(v1, v0);
    edge1.subtract(v2, v0);

    double fA00 = edge0.normSquared();
    double fA01 = edge0.dot(edge1);
    double fA11 = edge1.normSquared();
    double fB0 = pvec.dot(edge0);
    double fB1 = pvec.dot(edge1);
    double fC = pvec.normSquared();
    double fDet = fabs(fA00 * fA11 - fA01 * fA01);
    double fS = fA01 * fB1 - fA11 * fB0;
    double fT = fA01 * fB0 - fA00 * fB1;
    double fSqrDistance;

    if (fS + fT <= fDet) {
        if (fS < 0.0) {
            if (fT < 0.0) // region 4
                    {
                if (fB0 < 0.0) {
                    fT = 0.0;
                    if (-fB0 >= fA00) {
                        fS = 1.0;
                        fSqrDistance = fA00 + (2.0) * fB0 + fC;
                    } else {
                        fS = -fB0 / fA00;
                        fSqrDistance = fB0 * fS + fC;
                    }
                } else {
                    fS = 0.0;
                    if (fB1 >= 0.0) {
                        fT = 0.0;
                        fSqrDistance = fC;
                    } else if (-fB1 >= fA11) {
                        fT = 1.0;
                        fSqrDistance = fA11 + (2.0) * fB1 + fC;
                    } else {
                        fT = -fB1 / fA11;
                        fSqrDistance = fB1 * fT + fC;
                    }
                }
            } else
            // region 3
            {
                fS = 0.0;
                if (fB1 >= 0.0) {
                    fT = 0.0;
                    fSqrDistance = fC;
                } else if (-fB1 >= fA11) {
                    fT = 1.0;
                    fSqrDistance = fA11 + (2.0) * fB1 + fC;
                } else {
                    fT = -fB1 / fA11;
                    fSqrDistance = fB1 * fT + fC;
                }
            }
        } else if (fT < 0.0) // region 5
                {
            fT = 0.0;
            if (fB0 >= 0.0) {
                fS = 0.0;
                fSqrDistance = fC;
            } else if (-fB0 >= fA00) {
                fS = 1.0;
                fSqrDistance = fA00 + (2.0) * fB0 + fC;
            } else {
                fS = -fB0 / fA00;
                fSqrDistance = fB0 * fS + fC;
            }
        } else
        // region 0
        {
            // minimum at interior point
            double fInvDet = (1.0) / fDet;
            fS *= fInvDet;
            fT *= fInvDet;
            fSqrDistance = fS * (fA00 * fS + fA01 * fT + (2.0) * fB0)
                    + fT * (fA01 * fS + fA11 * fT + (2.0) * fB1) + fC;
        }
    } else {
        double fTmp0, fTmp1, fNumer, fDenom;

        if (fS < 0.0) // region 2
                {
            fTmp0 = fA01 + fB0;
            fTmp1 = fA11 + fB1;
            if (fTmp1 > fTmp0) {
                fNumer = fTmp1 - fTmp0;
                fDenom = fA00 - 2.0f * fA01 + fA11;
                if (fNumer >= fDenom) {
                    fS = 1.0;
                    fT = 0.0;
                    fSqrDistance = fA00 + (2.0) * fB0 + fC;
                } else {
                    fS = fNumer / fDenom;
                    fT = 1.0 - fS;
                    fSqrDistance = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0)
                            + fT * (fA01 * fS + fA11 * fT + (2.0) * fB1) + fC;
                }
            } else {
                fS = 0.0;
                if (fTmp1 <= 0.0) {
                    fT = 1.0;
                    fSqrDistance = fA11 + (2.0) * fB1 + fC;
                } else if (fB1 >= 0.0) {
                    fT = 0.0;
                    fSqrDistance = fC;
                } else {
                    fT = -fB1 / fA11;
                    fSqrDistance = fB1 * fT + fC;
                }
            }
        } else if (fT < 0.0) // region 6
                {
            fTmp0 = fA01 + fB1;
            fTmp1 = fA00 + fB0;
            if (fTmp1 > fTmp0) {
                fNumer = fTmp1 - fTmp0;
                fDenom = fA00 - (2.0) * fA01 + fA11;
                if (fNumer >= fDenom) {
                    fT = 1.0;
                    fS = 0.0;
                    fSqrDistance = fA11 + (2.0) * fB1 + fC;
                } else {
                    fT = fNumer / fDenom;
                    fS = 1.0 - fT;
                    fSqrDistance = fS * (fA00 * fS + fA01 * fT + (2.0) * fB0)
                            + fT * (fA01 * fS + fA11 * fT + (2.0) * fB1) + fC;
                }
            } else {
                fT = 0.0;
                if (fTmp1 <= 0.0) {
                    fS = 1.0;
                    fSqrDistance = fA00 + (2.0) * fB0 + fC;
                } else if (fB0 >= 0.0) {
                    fS = 0.0;
                    fSqrDistance = fC;
                } else {
                    fS = -fB0 / fA00;
                    fSqrDistance = fB0 * fS + fC;
                }
            }
        } else
        // region 1
        {
            fNumer = fA11 + fB1 - fA01 - fB0;
            if (fNumer <= 0.0) {
                fS = 0.0;
                fT = 1.0;
                fSqrDistance = fA11 + (2.0) * fB1 + fC;
            } else {
                fDenom = fA00 - 2.0f * fA01 + fA11;
                if (fNumer >= fDenom) {
                    fS = 1.0;
                    fT = 0.0;
                    fSqrDistance = fA00 + (2.0) * fB0 + fC;
                } else {
                    fS = fNumer / fDenom;
                    fT = 1.0 - fS;
                    fSqrDistance = fS * (fA00 * fS + fA01 * fT + (2.0) * fB0)
                            + fT * (fA01 * fS + fA11 * fT + (2.0) * fB1) + fC;
                }
            }
        }
    }

    closest.scaledAdd(v0, fS, edge0);
    closest.scaledAdd(closest, fT, edge1);

    if (fSqrDistance < 0) {
        duv.x = 0;
    } else {
        duv.x = sqrt(fSqrDistance);
    }
    duv.y = fS;
    duv.z = fT;

    return duv.x;
}

int TriangleIntersector::intersectTrianglePlane(const Point3d& p0,
        const Point3d& p1, const Point3d& p2, const Plane& plane,
        std::vector<Point3d>& pnts) const {

    double d0 = plane.distance(p0);
    double d1 = plane.distance(p1);
    double d2 = plane.distance(p2);
    double ad0 = fabs(d0);
    double ad1 = fabs(d1);
    double ad2 = fabs(d2);
    int numPnts = 0;

    // from p0 to p1
    double t = d0 / (d0 - d1);
    if (t > 0 && t < 1 && ad0 > epsilon && ad1 > epsilon) {
        pnts.push_back(Point3d());
        pnts[numPnts++].interpolate(p0, t, p1);
    }

    // from p1 to p2
    t = d1 / (d1 - d2);
    if (t > 0 && t < 1 && ad1 > epsilon && ad2 > epsilon) {
        pnts.push_back(Point3d());
        pnts[numPnts++].interpolate(p1, t, p2);
    }

    // from p2 to p0
    t = d2 / (d2 - d0);
    if (t > 0 && t < 1 && ad2 > epsilon && ad0 > epsilon) {
        pnts.push_back(Point3d());
        pnts[numPnts++].interpolate(p2, t, p0);
    }

    // end points
    if (ad0 <= epsilon) {
        pnts.push_back(Point3d());
        pnts[numPnts++] = p0;
    }
    if (ad1 <= epsilon) {
        pnts.push_back(Point3d());
        pnts[numPnts++] = p1;
    }
    if (ad2 <= epsilon) {
        pnts.push_back(Point3d());
        pnts[numPnts++] = p2;
    }

    return numPnts;
}

TriangleLineIntersection::TriangleLineIntersection(const SharedPolygon& face,
        const std::shared_ptr<Line>& line, std::vector<Point3d>&& pnts) :
        face(face), line(line), points(std::move(pnts)) {
}

TrianglePlaneIntersection::TrianglePlaneIntersection(const SharedPolygon& face,
        const std::shared_ptr<Plane>& plane, std::vector<Point3d>&& pnts) :
        face(face), plane(plane), points(std::move(pnts)) {
}

TriangleTriangleIntersection::TriangleTriangleIntersection(
        const SharedPolygon& tri0, const SharedPolygon& tri1,
        std::vector<Point3d>&& pnts) :
        triangle0(tri0), triangle1(tri1), points(std::move(pnts)) {
}

BVIntersector::BVIntersector() :
        epsilon(0), myTriIntersector(0) {
}

BVIntersector::BVIntersector(double eps) :
        epsilon(eps), myTriIntersector(eps) {
}

void BVIntersector::setEpsilon(double eps) {
    this->epsilon = eps;
    myTriIntersector.setEpsilon(eps);
}

double BVIntersector::getEpsilon() {
    return epsilon;
}

}
}
