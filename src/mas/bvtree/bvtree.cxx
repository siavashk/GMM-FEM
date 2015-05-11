#include "mas/bvtree/bvtree.h"
#include "mas/core/math.h"
#include <math.h>
#include <algorithm>

namespace mas {
namespace bvtree {

// Bounding Volume
BoundingVolume::BoundingVolume() :
        margin(0) {
}

BoundingVolume::BoundingVolume(double margin) :
        margin(margin) {
}

double BoundingVolume::getMargin() const {
    return margin;
}

BoundingSphere::BoundingSphere(double margin) :
        margin(margin), r(margin), c(0, 0, 0) {
}

BoundingSphere::BoundingSphere(const BoundingSphere& copyMe) :
        margin(copyMe.margin), r(copyMe.r), c(copyMe.c) {
}

BoundingSphere::BoundingSphere(const Point3d& c, double r, double margin) :
        margin(margin), r(r), c(c) {
}

void BoundingSphere::set(const Point3d& c, double r) {
    this->r = r;
    this->c = c;
}

void BoundingSphere::setRadius(double r) {
    this->r = r;
}

double BoundingSphere::getRadius() const {
    return r;
}

void BoundingSphere::setCentre(const Point3d& c) {
    this->c = c;
}

void BoundingSphere::getCentre(Point3d& c) const {
    c = this->c;
}

void BoundingSphere::setMargin(double m) {
    r = r - margin + m;	// adjust margin
    margin = m;
}

double BoundingSphere::getMargin() {
   return margin;
}

bool BoundingSphere::intersectsPoint(const Point3d& p) const {
    if (p.distanceSquared(c) <= r * r) {	// closed ball
        return true;
    }
    return false;
}

bool BoundingSphere::intersectsSphere(const Point3d& c, double r) const {

    double r2 = (r + this->r);
    r2 = r2 * r2;

    if (this->c.distanceSquared(c) <= r2) {
        return true;
    }
    return false;
}

bool BoundingSphere::intersectsLine(const Point3d& p, const Vector3d& v) const {

    // orthogonal projection onto line
    Point3d nearest = this->c;
    nearest.subtract(p);
    double s = nearest.dot(v) / v.dot(v);
    nearest.scaledAdd(p, s, v);

    if (nearest.distanceSquared(this->c) <= this->r * this->r) {
        return true;
    }
    return false;
}

bool BoundingSphere::intersectsRay(const Point3d& p, const Vector3d& v) const {

    Point3d nearest = this->c;
    nearest.subtract(p);
    double s = nearest.dot(v);
    if (s < 0) {
        return false;
    }
    s = s / v.dot(v);
    nearest.scaledAdd(p, s, v);

    if (nearest.distanceSquared(this->c) <= this->r * this->r) {
        return true;
    }
    return false;
}

bool BoundingSphere::intersectsPlane(const Plane& p) const {
    double d = p.distanceSigned(c);
    if (fabs(d) <= r) {
        return true;
    }
    return false;
}

double BoundingSphere::distanceToPoint(const Point3d& pnt,
        Point3d& nearest) const {

    double d = c.distance(pnt) - r;
    // inside
    if (d <= 0) {
        nearest = pnt;
        return 0;
    }

    // closest point
    nearest.interpolate(pnt, d / (d + r), c);
    return d;
}

double BoundingSphere::distanceToPoint(const Point3d& pnt, const Vector3d& dir,
        Point3d& nearest) const {

    Vector3d pc = Vector3d(pnt);
    pc.subtract(c);

    // inside
    if (pc.norm() <= r) {
        nearest = pnt;
        return 0;
    }

    // intersect ray
    double dirpc = dir.dot(pc);
    double dirsq = dir.dot(dir);
    double d = dirpc * dirpc - dirsq * (pc.dot(pc) - r * r);

    // doesn't intersect
    if (d < 0) {
        // negative direction
        if (dir.dot(pc) > 0) {
            nearest.x = -math::signum(dir.x) * math::DOUBLE_INFINITY;
            nearest.y = -math::signum(dir.y) * math::DOUBLE_INFINITY;
            nearest.z = -math::signum(dir.z) * math::DOUBLE_INFINITY;
        } else {
            // positive direction
            nearest.x = math::signum(dir.x) * math::DOUBLE_INFINITY;
            nearest.y = math::signum(dir.y) * math::DOUBLE_INFINITY;
            nearest.z = math::signum(dir.z) * math::DOUBLE_INFINITY;
        }
        return math::DOUBLE_INFINITY;
    }

    // both distances
    d = sqrt(d) / dirsq;
    double d1 = dirpc / dirsq;
    double d2 = d1 + d;
    d1 = d1 - d;

    if (fabs(d1) <= fabs(d2)) {
        d = fabs(d1);
    } else {
        d = fabs(d2);
    }
    nearest = pnt;
    nearest.scaledAdd(d, dir);
    return d;
}

BoundingSphere BoundingSphere::getBoundingSphere() const {
    return *this;
}

double BoundingSphere::getBoundingSphere(Point3d& centre) const {
	centre = c;
	return r;
}

bool BoundingSphere::updatePoint(const Point3d& p) {
    double rnew = p.distance(c) + margin;
    if (rnew > r) {
        r = rnew;
        return true;
    }
    return false;
}

bool BoundingSphere::updateSphere(const Point3d& c, double r) {
    double rnew = this->c.distance(c) + r + margin;
    if (rnew > r) {
        r = rnew;
        return true;
    }
    return false;
}

int BoundingBox::boxCorners[8][3] = { { -1, -1, -1 }, { 1, -1, -1 },
        { 1, 1, -1 }, { -1, 1, -1 }, { -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, {
                -1, 1, 1 } };

BoundingBox::BoundingBox(double margin) :
        margin(margin), c(0, 0, 0), halfWidths(margin, margin, margin) {
}

BoundingBox::BoundingBox(const BoundingBox& copyMe) :
        margin(copyMe.margin), c(copyMe.c), halfWidths(
                copyMe.halfWidths) {
}

BoundingBox::BoundingBox(const Point3d& c, const Vector3d& hw, double margin) :
        margin(margin), c(c), halfWidths(hw) {
    halfWidths.add(margin,margin,margin);
}

void BoundingBox::set(const Point3d& c, const Vector3d& hw) {
    this->c = c;
    halfWidths = hw;
    halfWidths.add(margin,margin,margin);
}

void BoundingBox::setHalfWidths(const Vector3d& hw) {
    halfWidths = hw;
    halfWidths.add(margin,margin,margin);
}

void BoundingBox::getHalfWidths(Vector3d& hw) const {
    hw.set(halfWidths.x-margin, halfWidths.y-margin, halfWidths.z-margin);
}

void BoundingBox::setCentre(const Point3d& c) {
    this->c = c;
}

void BoundingBox::getCentre(Point3d& c) const {
    c = this->c;
}

void BoundingBox::setMargin(double m) {
    halfWidths.add(m - margin, m - margin, m - margin);
    margin = m;
}

double BoundingBox::getMargin() {
   return margin;
}

void BoundingBox::getCorner(int idx, Point3d& pnt) const {
    pnt.set(boxCorners[idx][0] * halfWidths.x,
            boxCorners[idx][1] * halfWidths.y,
            boxCorners[idx][2] * halfWidths.z);
    getWorldCoords(pnt, pnt);
}

bool BoundingBox::intersectsPoint(const Point3d& p) const {

    Point3d pl;
    getLocalCoords(p, pl);

    bool status = (pl.x >= -halfWidths.x) && (pl.x <= halfWidths.x)
            && (pl.y >= -halfWidths.y) && (pl.y <= halfWidths.y)
            && (pl.z >= -halfWidths.z) && (pl.z <= halfWidths.z);

    return status;
}

bool BoundingBox::intersectsSphere(const Point3d& c, double r) const {

    Point3d pl;
    getLocalCoords(c, pl);
    bool status = (pl.x + r >= -halfWidths.x) && (pl.x - r <= halfWidths.x)
            && (pl.y + r >= -halfWidths.y) && (pl.y - r <= halfWidths.y)
            && (pl.z + r >= -halfWidths.z) && (pl.z - r <= halfWidths.z);

    return status;
}

bool BoundingBox::intersectsLine(const Point3d& p, const Vector3d& v) const {

    if (intersectsPoint(p)) {
        return true;
    }

    Point3d pl;
    Vector3d vl;
    getLocalCoords(p, pl);
    getLocalCoords(v, vl);

    double minl = -math::DOUBLE_INFINITY;
    double maxl = math::DOUBLE_INFINITY;

    // intersect with front/back planes
    for (int i = 0; i < 3; i++) {
        double hwb = halfWidths[i];
        double pb = pl[i];
        double vb = vl[i];

        double minb = -hwb;
        double maxb = hwb;

        if (vb == 0) {
            // parallel to and outside box
            if (minb - pb > 0 || maxb - pb < 0) {
                return false;
            }
        } else {

            // intersect plane
            double d = 1.0 / vb;
            double dist0 = (minb - pb) * d;
            double dist1 = (maxb - pb) * d;
            if (vb < 0) {
                double tmp = dist0;
                dist0 = dist1;
                dist1 = tmp;
            }

            if (minl < dist0) {
                minl = dist0;
            }
            if (maxl > dist1) {
                maxl = dist1;
            }

            if (minl > maxl) {
                return false;
            }
        }

    }
    return true;
}

bool BoundingBox::intersectsRay(const Point3d& p, const Vector3d& v) const {

    //LOG("p: (%lf %lf %lf),  v: (%lf %lf %lf),  c: (%lf %lf %lf)",
    //		p.x, p.y, p.z, v.x, v.y, v.z, c.x, c.y, c.z);

    if (intersectsPoint(p)) {
        return true;
    }

    Point3d pl;
    Vector3d vl;
    getLocalCoords(p, pl);
    getLocalCoords(v, vl);

    // 0 for ray
    double minl = 0; //-std::numeric_limits<double>::infinity();
    double maxl = math::DOUBLE_INFINITY;

    // intersect with front/back planes
    for (int i = 0; i < 3; i++) {
        double hwb = halfWidths[i];
        double pb = pl[i];
        double vb = vl[i];

        double minb = -hwb;
        double maxb = hwb;

        if (vb == 0) {
            // parallel to and outside box
            if (minb - pb > 0 || maxb - pb < 0) {
                return false;
            }
        } else {

            // intersect plane
            double d = 1.0 / vb;
            double dist0 = (minb - pb) * d;
            double dist1 = (maxb - pb) * d;

            if (vb >= 0) {
                if (dist0 > minl) {
                    minl = dist0;
                }
                if (dist1 < maxl) {
                    maxl = dist1;
                }
            } else {
                if (dist1 > minl) {
                    minl = dist1;
                }
                if (dist0 < maxl) {
                    maxl = dist0;
                }
            }

            if (minl > maxl) {
                return false;
            }
        }
    }

    return true;
}

bool BoundingBox::intersectsPlane(const Plane& p) const {

    // grab normal and point on plane
    // to be transformed, so non-constant
    Vector3d nl = p.normal;
    // compute point on plane
    Point3d pl = p.normal;
    pl.scale(-p.d);

    // transform plane to local coordinates
    getLocalCoords(nl, nl);
    getLocalCoords(pl, pl);
    double d = -nl.dot(pl);

    // check plane has at least one corner on each side
    bool up = false;
    bool down = false;
    double x[] = { -halfWidths.x, halfWidths.x };
    double y[] = { -halfWidths.y, halfWidths.y };
    double z[] = { -halfWidths.z, halfWidths.z };
    double b = 0;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                b = nl.x * x[i] + nl.y * y[j] + nl.z * z[k] + d;
                up = up | (b >= 0);
                down = down | (b <= 0);
                if (up && down) {
                    return true;
                }
            }
        }
    }

    return false;
}

BoundingSphere BoundingBox::getBoundingSphere() const {
    return BoundingSphere(c, halfWidths.norm(), margin);
}

double BoundingBox::getBoundingSphere(Point3d& centre) const {
	centre = c;
	return halfWidths.norm()+margin;
}

bool BoundingBox::updatePoint(const Point3d& p) {
    return updateSphere(p, 0);
}

bool BoundingBox::updateSphere(const Point3d& c, double r) {

    Point3d cl;
    getLocalCoords(c, cl);
    Point3d max = halfWidths;
    Point3d min = halfWidths;
    min.scale(-1);

    bool modified = false;

    r = r + margin;
    if (cl.x - r < min.x) {
        min.x = cl.x - r;
        modified = true;
    }
    if (cl.y - r < min.y) {
        min.y = cl.y - r;
        modified = true;
    }
    if (cl.z - r < min.z) {
        min.z = cl.z - r;
        modified = true;
    }
    if (cl.x + r > max.x) {
        max.x = cl.x + r;
        modified = true;
    }
    if (cl.y + r > max.y) {
        max.y = cl.y + r;
        modified = true;
    }
    if (cl.z + r > max.z) {
        max.z = cl.z + r;
        modified = true;
    }

    if (modified) {
        cl = min;
        cl.add(max);
        cl.scale(0.5);
        getWorldCoords(cl, this->c);

        halfWidths = max;
        halfWidths.subtract(min);
        halfWidths.scale(0.5);
        return true;
    }
    return false;
}

double BoundingBox::distanceToPoint(const Point3d& pnt,
        Point3d& nearest) const {

    Point3d lpnt;
    getLocalCoords(pnt, lpnt);

    // find nearest face/edge or corner
    nearest = lpnt;

    if (nearest.x > halfWidths.x) {
        nearest.x = halfWidths.x;
    } else if (nearest.x < -halfWidths.x) {
        nearest.x = -halfWidths.x;
    }

    if (nearest.y > halfWidths.y) {
        nearest.y = halfWidths.y;
    } else if (nearest.y < -halfWidths.y) {
        nearest.y = -halfWidths.y;
    }

    if (nearest.z > halfWidths.z) {
        nearest.z = halfWidths.z;
    } else if (nearest.z < -halfWidths.z) {
        nearest.z = -halfWidths.z;
    }

    double dist = nearest.distance(lpnt);

    getWorldCoords(nearest, nearest);

    return dist;
}

double BoundingBox::distanceToPoint(const Point3d& pnt, const Vector3d& dir,
        Point3d& nearest) const {

    //search along dir
    Point3d lpnt;
    Vector3d ldir;
    getLocalCoords(pnt, lpnt);
    getLocalCoords(dir, ldir);

    // first check if inside
    if (lpnt.x >= -halfWidths.x && lpnt.x <= halfWidths.x
            && lpnt.y >= -halfWidths.y && lpnt.y <= halfWidths.y
            && lpnt.z >= -halfWidths.z && lpnt.z <= halfWidths.z) {
        nearest = pnt;
        return 0;
    }

    double dmin = math::DOUBLE_INFINITY;
    Point3d p;

    // intersect with YZ planes
    if (ldir.x == 0) {
        if (lpnt.x > halfWidths.x || lpnt.x < -halfWidths.x) {
            return math::DOUBLE_INFINITY;
        }
    } else {
        // intersect along line
        double s = (-halfWidths.x - lpnt.x) / ldir.x;
        p.scaledAdd(lpnt, s, ldir);
        s = p.distance(lpnt);

        // check if p is in box and if point is closer
        if (p.x >= -halfWidths.x && p.x <= halfWidths.x && p.y >= -halfWidths.y
                && p.y <= halfWidths.y && p.z >= -halfWidths.z
                && p.z <= halfWidths.z && (s < dmin)) {
            nearest = p;
            dmin = s;
        }

        s = (halfWidths.x - lpnt.x) / ldir.x;
        p.scaledAdd(lpnt, s, ldir);
        s = p.distance(lpnt);

        // check if p is in box
        if (p.x >= -halfWidths.x && p.x <= halfWidths.x && p.y >= -halfWidths.y
                && p.y <= halfWidths.y && p.z >= -halfWidths.z
                && p.z <= halfWidths.z && (s < dmin)) {
            nearest = p;
            dmin = s;
        }
    }

    // intersect with XZ planes
    if (ldir.y == 0) {
        if (lpnt.y > halfWidths.y || lpnt.y < -halfWidths.y) {
            return math::DOUBLE_INFINITY;
        }
    } else {
        // intersect along line
        double s = (-halfWidths.y - lpnt.y) / ldir.y;
        p.scaledAdd(lpnt, s, ldir);
        s = p.distance(lpnt);

        // check if p is in box and if point is closer
        if (p.x >= -halfWidths.x && p.x <= halfWidths.x && p.y >= -halfWidths.y
                && p.y <= halfWidths.y && p.z >= -halfWidths.z
                && p.z <= halfWidths.z && (s < dmin)) {
            nearest = p;
            dmin = s;
        }

        s = (halfWidths.y - lpnt.y) / ldir.y;
        p.scaledAdd(lpnt, s, ldir);
        s = p.distance(lpnt);

        // check if p is in box
        if (p.x >= -halfWidths.x && p.x <= halfWidths.x && p.y >= -halfWidths.y
                && p.y <= halfWidths.y && p.z >= -halfWidths.z
                && p.z <= halfWidths.z && (s < dmin)) {
            nearest = p;
            dmin = s;
        }
    }

    // intersect with XY planes
    if (ldir.z == 0) {
        if (lpnt.z > halfWidths.z || lpnt.z < -halfWidths.z) {
            return math::DOUBLE_INFINITY;
        }
    } else {
        // intersect along line
        double s = (-halfWidths.z - lpnt.z) / ldir.z;
        p.scaledAdd(lpnt, s, ldir);
        s = p.distance(lpnt);

        // check if p is in box and if point is closer
        if (p.x >= -halfWidths.x && p.x <= halfWidths.x && p.y >= -halfWidths.y
                && p.y <= halfWidths.y && p.z >= -halfWidths.z
                && p.z <= halfWidths.z && (s < dmin)) {
            nearest = p;
            dmin = s;
        }

        s = (halfWidths.z - lpnt.z) / ldir.z;
        p.scaledAdd(lpnt, s, ldir);
        s = p.distance(lpnt);

        // check if p is in box
        if (p.x >= -halfWidths.x && p.x <= halfWidths.x && p.y >= -halfWidths.y
                && p.y <= halfWidths.y && p.z >= -halfWidths.z
                && p.z <= halfWidths.z && (s < dmin)) {
            nearest = p;
            dmin = s;
        }
    }

    getWorldCoords(nearest, nearest);
    return dmin;
}

AABB::AABB(double margin) :
        BoundingBox(margin) {
}

AABB::AABB(const AABB& copyMe) :
        BoundingBox(copyMe) {
}

AABB::AABB(const Point3d& c, const Vector3d& hw, double margin) :
        BoundingBox(c, hw, margin) {
}

void AABB::getLocalCoords(const Point3d& p, Point3d& out) const {
    out.subtract(p, c);
}

void AABB::getLocalCoords(const Vector3d& v, Vector3d& out) const {
    out = v;
}

void AABB::getWorldCoords(const Point3d& p, Point3d& out) const {
    out = c;	// first in case out == c, otherwise overwrites c
    out.add(p);
}

void AABB::getWorldCoords(const Vector3d& v, Vector3d& out) const {
    out = v;
}

template<>
bool AABB::intersects(const BoundingSphere& bv) const {
    return intersectsSphere(bv.c, bv.r);
}

// AABB specialization
template<>
bool AABB::intersects(const AABB& bv) const {

    // both axis-aligned, check if off to any side
    if (bv.c.x - bv.halfWidths.x > c.x + halfWidths.x) {
        return false;
    } else if (bv.c.x + bv.halfWidths.x < c.x - halfWidths.x) {
        return false;
    } else if (bv.c.y - bv.halfWidths.y > c.y + halfWidths.y) {
        return false;
    } else if (bv.c.y + bv.halfWidths.y < c.y - halfWidths.y) {
        return false;
    } else if (bv.c.z - bv.halfWidths.z > c.z + halfWidths.z) {
        return false;
    } else if (bv.c.z + bv.halfWidths.z < c.z - halfWidths.z) {
        return false;
    }

    return true;
}

// OBB specialization
template<>
bool AABB::intersects(const OBB& bv) const {
   return bv.intersects(*this);
}

// Game Physics, David Eberly, Pg. 445
bool OBB::boxesIntersect(const Vector3d& hw1, const Vector3d& hw2,
        const RotationMatrix3d& R1, const RotationMatrix3d& R2,
        const Vector3d& pd, const Vector3d& px) {

    double t;
    double cutoff = 1 - 1e-10;

    // mij and pi give the transformation from the argument obb to this obb.

    // A1 x A2 = A0
    double p_x = R1.m[IDX3D_00] * pd.x + R1.m[IDX3D_10] * pd.y
            + R1.m[IDX3D_20] * pd.z + px.x;
    if ((t = p_x) < 0)
        t = -t;
    double m00 = R1.m[IDX3D_00] * R2.m[IDX3D_00]
            + R1.m[IDX3D_10] * R2.m[IDX3D_10] + R1.m[IDX3D_20] * R2.m[IDX3D_20];
    double m01 = R1.m[IDX3D_00] * R2.m[IDX3D_01]
            + R1.m[IDX3D_10] * R2.m[IDX3D_11] + R1.m[IDX3D_20] * R2.m[IDX3D_21];
    double m02 = R1.m[IDX3D_00] * R2.m[IDX3D_02]
            + R1.m[IDX3D_10] * R2.m[IDX3D_12] + R1.m[IDX3D_20] * R2.m[IDX3D_22];
    double abs00 = (m00 >= 0 ? m00 : -m00);
    double abs01 = (m01 >= 0 ? m01 : -m01);
    double abs02 = (m02 >= 0 ? m02 : -m02);
    if (t > (hw1.x + hw2.x * abs00 + hw2.y * abs01 + hw2.z * abs02))
        return false;

    // B1 x B2 = B0
    double m10 = R1.m[IDX3D_01] * R2.m[IDX3D_00]
            + R1.m[IDX3D_11] * R2.m[IDX3D_10] + R1.m[IDX3D_21] * R2.m[IDX3D_20];
    double m20 = R1.m[IDX3D_02] * R2.m[IDX3D_00]
            + R1.m[IDX3D_12] * R2.m[IDX3D_10] + R1.m[IDX3D_22] * R2.m[IDX3D_20];
    double p_y = R1.m[IDX3D_01] * pd.x + R1.m[IDX3D_11] * pd.y
            + R1.m[IDX3D_21] * pd.z + px.y;
    double p_z = R1.m[IDX3D_02] * pd.x + R1.m[IDX3D_12] * pd.y
            + R1.m[IDX3D_22] * pd.z + px.z;
    if ((t = p_x * m00 + p_y * m10 + p_z * m20) < 0)
        t = -t;
    double abs10 = (m10 >= 0 ? m10 : -m10);
    double abs20 = (m20 >= 0 ? m20 : -m20);
    if (t > (hw2.x + hw1.x * abs00 + hw1.y * abs10 + hw1.z * abs20))
        return false;

    // A2 x A0 = A1
    if ((t = p_y) < 0)
        t = -t;
    double m11 = R1.m[IDX3D_01] * R2.m[IDX3D_01]
            + R1.m[IDX3D_11] * R2.m[IDX3D_11] + R1.m[IDX3D_21] * R2.m[IDX3D_21];
    double m12 = R1.m[IDX3D_01] * R2.m[IDX3D_02]
            + R1.m[IDX3D_11] * R2.m[IDX3D_12] + R1.m[IDX3D_21] * R2.m[IDX3D_22];
    double abs11 = (m11 >= 0 ? m11 : -m11);
    double abs12 = (m12 >= 0 ? m12 : -m12);
    if (t > (hw1.y + hw2.x * abs10 + hw2.y * abs11 + hw2.z * abs12))
        return false;

    // A0 x A1 = A2
    if ((t = p_z) < 0)
        t = -t;
    double m21 = R1.m[IDX3D_02] * R2.m[IDX3D_01]
            + R1.m[IDX3D_12] * R2.m[IDX3D_11] + R1.m[IDX3D_22] * R2.m[IDX3D_21];
    double m22 = R1.m[IDX3D_02] * R2.m[IDX3D_02]
            + R1.m[IDX3D_12] * R2.m[IDX3D_12] + R1.m[IDX3D_22] * R2.m[IDX3D_22];
    double abs21 = (m21 >= 0 ? m21 : -m21);
    double abs22 = (m22 >= 0 ? m22 : -m22);
    if (t > (hw1.z + hw2.x * abs20 + hw2.y * abs21 + hw2.z * abs22))
        return false;

    // B2 x B0 = B1
    if ((t = p_x * m01 + p_y * m11 + p_z * m21) < 0)
        t = -t;
    if (t > (hw2.y + hw1.x * abs01 + hw1.y * abs11 + hw1.z * abs21))
        return false;

    // B0 x B1 = B2
    if ((t = p_x * m02 + p_y * m12 + p_z * m22) < 0)
        t = -t;
    if (t > (hw2.z + hw1.x * abs02 + hw1.y * abs12 + hw1.z * abs22))
        return false;

    // parallel, so only needed to check face normal directions
    if (abs00 > cutoff || abs01 > cutoff || abs02 > cutoff || abs10 > cutoff
            || abs11 > cutoff || abs12 > cutoff || abs20 > cutoff
            || abs21 > cutoff || abs22 > cutoff) {
        return true;
    }

    // A0 x B0
    if ((t = p_z * m10 - p_y * m20) < 0)
        t = -t;
    if (t > (hw1.y * abs20 + hw1.z * abs10 + hw2.y * abs02 + hw2.z * abs01))
        return false;

    // A0 x B1
    if ((t = p_z * m11 - p_y * m21) < 0)
        t = -t;
    if (t > (hw1.y * abs21 + hw1.z * abs11 + hw2.x * abs02 + hw2.z * abs00))
        return false;

    // A0 x B2
    if ((t = p_z * m12 - p_y * m22) < 0)
        t = -t;
    if (t > (hw1.y * abs22 + hw1.z * abs12 + hw2.x * abs01 + hw2.y * abs00))
        return false;

    // A1 x B0
    if ((t = p_x * m20 - p_z * m00) < 0)
        t = -t;
    if (t > (hw1.x * abs20 + hw1.z * abs00 + hw2.y * abs12 + hw2.z * abs11))
        return false;

    // A1 x B1
    if ((t = p_x * m21 - p_z * m01) < 0)
        t = -t;
    if (t > (hw1.x * abs21 + hw1.z * abs01 + hw2.x * abs12 + hw2.z * abs10))
        return false;

    // A1 x B2
    if ((t = p_x * m22 - p_z * m02) < 0)
        t = -t;
    if (t > (hw1.x * abs22 + hw1.z * abs02 + hw2.x * abs11 + hw2.y * abs10))
        return false;

    // A2 x B0
    if ((t = p_y * m00 - p_x * m10) < 0)
        t = -t;
    if (t > (hw1.x * abs10 + hw1.y * abs00 + hw2.y * abs22 + hw2.z * abs21))
        return false;

    // A2 x B1
    if ((t = p_y * m01 - p_x * m11) < 0)
        t = -t;
    if (t > (hw1.x * abs11 + hw1.y * abs01 + hw2.x * abs22 + hw2.z * abs20))
        return false;

    // A2 x B2
    if ((t = p_y * m02 - p_x * m12) < 0)
        t = -t;
    if (t > (hw1.x * abs12 + hw1.y * abs02 + hw2.x * abs21 + hw2.y * abs20))
        return false;

    return true;
}

bool OBB::boxesIntersect(const Vector3d& hw1, const Vector3d& hw2,
        const RotationMatrix3d& R21, const Vector3d& t21) {

    // start with simple sphere test
    if (t21.norm() > hw1.norm() + hw2.norm()) {
        return false;
    }

    double cutoff = 1.0 - 1e-10;

    // OBB intersection:
    // OBBTree: A Hierarchichal Structure for Rapid Interference
    // Detection", Gottschalk Lin&  Manocha

    double t;

    // A1 x A2 = A0
    t = t21.x;
    if (t < 0) {
        t = -t;
    }

    double m00 = R21.m[IDX3D_00];
    double m01 = R21.m[IDX3D_01];
    double m02 = R21.m[IDX3D_02];

    double abs00 = (m00 >= 0 ? m00 : -m00);
    double abs01 = (m01 >= 0 ? m01 : -m01);
    double abs02 = (m02 >= 0 ? m02 : -m02);
    if (t > (hw1.x + hw2.x * abs00 + hw2.y * abs01 + hw2.z * abs02)) {
        return false;
    }

    // B1 x B2 = B0
    double m10 = R21.m[IDX3D_10];
    double m20 = R21.m[IDX3D_20];

    if ((t = t21.x * m00 + t21.y * m10 + t21.z * m20) < 0) {
        t = -t;
    }
    double abs10 = (m10 >= 0 ? m10 : -m10);
    double abs20 = (m20 >= 0 ? m20 : -m20);
    if (t > (hw2.x + hw1.x * abs00 + hw1.y * abs10 + hw1.z * abs20)) {
        return false;
    }

    // A2 x A0 = A1
    if ((t = t21.y) < 0) {
        t = -t;
    }

    double m11 = R21.m[IDX3D_11];
    double m12 = R21.m[IDX3D_12];
    double abs11 = (m11 >= 0 ? m11 : -m11);
    double abs12 = (m12 >= 0 ? m12 : -m12);
    if (t > (hw1.y + hw2.x * abs10 + hw2.y * abs11 + hw2.z * abs12)) {
        return false;
    }

    // A0 x A1 = A2
    if ((t = t21.z) < 0) {
        t = -t;
    }
    double m21 = R21.m[IDX3D_21];
    double m22 = R21.m[IDX3D_22];
    double abs21 = (m21 >= 0 ? m21 : -m21);
    double abs22 = (m22 >= 0 ? m22 : -m22);
    if (t > (hw1.z + hw2.x * abs20 + hw2.y * abs21 + hw2.z * abs22)) {
        return false;
    }

    // B2 x B0 = B1
    if ((t = t21.x * m01 + t21.y * m11 + t21.z * m21) < 0) {
        t = -t;
    }
    if (t > (hw2.y + hw1.x * abs01 + hw1.y * abs11 + hw1.z * abs21)) {
        return false;
    }

    // B0 x B1 = B2
    if ((t = t21.x * m02 + t21.y * m12 + t21.z * m22) < 0) {
        t = -t;
    }
    if (t > (hw2.z + hw1.x * abs02 + hw1.y * abs12 + hw1.z * abs22)) {
        return false;
    }

    // parallel, so only needed to check face normal directions
    if (abs00 > cutoff || abs01 > cutoff || abs02 > cutoff || abs10 > cutoff
            || abs11 > cutoff || abs12 > cutoff || abs20 > cutoff
            || abs21 > cutoff || abs22 > cutoff) {
        return true;
    }

    // A0 x B0
    if ((t = t21.z * m10 - t21.y * m20) < 0) {
        t = -t;
    }
    if (t > (hw1.y * abs20 + hw1.z * abs10 + hw2.y * abs02 + hw2.z * abs01)) {
        return false;
    }

    // A0 x B1
    if ((t = t21.z * m11 - t21.y * m21) < 0) {
        t = -t;
    }
    if (t > (hw1.y * abs21 + hw1.z * abs11 + hw2.x * abs02 + hw2.z * abs00)) {
        return false;
    }

    // A0 x B2
    if ((t = t21.z * m12 - t21.y * m22) < 0) {
        t = -t;
    }
    if (t > (hw1.y * abs22 + hw1.z * abs12 + hw2.x * abs01 + hw2.y * abs00)) {
        return false;
    }

    // A1 x B0
    if ((t = t21.x * m20 - t21.z * m00) < 0) {
        t = -t;
    }
    if (t > (hw1.x * abs20 + hw1.z * abs00 + hw2.y * abs12 + hw2.z * abs11)) {
        return false;
    }

    // A1 x B1
    if ((t = t21.x * m21 - t21.z * m01) < 0) {
        t = -t;
    }
    if (t > (hw1.x * abs21 + hw1.z * abs01 + hw2.x * abs12 + hw2.z * abs10)) {
        return false;
    }

    // A1 x B2
    if ((t = t21.x * m22 - t21.z * m02) < 0) {
        t = -t;
    }
    if (t > (hw1.x * abs22 + hw1.z * abs02 + hw2.x * abs11 + hw2.y * abs10)) {
        return false;
    }

    // A2 x B0
    if ((t = t21.y * m00 - t21.x * m10) < 0) {
        t = -t;
    }
    if (t > (hw1.x * abs10 + hw1.y * abs00 + hw2.y * abs22 + hw2.z * abs21)) {
        return false;
    }

    // A2 x B1
    if ((t = t21.y * m01 - t21.x * m11) < 0) {
        t = -t;
    }
    if (t > (hw1.x * abs11 + hw1.y * abs01 + hw2.x * abs22 + hw2.z * abs20)) {
        return false;
    }

    // A2 x B2
    if ((t = t21.y * m02 - t21.x * m12) < 0) {
        t = -t;
    }

    if (t > (hw1.x * abs12 + hw1.y * abs02 + hw2.x * abs21 + hw2.y * abs20)) {
        return false;
    }

    return true;

}

OBB::OBB(double margin) :
        BoundingBox(margin) {
}

OBB::OBB(const OBB& copyMe) :
        BoundingBox(copyMe), R(copyMe.R) {
}

OBB::OBB(const AABB& copyMe) :
        BoundingBox(copyMe), R(RotationMatrix3d::IDENTITY) {
}

OBB::OBB(const Point3d& c, const RotationMatrix3d& R, const Vector3d& hw) :
        BoundingBox(c, hw), R(R) {
}

OBB::OBB(const RigidTransform3d& trans, const Vector3d& hw) :
        BoundingBox(trans.t, hw), R(trans.R) {
}

void OBB::set(const Point3d& c, const RotationMatrix3d& R, const Vector3d& hw) {
    this->c = c;
    this->halfWidths = hw;
    this->R = R;
}

void OBB::set(const RigidTransform3d& trans, const Vector3d& hw) {
    c = trans.t;
    halfWidths = hw;
    R = trans.R;
}

void OBB::setRotation(const RotationMatrix3d& R) {
    this->R = R;
}

void OBB::getRotation(RotationMatrix3d& R) {
    R = this->R;
}

void OBB::getLocalCoords(const Point3d& p, Point3d& out) const {
    R.subtractMultiplyLeft(p, c, out);
}

void OBB::getLocalCoords(const Vector3d& v, Vector3d& out) const {
    R.multiplyLeft(v, out);
}

void OBB::getWorldCoords(const Point3d& p, Point3d& out) const {
    R.multiplyAdd(p, c, out);
}

void OBB::getWorldCoords(const Vector3d& v, Vector3d& out) const {
    R.multiply(v, out);
}

// template specializations
template<>
bool OBB::intersects(const BoundingSphere& bv) const {
    return intersectsSphere(bv.c, bv.r);
}

template<>
bool OBB::intersects(const AABB& bv) const {
    Vector3d t21 = c;
    t21.subtract(bv.c);
    return boxesIntersect(bv.halfWidths, halfWidths, R, t21);
}

template<>
bool OBB::intersects(const OBB& bv) const {
    Vector3d pd;
    pd.subtract(bv.c, c);
    bool isect = boxesIntersect(halfWidths, bv.halfWidths, R, bv.R, pd,
            Vector3d::ZERO);

    return isect;
}


//void printBoxDetails(const OBB& obb) {
//    printf("hw: %.17g %.17g %.17g\n", obb.halfWidths.x, obb.halfWidths.y,
//            obb.halfWidths.z);
//    printf(" c: %.17g %.17g %.17g\n", obb.c.x, obb.c.y, obb.c.z);
//    printf(" R: %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
//            obb.R.m[0], obb.R.m[1], obb.R.m[2], obb.R.m[3], obb.R.m[4],
//            obb.R.m[5], obb.R.m[6], obb.R.m[7], obb.R.m[8]);
//}
//
//int printBox(const OBB& obb, int vidx) {
//
//    double x = obb.halfWidths.x;
//    double y = obb.halfWidths.y;
//    double z = obb.halfWidths.z;
//
//    Point3d p[8] = { Point3d(-x, -y, -z), Point3d(x, -y, -z), Point3d(x, y, -z),
//            Point3d(-x, y, -z), Point3d(-x, -y, z), Point3d(x, -y, z), Point3d(
//                    x, y, z), Point3d(-x, y, z) };
//
//    Point3d pout;
//    for (int i = 0; i < 8; i++) {
//        obb.getWorldCoords(p[i], pout);
//        printf("v %lf %lf %lf\n", pout.x, pout.y, pout.z);
//    }
//
//    // faces
//    printf("f %d %d %d %d\n", vidx, vidx + 3, vidx + 2, vidx + 1);
//    printf("f %d %d %d %d\n", vidx + 4, vidx + 5, vidx + 6, vidx + 7);
//    printf("f %d %d %d %d\n", vidx, vidx + 1, vidx + 5, vidx + 4);
//    printf("f %d %d %d %d\n", vidx + 2, vidx + 3, vidx + 7, vidx + 6);
//    printf("f %d %d %d %d\n", vidx + 1, vidx + 2, vidx + 6, vidx + 5);
//    printf("f %d %d %d %d\n", vidx + 3, vidx, vidx + 4, vidx + 7);
//
//    return vidx + 8;
//
//}

}
}
