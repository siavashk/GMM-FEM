#include "mas/bvtree/bvtree.h"
#include "mas/core/math.h"
#include <math.h>
#include <limits>
#include <queue>

#define OCTANT_SIGN(a) (a >= 0 ? 1 : 0)
#define OCTANT(a,o) (OCTANT_SIGN(a.x-o.x)+2*OCTANT_SIGN(a.y-o.y)\
		+4*OCTANT_SIGN(a.z-o.z))

namespace mas {
namespace bvtree {

Boundable::Boundable(int idx)
	: idx(idx) {}

void Boundable::setIndex(int idx) {
	this->idx = idx;
}

int Boundable::getIndex() {
	return idx;
}

bool Boundable::updateBV(PBoundingVolume bv) const {
	return updateBV(bv.get());
}

BoundablePointSet::BoundablePointSet(int idx) : Boundable(idx), pnts() {}
BoundablePointSet::BoundablePointSet(const std::vector<Point3d> &pnts, int idx)
: Boundable(idx), pnts(pnts) {}

void BoundablePointSet::setPoints(const std::vector<Point3d> &pnts) {
	this->pnts = pnts;
}

void BoundablePointSet::addPoint(const Point3d &pnt) {
	pnts.push_back(pnt);
}

bool BoundablePointSet::updateBV(BoundingVolume* bv) const {

	bool updated = false;
	for (Point3d pnt : pnts) {
		updated |= bv->updatePoint(pnt);
	}

	return updated;
}

// default passing to pointer version
bool BoundablePointSet::updateBV(PBoundingVolume bv) const {
	return updateBV(bv.get());
}

void BoundablePointSet::getCentroid(Point3d &c) const {
	c.setZero();
	for (Point3d pnt : pnts) {
		c.add(pnt);
	}
	c.scale(1.0/pnts.size());

}

void BoundablePointSet::getCovariance(const Point3d &centre,
		Matrix3d &cov) const {

	cov.setZero();
	Point3d diff;
	for (Point3d pnt : pnts) {
		diff.set(pnt);
		diff.subtract(centre);
		cov.addOuterProduct(diff, diff);
	}
	cov.scale(1.0/pnts.size());
}

// closest point
double BoundablePointSet::distanceToPoint(const Point3d &pnt, Point3d &nearest) const {

	double dmin = math::DOUBLE_INFINITY;

	for (Point3d mpnt : pnts) {
		double d = pnt.distance(mpnt);
		if (d < dmin) {
			dmin = d;
			nearest.set(mpnt);
		}
	}

	return dmin;

}

double BoundablePointSet::distanceToPoint(const Point3d &pnt, const Vector3d &dir, Point3d &nearest ) const {

	double dmin = math::DOUBLE_INFINITY;

	/*
	 * Computes closest in direction
	double q0 = pnt.dot(pnt);
	double q1 = 2*pnt.dot(dir);
	double q2 = dir.dot(dir);

	Vector3d diff;
	for (Point3d mpnt : pnts) {
		diff.subtract(mpnt, pnt);
		double d = diff.dot(dir)/q2;
		d = sqrt(q0+d*q1+q2);

		if (d < dmin) {
			dmin = d;
			nearest.set(mpnt);
		}
	}
	 */

	return dmin;

}

// Bounding Volume
BoundingVolume::BoundingVolume()
: margin(0) {}

BoundingVolume::BoundingVolume(double margin)
: margin(margin) {}

double BoundingVolume::getMargin() const {
	return margin;
}

// default visitor pattern
bool BoundingVolume::intersects(const PBoundingVolume bv) const {
	return bv->intersectsVisitor(this);
}

// default visitor pattern
bool BoundingVolume::intersects(const BoundingVolume* bv) const {
	return bv->intersectsVisitor(this);
}

bool BoundingVolume::intersectsVisitor(const PBoundingVolume bv)
const {
	// I don't know what I am, so take my bounding sphere
	BoundingSphere bs = getBoundingSphere();
	return bv->intersectsSphere(bs.c, bs.r);
}

bool BoundingVolume::intersectsVisitor(const BoundingVolume* bv)
const {
	// I don't know what I am, so take my bounding sphere
	BoundingSphere bs = getBoundingSphere();
	return bv->intersectsSphere(bs.c, bs.r);
}

bool BoundingVolume::update(const PBoundable b) {
	// I don't know any boundables, so pass off to them
	return b->updateBV(this);
}

const unsigned long BoundingSphere::UNIQUE_ID = BVTREE_BVID_BS;

BoundingSphere::BoundingSphere()
: BoundingVolume(0), r(0), c(0,0,0) {}

BoundingSphere::BoundingSphere(const BoundingSphere &copyMe)
: BoundingVolume(copyMe.margin), r(copyMe.r), c(copyMe.c) {}

BoundingSphere::BoundingSphere(const Point3d &c, double r,
		double margin)
: BoundingVolume(margin), r(r), c(c) {}

unsigned long BoundingSphere::uniqueClassId() const {
	return BoundingSphere::UNIQUE_ID;
}

void BoundingSphere::set(const Point3d &c, double r) {
	this->r = r;
	this->c.set(c);
}

void BoundingSphere::setRadius(double r) {
	this->r = r;
}

double BoundingSphere::getRadius() const {
	return r;
}

void BoundingSphere::setCentre(const Point3d &c) {
	this->c.set(c);
}

void BoundingSphere::getCentre(Point3d &c) const {
	c.set(this->c);
}

void BoundingSphere::setMargin(double m) {
	r = r - margin + m;	// adjust margin
	margin = m;
}

bool BoundingSphere::intersectsPoint(const Point3d &p) const {
	if (p.distanceSquared(c) <= r*r) {	// closed ball
		return true;
	}
	return false;
}

bool BoundingSphere::intersectsSphere(const Point3d &c,
		double r) const {

	double r2 = (r+this->r);
	r2 = r2*r2;

	if (this->c.distanceSquared(c) <= r2) {
		return true;
	}
	return false;
}

bool BoundingSphere::intersectsLine(const Point3d &p,
		const Vector3d &v) const {
	// orthogonal projection onto line
	Point3d nearest = this->c;
	nearest.subtract(p);
	double s = nearest.dot(v)/v.dot(v);
	nearest.scaledAdd(p, s, v);

	if (nearest.distanceSquared(this->c) <= this->r*this->r) {
		return true;
	}
	return false;
}

bool BoundingSphere::intersectsRay(const Point3d &p,
		const Vector3d &v) const {

	Point3d nearest = this->c;
	nearest.subtract(p);
	double s = nearest.dot(v);
	if (s < 0) {
		return false;
	}
	s = s/v.dot(v);
	nearest.scaledAdd(p, s, v);

	if (nearest.distanceSquared(this->c) <= this->r*this->r) {
		return true;
	}
	return false;
}

bool BoundingSphere::intersectsPlane(const Plane &p) const {
	double d = p.distanceSigned(c);
	if (fabs(d) <= r) {
		return true;
	}
	return false;
}

bool BoundingSphere::intersects(const PBoundingVolume bv) const {
	return bv->intersectsSphere(c, r);
}

bool BoundingSphere::intersects(const BoundingVolume* bv) const {
	return bv->intersectsSphere(c, r);
}

bool BoundingSphere::intersects(const PBoundingSphere bs) const {
	return intersectsSphere(bs->c, bs->r);
}

bool BoundingSphere::intersects(const BoundingSphere* bs) const {
	return intersectsSphere(bs->c, bs->r);
}

bool BoundingSphere::intersectsVisitor(const PBoundingVolume bv)
const {
	return bv->intersectsSphere(c, r);
}

bool BoundingSphere::intersectsVisitor(const BoundingVolume* bv)
const {
	return bv->intersectsSphere(c, r);
}

double BoundingSphere::distanceToPoint(const Point3d &pnt,
		Point3d &nearest) const {

	double d = c.distance(pnt) - r;
	// inside
	if (d <= 0) {
		nearest.set(pnt);
		return 0;
	}

	// closest point
	nearest.interpolate(pnt, d/(d+r), c);
	return d;
}

double BoundingSphere::distanceToPoint(const Point3d &pnt,
		const Vector3d &dir, Point3d &nearest) const {

	Vector3d pc = Vector3d(pnt);
	pc.subtract(c);

	// inside
	if (pc.norm() <= r) {
		nearest.set(pnt);
		return 0;
	}

	// intersect ray
	double dirpc = dir.dot(pc);
	double dirsq = dir.dot(dir);
	double d = dirpc*dirpc - dirsq*(pc.dot(pc) - r*r);

	// doesn't intersect
	if (d < 0) {
		// negative direction
		if (dir.dot(pc) > 0) {
			nearest.x = -math::signum(dir.x)*math::DOUBLE_INFINITY;
			nearest.y = -math::signum(dir.y)*math::DOUBLE_INFINITY;
			nearest.z = -math::signum(dir.z)*math::DOUBLE_INFINITY;
		} else {
			// positive direction
			nearest.x = math::signum(dir.x)*math::DOUBLE_INFINITY;
			nearest.y = math::signum(dir.y)*math::DOUBLE_INFINITY;
			nearest.z = math::signum(dir.z)*math::DOUBLE_INFINITY;
		}
		return math::DOUBLE_INFINITY;
	}

	// both distances
	d = sqrt(d)/dirsq;
	double d1 = dirpc/dirsq;
	double d2 = d1+d;
	d1 = d1-d;

	if (fabs(d1) <= fabs(d2) ) {
		d = fabs(d1);
	} else {
		d = fabs(d2);
	}
	nearest.set(pnt);
	nearest.scaledAdd(d, dir);
	return d;
}

BoundingSphere BoundingSphere::getBoundingSphere() const {
	return *this;
}

void BoundingSphere::bound(const PBoundableList &blist) {

	// get total centroid, then compute radius
	Point3d centroid;
	c.set(0,0,0);

	// set center as average point
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		(*pit)->getCentroid(centroid);
		c.add(centroid);
	}
	c.scale(1.0/blist.size());

	r = 0;
	// update radius
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		(*pit)->updateBV(this);
	}

}

void BoundingSphere::split(const PBoundableList &blist,
		PBoundableListList &out) const {

	// split by octree
	out = PBoundableListList(8, PBoundableList());
	int outSizes[8] = {0,0,0,0,0,0,0,0};
	Point3d centroid;
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		// separate into quadrants
		(*pit)->getCentroid(centroid);
		int octant = OCTANT(centroid,c);
		out[octant].push_back(*pit);
		outSizes[octant]++;
	}
	double a = outSizes[0];
}

bool BoundingSphere::updatePoint(const Point3d &p) {
	double rnew = p.distance(c) + margin;
	if (rnew > r) {
		r = rnew;
		return true;
	}
	return false;
}

bool BoundingSphere::updateSphere(const Point3d &c, double r) {
	double rnew = this->c.distance(c)+r+margin;
	if (rnew > r) {
		r = rnew;
		return true;
	}
	return false;
}

PBoundingVolume BoundingSphere::copy() const {
	return BVFactory::createBoundingSphere(*this);
}

int BoundingBox::boxCorners[8][3] = {	{-1, -1, -1},
		{ 1, -1, -1},
		{ 1,  1, -1},
		{-1,  1, -1},
		{-1, -1,  1},
		{ 1, -1,  1},
		{ 1,  1,  1},
		{-1,  1,  1} };

BoundingBox::BoundingBox()
: BoundingVolume(0), c(0,0,0), halfWidths(0,0,0) {}

BoundingBox::BoundingBox(const BoundingBox &copyMe)
: BoundingVolume(copyMe.margin), c(copyMe.c),
  halfWidths(copyMe.halfWidths) {}

BoundingBox::BoundingBox(const Point3d &c, const Vector3d &hw,
		double margin)
: BoundingVolume(margin), c(c), halfWidths(hw) {}

void BoundingBox::set(const Point3d &c, const Vector3d &hw) {
	this->c.set(c);
	halfWidths.set(hw);
}

void BoundingBox::setHalfWidths(const Vector3d &hw) {
	halfWidths.set(hw);
}

void BoundingBox::getHalfWidths(Vector3d &hw) {
	hw.set(halfWidths);
}

void BoundingBox::setCentre(const Point3d &c) {
	this->c.set(c);
}

void BoundingBox::getCentre(Point3d &c) const {
	c.set(this->c);
}

void BoundingBox::setMargin (double m) {
	halfWidths.add(m-margin, m-margin, m-margin);
	margin = m;
}

void BoundingBox::getCorner(int idx, Point3d &pnt) const {
	pnt.set( boxCorners[idx][0]*halfWidths.x,
			boxCorners[idx][1]*halfWidths.y,
			boxCorners[idx][2]*halfWidths.z);
	getWorldCoords(pnt, pnt);
}

bool BoundingBox::intersectsPoint(const Point3d &p) const {

	Point3d pl;
	getLocalCoords(p, pl);

	bool status =  ( pl.x >= - halfWidths.x)
														&& ( pl.x <= halfWidths.x)
														&& ( pl.y >= - halfWidths.y)
														&& ( pl.y <= halfWidths.y)
														&& ( pl.z >= - halfWidths.z)
														&& ( pl.z <= halfWidths.z);

	return status;
}

bool BoundingBox::intersectsSphere(const Point3d &c, double r) const {

	Point3d pl;
	getLocalCoords(c, pl);
	bool status =  ( pl.x + r >= - halfWidths.x)
														&& ( pl.x - r <= halfWidths.x)
														&& ( pl.y + r >= - halfWidths.y)
														&& ( pl.y - r <= halfWidths.y)
														&& ( pl.z + r >= - halfWidths.z)
														&& ( pl.z - r <= halfWidths.z);

	return status;
}

bool BoundingBox::intersectsLine(const Point3d &p,
		const Vector3d &v) const {

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
	for (int i=0; i<3; i++) {
		double hwb = halfWidths.get(i);
		double pb = pl.get(i);
		double vb = vl.get(i);

		double minb = -hwb;
		double maxb = hwb;

		if (vb == 0) {
			// parallel to and outside box
			if (minb - pb > 0 || maxb - pb < 0) {
				return false;
			}
		} else {

			// intersect plane
			double d = 1.0/vb;
			double dist0 = (minb - pb)*d;
			double dist1 = (maxb - pb)*d;
			if (vb < 0) {
				double tmp = dist0;
				dist0 = dist1;
				dist1 = tmp;
			}

			if ( minl < dist0 ) {
				minl = dist0;
			}
			if ( maxl > dist1) {
				maxl = dist1;
			}

			if (minl > maxl) {
				return false;
			}
		}

	}
	return true;
}

bool BoundingBox::intersectsRay(const Point3d &p,
		const Vector3d &v) const {

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
	double maxl = std::numeric_limits<double>::infinity();

	// intersect with front/back planes
	for (int i=0; i<3; i++) {
		double hwb = halfWidths.get(i);
		double pb = pl.get(i);
		double vb = vl.get(i);

		double minb = -hwb;
		double maxb = hwb;

		if (vb == 0) {
			// parallel to and outside box
			if (minb - pb > 0 || maxb - pb < 0) {
				return false;
			}
		} else {

			// intersect plane
			double d = 1.0/vb;
			double dist0 = (minb - pb)*d;
			double dist1 = (maxb - pb)*d;

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

bool BoundingBox::intersectsPlane(const Plane &p) const {

	// grab normal and point on plane
	Vector3d nl = p.normal;
	Point3d pl = p.normal;
	pl.scale(-p.d);

	// transform plane to local coordinates
	getLocalCoords(nl, nl);
	getLocalCoords(pl, pl);
	double d = - nl.dot(pl);

	// check plane has at least one corner on each side
	bool up = false;
	bool down = false;
	double x[] = {-halfWidths.x, halfWidths.x};
	double y[] = {-halfWidths.y, halfWidths.y};
	double z[] = {-halfWidths.z, halfWidths.z};
	double b = 0;

	for (int i=0; i<2; i++) {
		for  (int j=0; j<2; j++) {
			for (int k=0; k<2; k++) {
				b = nl.x*x[i] + nl.y*y[j] + nl.z*z[k] + d;
				up = up | (b >=0);
				down = down | (b<=0);
				if (up && down) {
					return true;
				}
			}
		}
	}

	return false;
}

BoundingSphere BoundingBox::getBoundingSphere() const {
	BoundingSphere sphere =
			BoundingSphere(c, halfWidths.norm(), margin);
	return sphere;
}

bool BoundingBox::updatePoint(const Point3d &p) {
	return updateSphere(p, 0);
}

bool BoundingBox::updateSphere(const Point3d &c, double r) {

	Point3d cl;
	getLocalCoords(c, cl);
	Point3d max = halfWidths;
	Point3d min = halfWidths;
	min.scale(-1);

	bool modified = false;

	r = r+margin;
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
		cl.set(min);
		cl.add(max);
		cl.scale(0.5);
		getWorldCoords(cl, this->c);

		halfWidths.set(max);
		halfWidths.subtract(min);
		halfWidths.scale(0.5);
		return true;
	}
	return false;
}

double BoundingBox::distanceToPoint(const Point3d &pnt,
		Point3d &nearest) const {

	Point3d lpnt;
	getLocalCoords(pnt, lpnt);

	// find nearest face/edge or corner
	nearest.set(lpnt);

	if (nearest.x > halfWidths.x) {
		nearest.x = halfWidths.x;
	} else if (nearest.x < -halfWidths.x ) {
		nearest.x = -halfWidths.x;
	}

	if (nearest.y > halfWidths.y) {
		nearest.y = halfWidths.y;
	} else if (nearest.y < -halfWidths.y ) {
		nearest.y = -halfWidths.y;
	}

	if (nearest.z > halfWidths.z) {
		nearest.z = halfWidths.z;
	} else if (nearest.z < -halfWidths.z ) {
		nearest.z = -halfWidths.z;
	}

	double dist = nearest.distance(lpnt);

	getWorldCoords(nearest, nearest);

	return dist;
}

double BoundingBox::distanceToPoint(const Point3d &pnt,
		const Vector3d &dir, Point3d &nearest ) const {

	//search along dir
	Point3d lpnt;
	Vector3d ldir;
	getLocalCoords(pnt, lpnt);
	getLocalCoords(dir, ldir);

	// first check if inside
	if (lpnt.x >= -halfWidths.x && lpnt.x <= halfWidths.x
			&& lpnt.y >= -halfWidths.y && lpnt.y <= halfWidths.y
			&& lpnt.z >= -halfWidths.z && lpnt.z <= halfWidths.z ) {
		nearest.set(pnt);
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
		double s = (-halfWidths.x-lpnt.x)/ldir.x;
		p.scaledAdd(lpnt, s, ldir);
		s = p.distance(lpnt);

		// check if p is in box and if point is closer
		if (p.x >= -halfWidths.x && p.x <= halfWidths.x
				&& p.y >= -halfWidths.y && p.y <= halfWidths.y
				&& p.z >= -halfWidths.z && p.z <= halfWidths.z
				&& (s < dmin)) {
			nearest.set(p);
			dmin = s;
		}

		s = (halfWidths.x-lpnt.x)/ldir.x;
		p.scaledAdd(lpnt, s, ldir);
		s = p.distance(lpnt);

		// check if p is in box
		if (p.x >= -halfWidths.x && p.x <= halfWidths.x
				&& p.y >= -halfWidths.y && p.y <= halfWidths.y
				&& p.z >= -halfWidths.z && p.z <= halfWidths.z
				&& (s < dmin)) {
			nearest.set(p);
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
		double s = (-halfWidths.y-lpnt.y)/ldir.y;
		p.scaledAdd(lpnt, s, ldir);
		s = p.distance(lpnt);

		// check if p is in box and if point is closer
		if (p.x >= -halfWidths.x && p.x <= halfWidths.x
				&& p.y >= -halfWidths.y && p.y <= halfWidths.y
				&& p.z >= -halfWidths.z && p.z <= halfWidths.z
				&& (s < dmin)) {
			nearest.set(p);
			dmin = s;
		}

		s = (halfWidths.y-lpnt.y)/ldir.y;
		p.scaledAdd(lpnt, s, ldir);
		s = p.distance(lpnt);

		// check if p is in box
		if (p.x >= -halfWidths.x && p.x <= halfWidths.x
				&& p.y >= -halfWidths.y && p.y <= halfWidths.y
				&& p.z >= -halfWidths.z && p.z <= halfWidths.z
				&& (s < dmin)) {
			nearest.set(p);
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
		double s = (-halfWidths.z-lpnt.z)/ldir.z;
		p.scaledAdd(lpnt, s, ldir);
		s = p.distance(lpnt);

		// check if p is in box and if point is closer
		if (p.x >= -halfWidths.x && p.x <= halfWidths.x
				&& p.y >= -halfWidths.y && p.y <= halfWidths.y
				&& p.z >= -halfWidths.z && p.z <= halfWidths.z
				&& (s < dmin)) {
			nearest.set(p);
			dmin = s;
		}

		s = (halfWidths.z-lpnt.z)/ldir.z;
		p.scaledAdd(lpnt, s, ldir);
		s = p.distance(lpnt);

		// check if p is in box
		if (p.x >= -halfWidths.x && p.x <= halfWidths.x
				&& p.y >= -halfWidths.y && p.y <= halfWidths.y
				&& p.z >= -halfWidths.z && p.z <= halfWidths.z
				&& (s < dmin)) {
			nearest.set(p);
			dmin = s;
		}
	}

	getWorldCoords(nearest, nearest);
	return dmin;
}

const unsigned long AABB::UNIQUE_ID = BVTREE_BVID_AABB;

AABB::AABB()
: BoundingBox() {}

AABB::AABB(const AABB &copyMe)
: BoundingBox(copyMe) {}

AABB::AABB(const Point3d &c, const Vector3d &hw, double margin)
: BoundingBox(c, hw, margin) {}

PBoundingVolume AABB::copy() const {
	return BVFactory::createAABB(*this);
}

unsigned long AABB::uniqueClassId() const {
	return AABB::UNIQUE_ID;
}

void AABB::getLocalCoords(const Point3d &p, Point3d &out) const {
	out.subtract(p, c);
}

void AABB::getLocalCoords(const Vector3d &v, Vector3d &out) const {
	out.set(v);
}

void AABB::getWorldCoords(const Point3d &p, Point3d &out) const {
	out.set(c);	// first in case out == c, otherwise overwrites c
	out.add(p);
}

void AABB::getWorldCoords(const Vector3d &v, Vector3d &out) const {
	out.set(v);
}

bool AABB::intersects(const PBoundingVolume bv) const {
	return intersects(bv.get());	// raw pointer
}

bool AABB::intersects(const BoundingVolume* bv) const {
	switch(bv->uniqueClassId()) {
	case BoundingSphere::UNIQUE_ID: {
		const BoundingSphere* bs = static_cast<const BoundingSphere*>(bv);
		return intersectsSphere(bs->c, bs->r);
	}
	case AABB::UNIQUE_ID: {
		const AABB* aabb = static_cast<const AABB*>(bv);
		return intersects(aabb);
	}
	}
	return bv->intersectsVisitor(this);
}

bool AABB::intersectsVisitor(const PBoundingVolume bv) const {
	return intersects(bv.get());
}

bool AABB::intersectsVisitor(const BoundingVolume* bv) const {
	switch(bv->uniqueClassId()) {
	case BoundingSphere::UNIQUE_ID: {
		const BoundingSphere* bs = static_cast<const BoundingSphere*>(bv);
		return intersectsSphere(bs->c, bs->r);
	}
	case AABB::UNIQUE_ID: {
		const AABB* aabb = static_cast<const AABB*>(bv);
		return intersects(aabb);
	}
	}

	// rely on bounding sphere
	const BoundingSphere &bs = bv->getBoundingSphere();
	return intersectsSphere(bs.c, bs.r);
}

bool AABB::intersects(const PAABB bv) const {
	return intersects(bv.get());
}

bool AABB::intersects(const AABB* bv) const {

	// both axis-aligned, check if off to any side
	if (       bv->c.x - bv->halfWidths.x > c.x + halfWidths.x) {
		return false;
	} else if (bv->c.x + bv->halfWidths.x < c.x - halfWidths.x) {
		return false;
	} else if (bv->c.y - bv->halfWidths.y > c.y + halfWidths.y) {
		return false;
	} else if (bv->c.y + bv->halfWidths.y < c.y - halfWidths.y) {
		return false;
	} else if (bv->c.z - bv->halfWidths.z > c.z + halfWidths.z) {
		return false;
	} else if (bv->c.z + bv->halfWidths.z < c.z - halfWidths.z) {
		return false;
	}

	return true;
}

void AABB::bound(const PBoundableList &blist) {

	// get total centroid, then compute radius
	Point3d centroid;
	c.set(0,0,0);

	// set center as average point
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		(*pit)->getCentroid(centroid);
		c.add(centroid);
	}
	c.scale(1.0/blist.size());

	halfWidths.set(0,0,0);
	// update bounds
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		(*pit)->updateBV(this);
	}

}

void AABB::split(const PBoundableList &blist,
		PBoundableListList &out) const {

	// split box
	const Vector3d* axes[] = {&Vector3d::X_AXIS,
			&Vector3d::Y_AXIS,
			&Vector3d::Z_AXIS};

	// sort axes
	if (halfWidths.y > halfWidths.x) {
		if (halfWidths.z > halfWidths.y) {
			const Vector3d* tmp = axes[0];
			axes[0] = axes[2];
			axes[2] = tmp;
		} else if (halfWidths.z < halfWidths.x) {
			const Vector3d* tmp = axes[0];
			axes[0] = axes[1];
			axes[1] = axes[2];
			axes[2] = tmp;
		} else {
			const Vector3d* tmp = axes[0];
			axes[0] = axes[1];
			axes[1] = tmp;
		}
	} else {
		if (halfWidths.z > halfWidths.x) {
			const Vector3d* tmp = axes[0];
			axes[0] = axes[2];
			axes[2] = axes[1];
			axes[1] = tmp;
		} else if (halfWidths.z > halfWidths.y) {
			const Vector3d* tmp = axes[1];
			axes[1] = axes[2];
			axes[2] = tmp;
		}
	}


	for (int i=0; i<3; i++) {

		const Vector3d &normal = *axes[i];
		double d = -normal.dot(c);

		out = PBoundableListList(2, PBoundableList());
		Point3d centroid;
		for (PBoundableList::const_iterator pit = blist.begin();
				pit < blist.end(); pit++) {
			(*pit)->getCentroid(centroid);
			if (centroid.dot(normal) + d < 0) {
				out[0].push_back(*pit);
			} else {
				out[1].push_back(*pit);
			}
		}

		if (out[0].size() > 0 && out[1].size() > 0) {
			return;
		}
	}

}

const unsigned long OBB::UNIQUE_ID = BVTREE_BVID_OBB;

// Game Physics, David Eberly, Pg. 445
bool OBB::boxesIntersect (
		const Vector3d &hw1, const Vector3d &hw2, const RotationMatrix3d &R1,
		const RotationMatrix3d &R2, const Vector3d &pd, const Vector3d &px) {

	double t;
	double cutoff = 1-1e-10;

	// mij and pi give the transformation from the argument obb to this obb.

	// A1 x A2 = A0
	double p_x = R1.m[IDX3D_00] * pd.x + R1.m[IDX3D_10] * pd.y + R1.m[IDX3D_20] * pd.z + px.x;
	if ((t = p_x) < 0)
		t = -t;
	double m00 = R1.m[IDX3D_00] * R2.m[IDX3D_00] + R1.m[IDX3D_10] * R2.m[IDX3D_10] + R1.m[IDX3D_20] * R2.m[IDX3D_20];
	double m01 = R1.m[IDX3D_00] * R2.m[IDX3D_01] + R1.m[IDX3D_10] * R2.m[IDX3D_11] + R1.m[IDX3D_20] * R2.m[IDX3D_21];
	double m02 = R1.m[IDX3D_00] * R2.m[IDX3D_02] + R1.m[IDX3D_10] * R2.m[IDX3D_12] + R1.m[IDX3D_20] * R2.m[IDX3D_22];
	double abs00 = (m00 >= 0 ? m00 : -m00);
	double abs01 = (m01 >= 0 ? m01 : -m01);
	double abs02 = (m02 >= 0 ? m02 : -m02);
	if (t > (hw1.x + hw2.x * abs00 + hw2.y * abs01 + hw2.z * abs02))
		return false;

	// B1 x B2 = B0
	double m10 = R1.m[IDX3D_01] * R2.m[IDX3D_00] + R1.m[IDX3D_11] * R2.m[IDX3D_10] + R1.m[IDX3D_21] * R2.m[IDX3D_20];
	double m20 = R1.m[IDX3D_02] * R2.m[IDX3D_00] + R1.m[IDX3D_12] * R2.m[IDX3D_10] + R1.m[IDX3D_22] * R2.m[IDX3D_20];
	double p_y = R1.m[IDX3D_01] * pd.x + R1.m[IDX3D_11] * pd.y + R1.m[IDX3D_21] * pd.z + px.y;
	double p_z = R1.m[IDX3D_02] * pd.x + R1.m[IDX3D_12] * pd.y + R1.m[IDX3D_22] * pd.z + px.z;
	if ((t = p_x * m00 + p_y * m10 + p_z * m20) < 0)
		t = -t;
	double abs10 = (m10 >= 0 ? m10 : -m10);
	double abs20 = (m20 >= 0 ? m20 : -m20);
	if (t > (hw2.x + hw1.x * abs00 + hw1.y * abs10 + hw1.z * abs20))
		return false;

	// A2 x A0 = A1
	if ((t = p_y) < 0)
		t = -t;
	double m11 = R1.m[IDX3D_01] * R2.m[IDX3D_01] + R1.m[IDX3D_11] * R2.m[IDX3D_11] + R1.m[IDX3D_21] * R2.m[IDX3D_21];
	double m12 = R1.m[IDX3D_01] * R2.m[IDX3D_02] + R1.m[IDX3D_11] * R2.m[IDX3D_12] + R1.m[IDX3D_21] * R2.m[IDX3D_22];
	double abs11 = (m11 >= 0 ? m11 : -m11);
	double abs12 = (m12 >= 0 ? m12 : -m12);
	if (t > (hw1.y + hw2.x * abs10 + hw2.y * abs11 + hw2.z * abs12))
		return false;

	// A0 x A1 = A2
	if ((t = p_z) < 0)
		t = -t;
	double m21 = R1.m[IDX3D_02] * R2.m[IDX3D_01] + R1.m[IDX3D_12] * R2.m[IDX3D_11] + R1.m[IDX3D_22] * R2.m[IDX3D_21];
	double m22 = R1.m[IDX3D_02] * R2.m[IDX3D_02] + R1.m[IDX3D_12] * R2.m[IDX3D_12] + R1.m[IDX3D_22] * R2.m[IDX3D_22];
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
	if (   abs00 > cutoff || abs01 > cutoff || abs02 > cutoff
		|| abs10 > cutoff || abs11 > cutoff || abs12 > cutoff
		|| abs20 > cutoff || abs21 > cutoff || abs22 > cutoff) {
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

bool OBB::boxesIntersect(const Vector3d &hw1,
		const Vector3d &hw2, const RotationMatrix3d &R21,
		const Vector3d &t21) {

	// start with simple sphere test
	if (t21.norm() > hw1.norm() + hw2.norm()) {
		return false;
	}

	double cutoff = 1.0-1e-10;

	// OBB intersection:
	// OBBTree: A Hierarchichal Structure for Rapid Interference
	// Detection", Gottschalk Lin & Manocha

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
	if (   abs00 > cutoff || abs01 > cutoff || abs02 > cutoff
		|| abs10 > cutoff || abs11 > cutoff || abs12 > cutoff
		|| abs20 > cutoff || abs21 > cutoff || abs22 > cutoff) {
		return true;
	}

	// A0 x B0
	if ((t = t21.z * m10 - t21.y * m20) < 0) {
		t = -t;
	}
	if (t > (hw1.y * abs20 + hw1.z * abs10 + hw2.y * abs02
			+ hw2.z * abs01)) {
		return false;
	}

	// A0 x B1
	if ((t = t21.z * m11 - t21.y * m21) < 0) {
		t = -t;
	}
	if (t > (hw1.y * abs21 + hw1.z * abs11 + hw2.x * abs02
			+ hw2.z * abs00)) {
		return false;
	}

	// A0 x B2
	if ((t = t21.z * m12 - t21.y * m22) < 0) {
		t = -t;
	}
	if (t > (hw1.y * abs22 + hw1.z * abs12 + hw2.x * abs01
			+ hw2.y * abs00)) {
		return false;
	}

	// A1 x B0
	if ((t = t21.x * m20 - t21.z * m00) < 0) {
		t = -t;
	}
	if (t > (hw1.x * abs20 + hw1.z * abs00 + hw2.y * abs12
			+ hw2.z * abs11)) {
		return false;
	}

	// A1 x B1
	if ((t = t21.x * m21 - t21.z * m01) < 0) {
		t = -t;
	}
	if (t > (hw1.x * abs21 + hw1.z * abs01 + hw2.x * abs12
			+ hw2.z * abs10)) {
		return false;
	}

	// A1 x B2
	if ((t = t21.x * m22 - t21.z * m02) < 0) {
		t = -t;
	}
	if (t > (hw1.x * abs22 + hw1.z * abs02 + hw2.x * abs11
			+ hw2.y * abs10)) {
		return false;
	}

	// A2 x B0
	if ((t = t21.y * m00 - t21.x * m10) < 0) {
		t = -t;
	}
	if (t > (hw1.x * abs10 + hw1.y * abs00 + hw2.y * abs22
			+ hw2.z * abs21)) {
		return false;
	}

	// A2 x B1
	if ((t = t21.y * m01 - t21.x * m11) < 0) {
		t = -t;
	}
	if (t > (hw1.x * abs11 + hw1.y * abs01 + hw2.x * abs22
			+ hw2.z * abs20)) {
		return false;
	}

	// A2 x B2
	if ((t = t21.y * m02 - t21.x * m12) < 0) {
		t = -t;
	}

	if (t > (hw1.x * abs12 + hw1.y * abs02 + hw2.x * abs21
			+ hw2.y * abs20)) {
		return false;
	}

	return true;

}

OBB::OBB()
: BoundingBox() {}

OBB::OBB(const OBB &copyMe)
: BoundingBox(copyMe), R(copyMe.R) {}

OBB::OBB(const AABB &copyMe)
: BoundingBox(copyMe), R(RotationMatrix3d::IDENTITY) {}

OBB::OBB(const Point3d &c, const RotationMatrix3d &R,
		const Vector3d &hw)
: BoundingBox(c, hw), R(R) {}

OBB::OBB(const RigidTransform3d &trans, const Vector3d &hw)
: BoundingBox(trans.t, hw), R(trans.R) {}

PBoundingVolume OBB::copy() const {
	return BVFactory::createOBB(*this);
}

unsigned long OBB::uniqueClassId() const {
	return OBB::UNIQUE_ID;
}

void OBB::set(const Point3d &c, const RotationMatrix3d &R,
		const Vector3d &hw) {
	this->c.set(c);
	this->halfWidths.set(hw);
	this->R.set(R);
}

void OBB::set(const RigidTransform3d &trans, const Vector3d &hw) {
	c.set(trans.t);
	halfWidths.set(hw);
	R.set(trans.R);
}

void OBB::setRotation(const RotationMatrix3d &R) {
	this->R.set(R);
}

void OBB::getRotation(RotationMatrix3d &R) {
	R.set(this->R);
}

void OBB::getLocalCoords(const Point3d &p, Point3d &out) const {
	Point3d tmp = p;
	tmp.subtract(c);
	R.multiplyLeft(tmp, out);
}

void OBB::getLocalCoords(const Vector3d &v, Vector3d &out) const {
	R.multiplyLeft(v, out);
}

void OBB::getWorldCoords(const Point3d &p, Point3d &out) const {
	Point3d tmp = c;
	R.multiply(p, out);
	out.add(tmp);
}

void OBB::getWorldCoords(const Vector3d &v, Vector3d &out) const {
	R.multiply(v, out);
}

bool OBB::intersects(const BoundingVolume* bv) const {

	switch(bv->uniqueClassId()) {
	case BoundingSphere::UNIQUE_ID: {
		const BoundingSphere* bs = static_cast<const BoundingSphere*>(bv);
		return intersectsSphere(bs->c, bs->r);
	}
	case AABB::UNIQUE_ID: {
		const AABB* aabb = static_cast<const AABB*>(bv);
		return intersects(aabb);
	}
	case OBB::UNIQUE_ID: {
		const OBB* obb = static_cast<const OBB*>(bv);
		return intersects(obb);
	}
	}
	return bv->intersectsVisitor(this);
}

bool OBB::intersects(const PBoundingVolume bv) const {
	return intersects(bv.get());
}

bool OBB::intersectsVisitor(const BoundingVolume* bv) const {

	switch(bv->uniqueClassId()) {
	case BoundingSphere::UNIQUE_ID: {
		const BoundingSphere* bs = static_cast<const BoundingSphere*>(bv);
		return intersectsSphere(bs->c, bs->r);
	}
	case AABB::UNIQUE_ID: {
		const AABB* aabb = static_cast<const AABB*>(bv);
		return intersects(aabb);
	}
	case OBB::UNIQUE_ID: {
		const OBB* obb = static_cast<const OBB*>(bv);
		return intersects(obb);
	}
	}

	// resort to using bounding sphere
	BoundingSphere bs = bv->getBoundingSphere();
	return intersectsSphere(bs.c, bs.r);
}

bool OBB::intersectsVisitor(const PBoundingVolume bv) const {
	return intersects(bv.get());
}

bool OBB::intersects(const AABB* bv) const {
	Vector3d t21 = c;
	t21.subtract(bv->c);
	return boxesIntersect(bv->halfWidths, halfWidths, R, t21);
}

bool OBB::intersects(const PAABB bv) const {
	return intersects(bv.get());
}

void printBoxDetails(const OBB &obb) {
	printf("hw: %.17g %.17g %.17g\n", obb.halfWidths.x, obb.halfWidths.y, obb.halfWidths.z);
	printf(" c: %.17g %.17g %.17g\n", obb.c.x, obb.c.y, obb.c.z);
	printf(" R: %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
			obb.R.m[0], obb.R.m[1], obb.R.m[2],
			obb.R.m[3], obb.R.m[4], obb.R.m[5],
			obb.R.m[6], obb.R.m[7], obb.R.m[8]);
}

int printBox(const OBB &obb, int vidx) {

	double x = obb.halfWidths.x;
	double y = obb.halfWidths.y;
	double z = obb.halfWidths.z;

	Point3d p[8] = {
			Point3d(-x, -y, -z),
			Point3d( x, -y, -z),
			Point3d( x,  y, -z),
			Point3d(-x,  y, -z),
			Point3d(-x, -y,  z),
			Point3d( x, -y,  z),
			Point3d( x,  y,  z),
			Point3d(-x,  y,  z)
	};

	Point3d pout;
	for (int i=0; i<8; i++) {
		obb.getWorldCoords(p[i], pout);
		printf("v %lf %lf %lf\n", pout.x, pout.y, pout.z);
	}

	// faces
	printf("f %d %d %d %d\n", vidx, vidx+3, vidx+2, vidx+1);
	printf("f %d %d %d %d\n", vidx+4, vidx+5, vidx+6, vidx+7);
	printf("f %d %d %d %d\n", vidx, vidx+1, vidx+5, vidx+4);
	printf("f %d %d %d %d\n", vidx+2, vidx+3, vidx+7, vidx+6);
	printf("f %d %d %d %d\n", vidx+1, vidx+2, vidx+6, vidx+5);
	printf("f %d %d %d %d\n", vidx+3, vidx, vidx+4, vidx+7);

	return vidx+8;

}

bool OBB::intersects(const OBB* bv) const {

	Vector3d pd;
	pd.subtract (bv->c, c);

	bool isect = boxesIntersect(
			halfWidths, bv->halfWidths,
			R, bv->R,
			pd, Vector3d::ZERO);

	return isect;
}

bool OBB::intersects(const POBB bv) const {
	return intersects(bv.get());
}

void OBB::bound(const PBoundableList &blist) {

	c.set(0,0,0);

	// set center as average point
	Point3d centroid;
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		(*pit)->getCentroid(centroid);
		c.add(centroid);
	}
	c.scale(1.0/blist.size());

	// compute tight-fitting axes via SVD
	Matrix3d cov = Matrix3d(0,0,0,0,0,0,0,0,0);
	Matrix3d covl;

	// covariance... technically, according to paper,
	// should be using surface area of convex hull.  However,
	// this doesn't generalize well and expects all objects
	// to be polygons
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		(*pit)->getCovariance(c, covl);
		cov.add(covl);
	}

	Matrix3d U;
	Vector3d S;
	Matrix3d V;
	math::svd3(cov, U, S, V);
	// change to rotation
	if (U.determinant() < 0) {
		U.scaleColumn(2, -1);
	}

	this->R.set(U);

	halfWidths.set(0,0,0);
	// update bounds
	for (PBoundableList::const_iterator pit = blist.begin();
			pit < blist.end(); pit++) {
		(*pit)->updateBV(this);
	}

}

void OBB::split(const PBoundableList &blist,
		PBoundableListList &out) const {

	// division plane given by normal.dot(x) + d = 0
	Vector3d normal;
	Point3d centroid;

	int outSizes[] = {0, 0};
	int nElems = blist.size();

	// start with largest axis, if fails continue
	// to second largest
	for (int i=0; i<3; i++) {
		R.getColumn(i, normal);
		double d = -normal.dot(c);

		// split along first dimension
		out = PBoundableListList(2, PBoundableList());
		for (PBoundableList::const_iterator pit = blist.begin();
				pit < blist.end(); pit++) {
			(*pit)->getCentroid(centroid);
			if (normal.dot(centroid) + d < 0) {
				out[0].push_back(*pit);
			} else {
				out[1].push_back(*pit);
			}
		}
		outSizes[0] = out[0].size();
		outSizes[1] = out[1].size();
		if (out[0].size() > 0 && out[1].size() > 0) {
			return;
		}
	}
}


// BV Node
BVNode::BVNode()
: bv(), elems(), children(), _parent(), _this() {}

BVNode::BVNode(PBoundingVolume bv, double margin)
: bv(bv->copy()), elems(), children(), _parent(), _this() {
	this->bv->setMargin(margin);
}

BVNode::BVNode(PBoundingVolume bv, PBoundableList elems,
		double margin)
: bv(bv->copy()), elems(elems), children(), _parent(), _this() {
	this->bv->setMargin(margin);
	this->bv->bound(elems);
}

void BVNode::attach(PBVNode pthis) {
	_this = pthis;
	for (typename PBVNodeList::iterator pit = children.begin();
			pit < children.end(); pit++) {
		PBVNode &child = (*pit);
		child->_parent = pthis;
		child->attach(child);
	}
}

PBVNode BVNode::getParent() {
	return _parent.lock();
}

BoundingSphere BVNode::getBoundingSphere() const {
	return bv->getBoundingSphere();
}

const PBoundingVolume BVNode::getBoundingVolume() const {
	return bv;
}

PBoundableList BVNode::getElements() {
	return elems;
}

void BVNode::setElements(const PBoundableList &elems) {
	this->elems = elems;
	bv->bound(elems);
}

size_t BVNode::numElements() const {
	return elems.size();
}

void BVNode::clearElements() {
	elems.clear();
}

PBVNodeList BVNode::getChildren() {
	return children;
}

void BVNode::setChildren(const PBVNodeList &children) {
	this->children = children;
}

size_t BVNode::numChildren() const {
	return children.size();
}

void BVNode::clearChildren() {
	children.clear();
}

void BVNode::clear() {
	clearElements();
	clearChildren();
}

void BVNode::setMargin(double margin) {
	bv->setMargin(margin);
	for (PBVNodeList::iterator pit = children.begin();
			pit < children.end(); pit++) {
		(*pit)->setMargin(margin);
	}
}

double BVNode::getMargin() const {
	return bv->getMargin();
}

bool BVNode::isLeaf() const {
	return (children.size() == 0);
}

bool BVNode::isRoot() const {
	return (_parent.lock() == NULL);
}

bool BVNode::grow() {

	if (elems.size() > 1) {
		// split elements
		PBoundableListList split;
		bv->split(elems, split);

		//add children
		int nonEmpty = 0;
		for (PBoundableListList::iterator blit = split.begin();
				blit < split.end(); blit++) {
			if (blit->size() > 0) {
				children.push_back(spawnChild(*blit));
				Point3d c;
				children.back()->bv->bound(*blit);
				nonEmpty++;
			}
		}

		if (nonEmpty == 1) {
			children.pop_back(); // same as this
		} else {
			clearElements();
		}

	}
}

PBVNode BVNode::spawnChild(const PBoundableList &elems) {
	PBVNode node = BVTreeFactory::createNode(bv, elems,
			getMargin());
	node->attach(node);
	node->_parent = _this;
	return node;
}

bool BVNode::growRecursively() {
	grow();
	for (PBVNodeList::iterator pit = children.begin();
			pit < children.end(); pit++) {
		(*pit)->growRecursively();
	}
}

void BVNode::updateBounds() {
	PBVNodeList nodes;
	updateBoundsDown(nodes);
}

void BVNode::updateBoundsUp(const PBoundable b) {
	b->updateBV(bv);
	PBVNode parent = getParent();
	if (parent == NULL) {
		return;
	}
	parent->updateBoundsUp(b);	// tail recursion
}

void BVNode::updateBoundsDown(PBVNodeList &nodes) {

	if (isLeaf()) {
		for (PBoundableList::iterator bit = elems.begin();
				bit < elems.end(); bit++) {

			(*bit)->updateBV(bv);
			for (PBVNodeList::iterator pit = nodes.begin();
					pit < nodes.end(); pit++) {
				(*bit)->updateBV(((*pit)->bv));
			}

		}

	} else {
		nodes.push_back(_this.lock());
		for (PBVNodeList::iterator nit = children.begin();
				nit < children.end(); nit++ ) {
			(*nit)->updateBoundsDown(nodes);
		}
		nodes.pop_back();
	}
}

// Tree
BVTree::BVTree()
: _root() {}

BVTree::BVTree(const PBoundingVolume bv, double margin)
: _root() {
	_root = BVTreeFactory::createNode(bv, margin);
}

BVTree::BVTree(const PBoundingVolume bv, const PBoundableList &elems,
		double margin)
: _root() {
	build(bv, elems, margin);
}

BVTree::BVTree(PBVNode root) {
	this->_root = _root;
}

PBVNode BVTree::getRoot() {
	return _root;
}

void BVTree::setMargin(double margin) {
	_root->setMargin(margin);
}

double BVTree::getMargin() const {
	return _root->getMargin();
}

void BVTree::build(const PBoundingVolume rootbv,
		const PBoundableList &elems, double margin) {
	_root = BVTreeFactory::createNode(rootbv, elems, margin);
	_root->growRecursively();
}

size_t BVTree::intersectPoint(const Point3d &p, PBVNodeList &out)
const {
	size_t os = out.size();
	intersectPointRecursively(p, out, _root);
	return out.size()-os;
}

size_t BVTree::intersectSphere(const Point3d& c, double r,
		PBVNodeList &out) const {
	size_t os = out.size();
	intersectSphereRecursively(c, r, out, _root);
	return out.size()-os;
}

size_t BVTree::intersectLine(const Point3d &p, const Vector3d &dir,
		PBVNodeList &out) const {
	size_t os = out.size();
	intersectLineRecursively(p, dir, out, _root);
	return out.size()-os;
}

size_t BVTree::intersectRay(const Point3d &p, const Vector3d &dir,
		PBVNodeList &out) const {
	size_t os = out.size();
	intersectRayRecursively(p, dir, out, _root);
	return out.size()-os;
}

size_t BVTree::intersectPlane(const Plane &plane, PBVNodeList &out)
const {
	size_t os = out.size();
	intersectPlaneRecursively(plane, out, _root);
	return out.size()-os;
}

size_t BVTree::intersectBV(const PBoundingVolume bv,
		PBVNodeList &out) const {
	size_t os = out.size();
	intersectBVRecursively(bv.get(), out, _root);
	return out.size()-os;
}

size_t BVTree::intersectBV(const BoundingVolume* bv,
		PBVNodeList &out) const {
	size_t os = out.size();
	intersectBVRecursively(bv, out, _root);
	return out.size()-os;
}

size_t BVTree::intersectTree(const PBVTree tree, PBVNodeList &mine,
		PBVNodeList &hers) const {
	size_t os = mine.size();
	intersectTreeRecursively(_root, tree->_root, mine, hers);
	return mine.size()-os;
}

size_t BVTree::getLeaves(PBVNodeList &leaves) {
	size_t os = leaves.size();
	getLeavesRecursively(leaves, _root);
	return leaves.size()-os;
}

void BVTree::update() {
	_root->updateBounds();
}

void BVTree::intersectPointRecursively(const Point3d &p,
		PBVNodeList &out, const PBVNode node) const {

	if (node->bv->intersectsPoint(p)) {
		if (node->isLeaf()) {
			out.push_back(node);
		} else {
			for (PBVNode child : node->children) {
				intersectPointRecursively(p, out, child);
			}
		}
	}
}

void BVTree::intersectSphereRecursively(const Point3d &c, double r,
		PBVNodeList &out, const PBVNode node) const {

	if (node->bv->intersectsSphere(c, r)) {
		if (node->isLeaf()) {
			out.push_back(node);
		} else {
			for (PBVNode child : node->children) {
				intersectSphereRecursively(c, r, out, child);
			}
		}
	}
}

void BVTree::intersectLineRecursively(const Point3d &p,
		const Vector3d &dir, PBVNodeList &out, const PBVNode node) const {

	if (node->bv->intersectsLine(p, dir)) {
		if (node->isLeaf()) {
			out.push_back(node);
		} else {
			for (PBVNode child : node->children) {
				intersectLineRecursively(p, dir, out, child);
			}
		}
	}
}

void BVTree::intersectRayRecursively(const Point3d &p,
		const Vector3d &dir, PBVNodeList &out, const PBVNode node) const {

	if (node->bv->intersectsRay(p, dir)) {
		if (node->isLeaf()) {
			out.push_back(node);
		} else {
			for (PBVNode child : node->children) {
				intersectRayRecursively(p, dir, out, child);
			}
		}
	}
}

void BVTree::intersectPlaneRecursively(const Plane &plane,
		PBVNodeList &out, const PBVNode node) const {

	if (node->bv->intersectsPlane(plane)) {
		if (node->isLeaf()) {
			out.push_back(node);
		} else {
			for (PBVNode child : node->children) {
				intersectPlaneRecursively(plane, out, child);
			}
		}
	}
}


void BVTree::intersectBVRecursively(const BoundingVolume* bv,
		PBVNodeList &out, const PBVNode node) const {

	if (node->bv->intersects(bv)) {
		if (node->isLeaf()) {
			out.push_back(node);
		} else {
			for (PBVNode child : node->children) {
				intersectBVRecursively(bv, out, child);
			}
		}
	}
}

void BVTree::intersectTreeRecursively(const PBVNode me,
		const PBVNode her, PBVNodeList &mine, PBVNodeList &hers) const {

	//	if (me->bv->intersects(her->bv)) {
	//		printf("yes\n");
	//	} else {
	//		printf("no\n");
	//	}
	if (me->bv->intersects(her->bv)) {
		if (me->isLeaf() && her->isLeaf()) {
			mine.push_back(me);
			hers.push_back(her);
		} else {
			if (me->isLeaf()) {
				for (PBVNode herChild : her->children) {
					intersectTreeRecursively(me, herChild, mine, hers);
				}
			} else if (her->isLeaf()) {
				for (PBVNode myChild : me->children) {
					intersectTreeRecursively(myChild, her, mine, hers);
				}
			} else {

				for (PBVNode myChild : me->children) {
					for (PBVNode herChild : her->children) {
						intersectTreeRecursively(myChild, herChild, mine, hers);
					}
				}

			}
		}
	}
}

void BVTree::intersectBVRecursively(const PBoundingVolume bv,
		PBVNodeList &out, const PBVNode node) const {

	if (bv->intersects(node->bv)) {
		if (node->isLeaf()) {
			out.push_back(node);
		} else {
			for (PBVNode child : node->children) {
				intersectBVRecursively(bv, out, child);
			}
		}
	}
}

double BVTree::getRadius() const {
	return _root->getBoundingSphere().getRadius();
}


void BVTree::getLeavesRecursively(PBVNodeList &leaves,
		const PBVNode node) const {

	if (node->isLeaf()) {
		leaves.push_back(node);
	} else {
		for (PBVNode child : node->children) {
			getLeavesRecursively(leaves, child);
		}
	}
}


// Factory methods

PBoundingSphere BVFactory::createBoundingSphere() {
	return std::make_shared<BoundingSphere>();
}

PBoundingSphere BVFactory::createBoundingSphere(
		const BoundingSphere &bs) {
	return std::make_shared<BoundingSphere>(bs);
}

PAABB BVFactory::createAABB() {
	return std::make_shared<AABB>();
}

PAABB BVFactory::createAABB(const AABB &aabb) {
	return std::make_shared<AABB>(aabb);
}

POBB BVFactory::createOBB() {
	return std::make_shared<OBB>();
}

POBB BVFactory::createOBB(const OBB &obb) {
	return std::make_shared<OBB>(obb);
}


PBVNode BVTreeFactory::createNode(const PBoundingVolume bv, double margin) {
	PBVNode node = std::make_shared<BVNode>(bv, margin);
	node->attach(node);
}

PBVNode BVTreeFactory::createNode(const PBoundingVolume bv,
		const PBoundableList &elems, double margin) {
	PBVNode node = std::make_shared<BVNode>(bv, elems, margin);
	node->attach(node);
	return node;
}

PBVTree BVTreeFactory::createTree(const PBoundingVolume bv,
		double margin) {
	PBVTree tree = std::make_shared<BVTree>(bv, margin);
	return tree;
}

PBVTree BVTreeFactory::createTree(const PBoundingVolume bv,
		const PBoundableList &elems, double margin) {

	PBVTree tree = std::make_shared<BVTree>(bv, elems, margin);
	return tree;
}

/* Static routines */
class BVNodeDistance {
public:
	PBVNode node;
	Point3d nearest;
	double dist;
public:
	BVNodeDistance(const BVNodeDistance &copyMe)
: node(copyMe.node), nearest(copyMe.nearest),
  dist(copyMe.dist) {}

	BVNodeDistance(const PBVNode node, const Point3d &nearest,
			double dist)
	: node(node), nearest(nearest), dist(dist) {}

	BVNodeDistance(const PBVNode node, const Point3d &pnt) {
		this->node = node;
		this->dist = node->bv->distanceToPoint(pnt, nearest);
	}

	BVNodeDistance(const PBVNode node, const Point3d &pnt,
			const Vector3d &dir) {
		this->node = node;
		this->dist = node->bv->distanceToPoint(pnt, dir, nearest);
	}

};

class BVNodeDistanceComparator {
public:
	// Returns true if b1 is earlier than b2
	bool operator()(BVNodeDistance& b1, BVNodeDistance& b2) {
		if (b1.dist < b2.dist) {
			return true;
		}
		return false;
	}
};


PBoundable nearest_boundable(const PBVTree bvh, const Point3d &pnt,
		Point3d &nearestPoint) {

	PBoundable nearest = NULL;
	Point3d p;

	double dist = math::DOUBLE_INFINITY;

	std::priority_queue<BVNodeDistance, std::vector<BVNodeDistance>,
	BVNodeDistanceComparator> queue;

	queue.push( BVNodeDistance(bvh->getRoot(),pnt) );
	while (queue.size() > 0) {

		BVNodeDistance bvd = queue.top();
		queue.pop();
		if (bvd.dist > dist) {
			// remaining nodes are further
			break;
		}

		PBVNode node = bvd.node;
		if (node->isLeaf()) {
			for (PBoundable elem : node->elems) {

				double d = elem->distanceToPoint(pnt, p);
				if (d < dist) {
					dist = d;
					nearest = elem;
					nearestPoint = p;
				}
			}
		} else {

			// add children to queue
			for (PBVNode child : node->children) {
				double d = child->bv->distanceToPoint(pnt, p);
				if (d <= dist) {
					queue.push (BVNodeDistance(child, p, d));
				}
			}
		}
	}

	return nearest;

}

PBoundable nearest_boundable(const PBVTree bvh, const Point3d &pnt,
		const Vector3d &dir, Point3d &nearestPoint) {

	PBoundable nearest = NULL;
	Point3d p;
	double dist = math::DOUBLE_INFINITY;

	std::priority_queue<BVNodeDistance, std::vector<BVNodeDistance>,
	BVNodeDistanceComparator> queue;

	queue.push( BVNodeDistance(bvh->getRoot(), pnt, dir) );
	while (queue.size() > 0) {

		BVNodeDistance bvd = queue.top();
		queue.pop();
		if (bvd.dist > dist) {
			// remaining nodes are further
			break;
		}

		PBVNode node = bvd.node;
		if (node->isLeaf()) {
			for (PBoundable elem : node->elems) {
				double d = elem->distanceToPoint(pnt, dir, p);
				if (d < dist) {
					// clear current list of nearest
					dist = d;
					nearest = elem;
					nearestPoint.set(p);
				}
			}
		} else {

			// add children to queue
			for (PBVNode child : node->children) {
				double d = child->bv->distanceToPoint(pnt, dir, p);
				if (d <= dist) {
					queue.push (BVNodeDistance(child, p, d));
				}
			}
		}
	}

	return nearest;
}

/*


PBoundable nearest_boundable(const PBVTree bvh, const Point3d &pnt,
		double tol, NearestBoundableData &data) {

	PBoundable nearest = NULL;
	Point3d p;

	data.dist = math::DOUBLE_INFINITY;
	data.tol = tol;
	data.nearestBoundables.clear();
	data.nearestPoints.clear();

	std::priority_queue<BVNodeDistance, std::vector<BVNodeDistance>,
	BVNodeDistanceComparator> queue;

	queue.push( BVNodeDistance(bvh->getRoot(),pnt) );
	while (queue.size() > 0) {

		BVNodeDistance bvd = queue.top();
		queue.pop();
		if (bvd.dist > data.dist+tol) {
			// remaining nodes are further
			break;
		}

		PBVNode node = bvd.node;
		if (node->isLeaf()) {

			PBoundableList elems = node->elems;
			for (PBoundable elem : node->elems) {

				double d = elem->distanceToPoint(pnt, p);
				if (d < data.dist-tol) {
					// clear current list of nearest
					data.dist = d;
					nearest = elem;
					data.nearestBoundables.clear();
					data.nearestBoundables.push_back(nearest);
					data.nearestPoints.clear();
					data.nearestPoints.push_back(p);
				} else if (d < data.dist) {
					// add new nearest to front of list
					data.dist = d;
					data.nearestPoints.insert(data.nearestPoints.begin(), p);
					data.nearestBoundables.insert(data.nearestBoundables.begin(), elem);
				} else if (d <= data.dist+tol) {
					// add new nearest to back of list
					data.nearestBoundables.push_back(elem);
					data.nearestPoints.push_back(p);
				}
			}
		} else {

			// add children to queue
			for (PBVNode child : node->children) {
				double d = child->bv->distanceToPoint(pnt, p);
				if (d <= data.dist+tol) {
					queue.push (BVNodeDistance(child, p, d));
				}
			}
		}
	}

	return nearest;

}
PBoundable nearest_boundable(const PBVTree bvh, const Point3d &pnt,
		const Vector3d &dir, double tol, NearestBoundableData &data) {

	PBoundable nearest = NULL;
	Point3d p;

	data.dist = math::DOUBLE_INFINITY;
	data.tol = tol;
	data.nearestBoundables.clear();
	data.nearestPoints.clear();

	std::priority_queue<BVNodeDistance, std::vector<BVNodeDistance>,
	BVNodeDistanceComparator> queue;

	queue.push( BVNodeDistance(bvh->getRoot(), pnt, dir) );
	while (queue.size() > 0) {

		BVNodeDistance bvd = queue.top();
		queue.pop();
		if (bvd.dist > data.dist+tol) {
			// remaining nodes are further
			break;
		}

		PBVNode node = bvd.node;
		if (node->isLeaf()) {
			for (PBoundable elem : node->elems) {
				double d = elem->distanceToPoint(pnt, dir, p);
				if (d < data.dist-tol) {
					// clear current list of nearest
					data.dist = d;
					nearest = elem;
					data.nearestBoundables.clear();
					data.nearestBoundables.push_back(nearest);
					data.nearestPoints.clear();
					data.nearestPoints.push_back(p);
				} else if (d < data.dist) {
					// add new nearest to front of list
					data.dist = d;
					data.nearestPoints.insert(data.nearestPoints.begin(), p);
					data.nearestBoundables.insert(data.nearestBoundables.begin(), elem);
				} else if (d <= data.dist+tol) {
					// add new nearest to back of list
					data.nearestBoundables.push_back(elem);
					data.nearestPoints.push_back(p);
				}

			}
		} else {

			// add children to queue
			for (PBVNode child : node->children) {
				double d = child->bv->distanceToPoint(pnt, dir, p);
				if (d <= data.dist+tol) {
					queue.push (BVNodeDistance(child, p, d));
				}
			}
		}
	}

	return nearest;

}
 */

}
}
