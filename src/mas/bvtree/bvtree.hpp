#ifndef MAS_BVTREE_HPP
#define MAS_BVTREE_HPP

#include <algorithm>
#include "mas/core/math.h"

// template implementation
namespace mas {
namespace bvtree {

template<typename Point3d>
BoundablePointSet<Point3d>::BoundablePointSet(size_t idx) :
        idx(idx), pnts() {
}

template<typename Point3d>
BoundablePointSet<Point3d>::BoundablePointSet(const std::vector<Point3d>& pnts,
        size_t idx) :
        idx(idx), pnts(pnts) {
}

template<typename Point3d>
BoundablePointSet<Point3d>::BoundablePointSet(std::vector<Point3d>&& pnts,
        size_t idx) :
        idx(idx), pnts(std::move(pnts)) {
}

template<typename Point3d>
size_t BoundablePointSet<Point3d>::getIndex() const {
    return idx;
}

template<typename Point3d>
void BoundablePointSet<Point3d>::setIndex(size_t idx) {
    this->idx = idx;
}

template<typename Point3d>
void BoundablePointSet<Point3d>::setPoints(const std::vector<Point3d>& pnts) {
    this->pnts = pnts;
}

template<typename Point3d>
void BoundablePointSet<Point3d>::setPoints(std::vector<Point3d>&& pnts) {
    this->pnts = std::move(pnts);
}

template<typename Point3d>
void BoundablePointSet<Point3d>::addPoint(const Point3d& pnt) {
    pnts.push_back(pnt);
}

template<typename Point3d>
void BoundablePointSet<Point3d>::addPoint(Point3d&& pnt) {
    pnts.push_back(std::move(pnt));
}

template<typename Point3d>
template<typename BV>
bool BoundablePointSet<Point3d>::updateBV(BV& bv) const {

    bool updated = false;
    for (const Point3d& pnt : pnts) {
        updated |= bv.updatePoint(pnt);
    }

    return updated;
}

template<typename Point3d>
void BoundablePointSet<Point3d>::getCentroid(Point3d& c) const {

    c.setZero();
    for (const Point3d& pnt : pnts) {
        c.add(pnt);
    }
    c.scale(1.0 / pnts.size());

}

template<typename Point3d>
void BoundablePointSet<Point3d>::getCovariance(const Point3d& centre,
        Matrix3d& cov) const {

    cov.setZero();
    Point3d diff;
    for (const Point3d& pnt : pnts) {
        diff = pnt;
        diff.subtract(centre);
        cov.addOuterProduct(diff, diff);
    }
    cov.scale(1.0 / pnts.size());
}

// closest point
template<typename Point3d>
double BoundablePointSet<Point3d>::distanceToPoint(const Point3d& pnt,
        Point3d& nearest) const {

    double dmin = mas::math::DOUBLE_INFINITY;

    for (const Point3d& mpnt : pnts) {
        double d = pnt.distance(mpnt);
        if (d < dmin) {
            dmin = d;
            nearest = mpnt;
        }
    }

    return dmin;

}

/**
 * @return mas::math::DOUBLE_INFINITY
 */
template<typename Point3d>
double BoundablePointSet<Point3d>::distanceToPoint(const Point3d& pnt,
        const Vector3d& dir, Point3d& nearest) const {

    double dmin = math::DOUBLE_INFINITY;

    //   // Computes closest in direction
    //   double q0 = pnt.dot(pnt);
    //   double q1 = 2*pnt.dot(dir);
    //   double q2 = dir.dot(dir);
    //
    //  Vector3d diff;
    //  for (Point3d& mpnt : pnts) {
    //      diff.subtract(mpnt, pnt);
    //      double d = diff.dot(dir)/q2;
    //      d = sqrt(q0+d*q1+q2);
    //
    //      if (d < dmin) {
    //          dmin = d;
    //          nearest = mpnt;
    //      }
    //  }

    return dmin;
}

template<typename PointPtr>
BoundablePointPtrSet<PointPtr>::BoundablePointPtrSet(size_t idx) :
        idx(idx), pnts() {
}

template<typename PointPtr>
BoundablePointPtrSet<PointPtr>::BoundablePointPtrSet(
        const std::vector<PointPtr>& pnts, size_t idx) :
        idx(idx), pnts(pnts) {
}

template<typename PointPtr>
BoundablePointPtrSet<PointPtr>::BoundablePointPtrSet(
        std::vector<PointPtr>&& pnts, size_t idx) :
        idx(idx), pnts(std::move(pnts)) {
}

template<typename PointPtr>
size_t BoundablePointPtrSet<PointPtr>::getIndex() const {
    return idx;
}

template<typename PointPtr>
void BoundablePointPtrSet<PointPtr>::setIndex(size_t idx) {
    this->idx = idx;
}

template<typename PointPtr>
void BoundablePointPtrSet<PointPtr>::setPoints(
        const std::vector<PointPtr>& pnts) {
    this->pnts = pnts;
}

template<typename PointPtr>
void BoundablePointPtrSet<PointPtr>::setPoints(std::vector<PointPtr>&& pnts) {
    this->pnts = std::move(pnts);
}

template<typename PointPtr>
void BoundablePointPtrSet<PointPtr>::addPoint(const PointPtr& pnt) {
    pnts.push_back(pnt);
}

template<typename PointPtr>
void BoundablePointPtrSet<PointPtr>::addPoint(PointPtr&& pnt) {
    pnts.push_back(std::move(pnt));
}

template<typename PointPtr>
template<typename BV>
bool BoundablePointPtrSet<PointPtr>::updateBV(BV& bv) const {

    bool updated = false;
    for (const PointPtr& pnt : pnts) {
        updated |= bv.updatePoint(*pnt);
    }

    return updated;
}

template<typename PointPtr>
void BoundablePointPtrSet<PointPtr>::getCentroid(Point3d& c) const {

    c.setZero();
    for (const PointPtr& pnt : pnts) {
        c.add(*pnt);
    }
    c.scale(1.0 / pnts.size());

}

template<typename PointPtr>
void BoundablePointPtrSet<PointPtr>::getCovariance(const Point3d& centre,
        Matrix3d& cov) const {

    cov.setZero();
    Point3d diff;
    for (const PointPtr& pnt : pnts) {
        diff = *pnt;
        diff.subtract(centre);
        cov.addOuterProduct(diff, diff);
    }
    cov.scale(1.0 / pnts.size());
}

// closest point
template<typename PointPtr>
double BoundablePointPtrSet<PointPtr>::distanceToPoint(const Point3d& pnt,
        Point3d& nearest) const {

    double dmin = math::DOUBLE_INFINITY;

    for (const PointPtr& mpnt : pnts) {
        double d = mpnt->distance(pnt);
        if (d < dmin) {
            dmin = d;
            nearest = *mpnt;
        }
    }

    return dmin;

}

/**
 * @return mas::math::DOUBLE_INFINITY
 */
template<typename PointPtr>
double BoundablePointPtrSet<PointPtr>::distanceToPoint(const Point3d& pnt,
        const Vector3d& dir, Point3d& nearest) const {

    double dmin = math::DOUBLE_INFINITY;

    //   // Computes closest in direction
    //   double q0 = pnt.dot(pnt);
    //   double q1 = 2*pnt.dot(dir);
    //   double q2 = dir.dot(dir);
    //
    //  Vector3d diff;
    //  for (Point3d& mpnt : pnts) {
    //      diff.subtract(mpnt, pnt);
    //      double d = diff.dot(dir)/q2;
    //      d = sqrt(q0+d*q1+q2);
    //
    //      if (d < dmin) {
    //          dmin = d;
    //          nearest = mpnt;
    //      }
    //  }

    return dmin;

}

template<typename BoundablePtr>
bool BoundingVolume::update(const BoundablePtr& b) {
    // I don't know any boundables, so pass off to them
    return b->updateBV(*this);
}

template<typename BV>
bool BoundingSphere::intersects(const BV& bv) const {
    return bv.intersectsSphere(c, r);
}

template<typename BoundablePtr>
void BoundingSphere::bound(const std::vector<BoundablePtr>& blist) {

    // get total centroid, then compute radius
    Point3d centroid;
    c.setZero();

    // set center as average point
    for (const BoundablePtr& sb : blist) {
        sb->getCentroid(centroid);
        c.add(centroid);
    }
    c.scale(1.0 / blist.size());

    r = 0;
    // update radius
    for (const BoundablePtr& sb : blist) {
        sb->updateBV(*this);
    }

}

#define OCTANT_SIGN(a) (a >= 0 ? 1 : 0)
#define OCTANT(a,o) (OCTANT_SIGN(a.x-o.x)+2*OCTANT_SIGN(a.y-o.y)\
        +4*OCTANT_SIGN(a.z-o.z))

// copy shared boundables
template<typename BoundablePtr>
bool BoundingSphere::split(const std::vector<BoundablePtr>& b,
        std::vector<std::vector<BoundablePtr>>& out) const {

    // split by octree
    out = std::vector<std::vector<BoundablePtr>>(8,
            std::vector<BoundablePtr>());
    int outSizes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    Point3d centroid;
    for (const BoundablePtr& sb : b) {
        // separate into quadrants
        sb->getCentroid(centroid);
        int octant = OCTANT(centroid, c);
        out[octant].push_back(sb);
        outSizes[octant]++;
    }

    return true;
}

// move shared boundables
template<typename BoundablePtr>
bool BoundingSphere::split(std::vector<BoundablePtr>&& b,
        std::vector<std::vector<BoundablePtr>>& out) const {

    // split by octree
    out = std::vector<std::vector<BoundablePtr>>(8,
            std::vector<BoundablePtr>());
    int outSizes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    Point3d centroid;
    for (BoundablePtr& sb : b) {
        // separate into quadrants
        sb->getCentroid(centroid);
        int octant = OCTANT(centroid, c);
        out[octant].push_back(std::move(sb));
        outSizes[octant]++;
    }

    return true;
}

// generic implementation
template<typename BV>
bool AABB::intersects(const BV& bv) const {

   // get sphere:
   Point3d c;
   double r = bv.getBoundingSphere(c);
   return intersectsSphere(c, r);

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

template<typename BoundablePtr>
void AABB::bound(const std::vector<BoundablePtr>& b) {

    // get total centroid, then compute radius
    Point3d centroid;
    c.setZero();

    // set center as average point
    for (const BoundablePtr& sb : b) {
        sb->getCentroid(centroid);
        c.add(centroid);
    }
    c.scale(1.0 / b.size());

    halfWidths.setZero();
    // update bounds
    for (const BoundablePtr& sb : b) {
        sb->updateBV(*this);
    }

}

// copy shared boundables
template<typename BoundablePtr>
bool AABB::split(const std::vector<BoundablePtr>& b,
        std::vector<std::vector<BoundablePtr>>& out) const {

    // split box
    const Vector3d* axes[] = { &Vector3d::X_AXIS, &Vector3d::Y_AXIS,
            &Vector3d::Z_AXIS };

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

    for (int i = 0; i < 3; i++) {

        const Vector3d& normal = *axes[i];
        double d = -normal.dot(c);

        out = std::vector<std::vector<BoundablePtr> >(2,
                std::vector<BoundablePtr>());
        Point3d centroid;

        for (const BoundablePtr& sb : b) {
            sb->getCentroid(centroid);
            if (centroid.dot(normal) + d < 0) {
                out[0].push_back(sb);
            } else {
                out[1].push_back(sb);
            }
        }

        if (out[0].size() > 0 && out[1].size() > 0) {
            return true;
        }
    }

    // all splitting failed, put all in one
    out = std::vector<std::vector<BoundablePtr> >(1, b);
    return false;
}

// copy shared boundables
template<typename BoundablePtr>
bool AABB::split(std::vector<BoundablePtr>&& b,
        std::vector<std::vector<BoundablePtr>>& out) const {

    // split box
    const Vector3d* axes[] = { &Vector3d::X_AXIS, &Vector3d::Y_AXIS,
            &Vector3d::Z_AXIS };

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

    for (int i = 0; i < 3; i++) {

        const Vector3d& normal = *axes[i];
        double d = -normal.dot(c);

        out = std::vector<std::vector<BoundablePtr> >(2,
                std::vector<BoundablePtr>());
        Point3d centroid;

        for (BoundablePtr& sb : b) {
            sb->getCentroid(centroid);
            if (centroid.dot(normal) + d < 0) {
                out[0].push_back(std::move(sb));
            } else {
                out[1].push_back(std::move(sb));
            }
        }

        if (out[0].size() > 0 && out[1].size() > 0) {
            return true;
        } else {
            if (out[0].size() > 0) {
                b = std::move(out[0]);
            } else {
                b = std::move(out[1]);
            }
        }
    }

    // all splitting failed, put all in one
    out = std::vector<std::vector<BoundablePtr> >(1, std::move(b));
    return false;
}

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

template<typename BV>
bool OBB::intersects(const BV& bv) const {

   Point3d c;
   double r = bv.getBoundingSphere(c);
   return intersectsSphere(c, r);
}

template<typename BoundablePtr>
void OBB::bound(const std::vector<BoundablePtr>& b) {

    c.setZero();

    // set center as average point
    Point3d centroid;
    for (const BoundablePtr& sb : b) {
        sb->getCentroid(centroid);
        c.add(centroid);
    }
    c.scale(1.0 / b.size());

    // compute tight-fitting axes via SVD
    Matrix3d cov = Matrix3d(0, 0, 0, 0, 0, 0, 0, 0, 0);
    Matrix3d covl;

    // covariance... technically, according to paper,
    // should be using surface area of convex hull.  However,
    // this doesn't generalize well and expects all objects
    // to be polygons
    for (const BoundablePtr& sb : b) {
        sb->getCovariance(c, covl);
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

    this->R = U;

    halfWidths.setZero();
    // update bounds
    for (const BoundablePtr& sb : b) {
        sb->updateBV(*this);
    }

}

// copy shared boundables
template<typename BoundablePtr>
bool OBB::split(const std::vector<BoundablePtr>& b,
        std::vector<std::vector<BoundablePtr>>& out) const {

    // division plane given by normal.dot(x) + d = 0
    Vector3d normal;
    Point3d centroid;

    int outSizes[] = { 0, 0 };
    int nElems = b.size();

    // start with largest axis, if fails continue
    // to second largest
    for (int i = 0; i < 3; i++) {
        R.getColumn(i, normal);
        double d = -normal.dot(c);

        // split along first dimension
        out = std::vector<std::vector<BoundablePtr> >(2,
                std::vector<BoundablePtr>());
        for (const BoundablePtr& sb : b) {
            sb->getCentroid(centroid);
            if (normal.dot(centroid) + d < 0) {
                out[0].push_back(sb);
            } else {
                out[1].push_back(sb);
            }
        }
        outSizes[0] = out[0].size();
        outSizes[1] = out[1].size();
        if (out[0].size() > 0 && out[1].size() > 0) {
            return true;
        }
    }

    out = std::vector<std::vector<BoundablePtr> >(1, b);

    return false;
}

// copy shared boundables
template<typename BoundablePtr>
bool OBB::split(std::vector<BoundablePtr>&& b,
        std::vector<std::vector<BoundablePtr>>& out) const {

    // division plane given by normal.dot(x) + d = 0
    Vector3d normal;
    Point3d centroid;

    // start with largest axis, if fails continue
    // to second largest
    for (int i = 0; i < 3; i++) {
        R.getColumn(i, normal);
        double d = -normal.dot(c);

        // split along first dimension
        out = std::vector<std::vector<BoundablePtr> >(2,
                std::vector<BoundablePtr>());
        for (const BoundablePtr& sb : b) {
            sb->getCentroid(centroid);
            if (normal.dot(centroid) + d < 0) {
                out[0].push_back(std::move(sb));
            } else {
                out[1].push_back(std::move(sb));
            }
        }
        if (out[0].size() > 0 && out[1].size() > 0) {
            return true;
        } else {
            if (out[0].size() > 0) {
                b = std::move(out[0]);
            } else {
                b = std::move(out[1]);
            }
        }
    }

    out = std::vector<std::vector<BoundablePtr> >(1, std::move(b));

    return false;
}

// BV Node
template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>::BVNode() :
        bv(new BV()), elems(), children(), parent() {
}

template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>::BVNode(double margin) :
        bv(new BV(margin)), elems(), children(), parent() {
}

template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>::BVNode(const std::vector<BoundablePtr>& elems,
        double margin) :
        parent(), bv(new BV()), elems(elems), children() {
    bv->setMargin(margin);
    bv->bound(this->elems);
}

template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>::BVNode(std::vector<BoundablePtr>&& elems,
        double margin) :
        parent(), bv(new BV()), elems(std::move(elems)), children() {
    bv->setMargin(margin);
    bv->bound(this->elems);
}

template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>* BVNode<BoundablePtr, BV>::getParent() {
    return parent;
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::setParent(BVNode<BoundablePtr, BV>* p) {
    parent = p;
}

template<typename BoundablePtr, typename BV>
BoundingSphere BVNode<BoundablePtr, BV>::getBoundingSphere() const {
    return bv->getBoundingSphere();
}

template<typename BoundablePtr, typename BV>
double BVNode<BoundablePtr, BV>::getBoundingSphere(Point3d& centre) const {
    return bv->getBoundingSphere(centre);
}

template<typename BoundablePtr, typename BV>
const BV& BVNode<BoundablePtr, BV>::getBoundingVolume() const {
    return *bv;
}

template<typename BoundablePtr, typename BV>
std::vector<BoundablePtr>& BVNode<BoundablePtr, BV>::getElements() const {
    return elems;
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::setElements(
        const std::vector<BoundablePtr>& elems) {
    this->elems = elems;
    bv->bound(elems);
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::setElements(std::vector<BoundablePtr>&& elems) {
    this->elems = std::move(elems);
    bv->bound(this->elems);
}

template<typename BoundablePtr, typename BV>
size_t BVNode<BoundablePtr, BV>::numElements() const {
    return elems.size();
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::clearElements() {
    elems.clear();
}

template<typename BoundablePtr, typename BV>
std::vector<std::shared_ptr<BVNode<BoundablePtr,BV>>>& BVNode<BoundablePtr, BV>::getChildren() {
    return children;
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::setChildren(
        const std::vector<std::shared_ptr<BVNode<BoundablePtr, BV>>>& children) {
    this->children = children;
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::setChildren(
        std::vector<std::shared_ptr<BVNode<BoundablePtr, BV>>>&& children) {
    this->children = std::move(children);
}

template<typename BoundablePtr, typename BV>
size_t BVNode<BoundablePtr, BV>::numChildren() const {
    return children.size();
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::clearChildren() {
    children.clear();
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::clear() {
    clearElements();
    clearChildren();
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::setMargin(double margin) {
    bv->setMargin(margin);
    for (SharedBVNode &child : children) {
        child->setMargin(margin);
    }
}

template<typename BoundablePtr, typename BV>
double BVNode<BoundablePtr, BV>::getMargin() const {
    return bv->getMargin();
}

template<typename BoundablePtr, typename BV>
bool BVNode<BoundablePtr, BV>::isLeaf() const {
    return (children.size() == 0);
}

template<typename BoundablePtr, typename BV>
bool BVNode<BoundablePtr, BV>::isRoot() const {
    return (parent == nullptr);
}

template<typename BoundablePtr, typename BV>
bool BVNode<BoundablePtr, BV>::grow() {

    if (elems.size() > 1) {

        // split elements
        std::vector < std::vector<BoundablePtr> > split;
        bv->split(std::move(elems), split);

        //add children
        int nonEmpty = 0;
        for (std::vector<BoundablePtr>& subelems : split) {
            if (subelems.size() > 0) {
                children.push_back(
                        std::shared_ptr<BVNode<BoundablePtr, BV>>(
                                spawnChild(std::move(subelems))));
                nonEmpty++;
            }
        }

        if (nonEmpty == 1) {
            // recover elements and remove children
            elems = std::move(children[0]->elems);
            children.clear();
            return false;
        } else {
            // remove elements, now children contain them
            clearElements();
            return true;
        }

    }
    return false;
}

template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>* BVNode<BoundablePtr, BV>::spawnChild(
        const std::vector<BoundablePtr>& elems) {
    BVNode < BoundablePtr, BV > *node = new BVNode<BoundablePtr, BV>(elems,
            getMargin());
    node->setParent(this);
    return node;
}

template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>* BVNode<BoundablePtr, BV>::spawnChild(
        std::vector<BoundablePtr>&& elems) {
    BVNode < BoundablePtr, BV > *node = new BVNode<BoundablePtr, BV>(
            std::move(elems), getMargin());
    node->setParent(this);
    return node;
}

template<typename BoundablePtr, typename BV>
bool BVNode<BoundablePtr, BV>::growRecursively() {
    if (grow()) {
        for (SharedBVNode& child : children) {
            child->growRecursively();
        }
        return true;
    }
    return false;
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::updateBounds() {
    if (isLeaf()) {
        // update bounds up from leaf nodes
        for (BoundablePtr& sb : elems) {
            updateBoundsUp(*sb);
        }
    } else {

        // continue moving down the rabbit hole
        for (SharedBVNode& child : children) {
            child->updateBounds();
        }

    }
}

template<typename BoundablePtr, typename BV>
void BVNode<BoundablePtr, BV>::updateBoundsUp(const BoundablePtr& b) {
    b->updateBV(*bv);
    BVNode<BoundablePtr,BV>* parent = getParent();
    if (parent == nullptr) {
        return;
    }
    parent->updateBoundsUp(b);  // tail recursion
}

// Tree
template<typename BoundablePtr, typename BV>
BVTree<BoundablePtr, BV>::BVTree(double margin) {
    root = std::make_shared <BVNode<BoundablePtr, BV> >(margin);
}

template<typename BoundablePtr, typename BV>
BVTree<BoundablePtr, BV>::BVTree(const std::vector<BoundablePtr>& elems,
        double margin) :
        root(nullptr) {
    build(elems, margin);
}

template<typename BoundablePtr, typename BV>
BVTree<BoundablePtr, BV>::BVTree(std::vector<BoundablePtr>&& elems,
        double margin) :
        root(nullptr) {
    build(std::move(elems), margin);
}

template<typename BoundablePtr, typename BV>
BVNode<BoundablePtr, BV>& BVTree<BoundablePtr, BV>::getRoot() const {
    return *root;
}

template<typename BoundablePtr, typename BV>
double BVTree<BoundablePtr, BV>::getRadius() const {
    return root->getBoundingSphere().getRadius();
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::setMargin(double margin) {
    root->setMargin(margin);
}

template<typename BoundablePtr, typename BV>
double BVTree<BoundablePtr, BV>::getMargin() const {
    return root->getMargin();
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::build(const std::vector<BoundablePtr>& elems,
        double margin) {
    root = std::make_shared<BVNode<BoundablePtr, BV>>(elems, margin);
    root->growRecursively();
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::build(std::vector<BoundablePtr>&& elems, double margin) {
    root = std::make_shared<BVNodeType> (std::move(elems), margin);
    root->growRecursively();
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectPoint(const Point3d& p,
        std::vector<std::shared_ptr<BVNodeType>>& out) const {
    size_t os = out.size();
    intersectPointRecursively(p, out, root);
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectPoint(const Point3d& p,
        std::vector<BVNodeType*>& out) const {
    size_t os = out.size();
    intersectPointRecursively(p, out, root.get());
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectSphere(const Point3d& c, double r,
        std::vector<std::shared_ptr<BVNodeType>>& out) const {
    size_t os = out.size();
    intersectSphereRecursively(c, r, out, root);
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectSphere(const Point3d& c, double r,
        std::vector<BVNodeType*>& out) const {
    size_t os = out.size();
    intersectSphereRecursively(c, r, out, root.get());
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectLine(const Point3d& p, const Vector3d& dir,
        std::vector<std::shared_ptr<BVNodeType>>& out) const {
    size_t os = out.size();
    intersectLineRecursively(p, dir, out, root);
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectLine(const Point3d& p, const Vector3d& dir,
        std::vector<BVNodeType*>& out) const {
    size_t os = out.size();
    intersectLineRecursively(p, dir, out, root.get());
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectRay(const Point3d& p,
        const Vector3d& dir,
        std::vector<std::shared_ptr<BVNodeType>>& out) const {
    size_t os = out.size();
    intersectRayRecursively(p, dir, out, root);
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectRay(const Point3d& p,
        const Vector3d& dir,
        std::vector<BVNodeType*>& out) const {
    size_t os = out.size();
    intersectRayRecursively(p, dir, out, root.get());
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectPlane(const Plane& plane,
        std::vector<std::shared_ptr<BVNodeType>>& out) const {
    size_t os = out.size();
    intersectPlaneRecursively(plane, out, root);
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::intersectPlane(const Plane& plane,
        std::vector<BVNodeType*>& out) const {
    size_t os = out.size();
    intersectPlaneRecursively(plane, out, root.get());
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
template<typename BV2>
size_t BVTree<BoundablePtr, BV>::intersectBV(const BV2& bv,
        std::vector<std::shared_ptr<BVNodeType>>& out) const {
    size_t os = out.size();
    intersectBVRecursively(bv, out, root);
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
template<typename BV2>
size_t BVTree<BoundablePtr, BV>::intersectBV(const BV2& bv,
        std::vector<BVNodeType*>& out) const {
    size_t os = out.size();
    intersectBVRecursively(bv, out, root.get());
    return out.size() - os;
}

template<typename BoundablePtr, typename BV>
template<typename BoundablePtr2, typename BV2>
size_t BVTree<BoundablePtr, BV>::intersectTree(
        const BVTree<BoundablePtr2, BV2>& tree,
        std::vector<std::shared_ptr<BVNodeType>>& mine,
        std::vector<std::shared_ptr<BVNode<BoundablePtr2,BV2>>>& hers) const {
    size_t os = mine.size();
    intersectTreeRecursively(root, tree.root, mine, hers);
    return mine.size() - os;
}

template<typename BoundablePtr, typename BV>
template<typename BoundablePtr2, typename BV2>
size_t BVTree<BoundablePtr, BV>::intersectTree(
        const BVTree<BoundablePtr2, BV2>& tree,
        std::vector<BVNodeType*>& mine,
        std::vector<BVNode<BoundablePtr2,BV2>*>& hers) const {
    size_t os = mine.size();
    intersectTreeRecursively(root.get(), tree.root.get(), mine, hers);
    return mine.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::getLeaves(
        std::vector<std::shared_ptr<BVNodeType>>& leaves) {
    size_t os = leaves.size();
    getLeavesRecursively(leaves, root);
    return leaves.size() - os;
}

template<typename BoundablePtr, typename BV>
size_t BVTree<BoundablePtr, BV>::getLeaves(
        std::vector<BVNodeType*>& leaves) {
    size_t os = leaves.size();
    getLeavesRecursively(leaves, root.get());
    return leaves.size() - os;
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::update() {
    root->updateBounds();
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectPointRecursively(const Point3d& p,
        std::vector<std::shared_ptr<BVNodeType>>& out,
        const std::shared_ptr<BVNodeType>& node) const {

    if (node->bv->intersectsPoint(p)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (std::shared_ptr<BVNodeType>& child : node->children) {
                intersectPointRecursively(p, out, child);
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectPointRecursively(const Point3d& p,
        std::vector<BVNodeType*>& out, BVNodeType* node) const {

    if (node->bv->intersectsPoint(p)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectPointRecursively(p, out, child.get());
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectSphereRecursively(const Point3d& c,
        double r, std::vector<std::shared_ptr<BVNodeType>>& out,
        const std::shared_ptr<BVNodeType>& node) const {

    if (node->bv->intersectsSphere(c, r)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectSphereRecursively(c, r, out, child);
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectSphereRecursively(const Point3d& c,
        double r, std::vector<BVNodeType*>& out, BVNodeType* node) const {

    if (node->bv->intersectsSphere(c, r)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectSphereRecursively(c, r, out, child.get());
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectLineRecursively(const Point3d& p,
        const Vector3d& dir, std::vector<std::shared_ptr<BVNodeType>>& out,
        const std::shared_ptr<BVNodeType>& node) const {

    if (node->bv->intersectsLine(p, dir)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectLineRecursively(p, dir, out, child);
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectLineRecursively(const Point3d& p,
        const Vector3d& dir, std::vector<BVNodeType*>& out, BVNodeType* node) const {

    if (node->bv->intersectsLine(p, dir)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectLineRecursively(p, dir, out, child.get());
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectRayRecursively(const Point3d& p,
        const Vector3d& dir, std::vector<std::shared_ptr<BVNodeType>>& out,
        const std::shared_ptr<BVNodeType>& node) const {

    if (node->bv->intersectsRay(p, dir)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectRayRecursively(p, dir, out, child);
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectRayRecursively(const Point3d& p,
        const Vector3d& dir, std::vector<BVNodeType*>& out, BVNodeType* node) const {

    if (node->bv->intersectsRay(p, dir)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectRayRecursively(p, dir, out, child.get());
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectPlaneRecursively(const Plane& plane,
        std::vector<std::shared_ptr<BVNodeType>>& out,
        const std::shared_ptr<BVNodeType>& node) const {

    if (node->bv->intersectsPlane(plane)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectPlaneRecursively(plane, out, child);
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::intersectPlaneRecursively(const Plane& plane,
        std::vector<BVNodeType*>& out, BVNodeType* node) const {

    if (node->bv->intersectsPlane(plane)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectPlaneRecursively(plane, out, child.get());
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
template<typename BV2>
void BVTree<BoundablePtr, BV>::intersectBVRecursively(const BV2& bv,
        std::vector<std::shared_ptr<BVNodeType>>& out,
        const std::shared_ptr<BVNodeType>& node) const {

    if (node->bv->intersects(bv)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectBVRecursively(bv, out, child);
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
template<typename BV2>
void BVTree<BoundablePtr, BV>::intersectBVRecursively(const BV2& bv,
        std::vector<BVNodeType*>& out, BVNodeType* node) const {

    if (node->bv->intersects(bv)) {
        if (node->isLeaf()) {
            out.push_back(node);
        } else {
            for (const std::shared_ptr<BVNodeType>& child : node->children) {
                intersectBVRecursively(bv, out, child.get());
            }
        }
    }
}

template<typename BoundablePtr, typename BV>
template<typename BoundablePtr2, typename BV2>
void BVTree<BoundablePtr, BV>::intersectTreeRecursively(const std::shared_ptr<BVNodeType>& me,
        const std::shared_ptr<BVNode<BoundablePtr2,BV2>>& her,
        std::vector<std::shared_ptr<BVNodeType>>& mine,
        std::vector<std::shared_ptr<BVNode<BoundablePtr2,BV2>>>& hers) const {

    //  if (me->bv->intersects(her->bv)) {
    //      printf("yes\n");
    //  } else {
    //      printf("no\n");
    //  }
    if (me->bv->intersects(*(her->bv))) {

        // both leaves intersect, add
        if (me->isLeaf() && her->isLeaf()) {
            mine.push_back(me);
            hers.push_back(her);
        } else {

            if (me->isLeaf()) {
                // intersect leaf with her
                for (const auto& herChild : her->children) {
                    intersectTreeRecursively(me, herChild, mine, hers);
                }
            } else if (her->isLeaf()) {
                // intersect her leaf with my children
                for (const auto& myChild : me->children) {
                    intersectTreeRecursively(myChild, her, mine, hers);
                }
            } else {

                // intersect all children with each other
                for (const auto& myChild : me->children) {
                    for (const auto& herChild : her->children) {
                        intersectTreeRecursively(myChild, herChild, mine, hers);
                    }
                }

            } // end one or no leaf cases
        } // end all leaf cases
    } // end if intersects
}

template<typename BoundablePtr, typename BV>
template<typename BoundablePtr2, typename BV2>
void BVTree<BoundablePtr, BV>::intersectTreeRecursively(BVNodeType* me,
        BVNode<BoundablePtr2,BV2>* her,
        std::vector<BVNodeType*>& mine,
        std::vector<BVNode<BoundablePtr2,BV2>*>& hers) const {

    //  if (me->bv->intersects(her->bv)) {
    //      printf("yes\n");
    //  } else {
    //      printf("no\n");
    //  }
    if (me->bv->intersects(*(her->bv))) {

        // both leaves intersect, add
        if (me->isLeaf() && her->isLeaf()) {
            mine.push_back(me);
            hers.push_back(her);
        } else {

            if (me->isLeaf()) {
                // intersect leaf with her
                for (const auto& herChild : her->children) {
                    intersectTreeRecursively(me, herChild.get(), mine, hers);
                }
            } else if (her->isLeaf()) {
                // intersect her leaf with my children
                for (const auto& myChild : me->children) {
                    intersectTreeRecursively(myChild.get(), her, mine, hers);
                }
            } else {

                // intersect all children with each other
                for (const auto& myChild : me->children) {
                    for (const auto& herChild : her->children) {
                        intersectTreeRecursively(myChild.get(), herChild.get(),
                                mine, hers);
                    }
                }

            } // end one or no leaf cases
        } // end all leaf cases
    } // end if intersects
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::getLeavesRecursively(
        std::vector<std::shared_ptr<BVNodeType>>& leaves,
        const std::shared_ptr<BVNodeType>& node) const {

    if (node->isLeaf()) {
        leaves.push_back(node);
    } else {
        for (std::shared_ptr<BVNodeType>& child : node->children) {
            getLeavesRecursively(leaves, child);
        }
    }
}

template<typename BoundablePtr, typename BV>
void BVTree<BoundablePtr, BV>::getLeavesRecursively(
        std::vector<BVNodeType*>& leaves, BVNodeType* node) const {

    if (node->isLeaf()) {
        leaves.push_back(node);
    } else {
        for (std::shared_ptr<BVNodeType>& child : node->children) {
            getLeavesRecursively(leaves, child.get());
        }
    }
}

// nearest boundable stuff
template<typename BoundablePtr, typename BV>
double nearest_boundable_recursive(const Point3d& pnt,
        const BVNode<BoundablePtr,BV>& node,
        std::reference_wrapper<const BoundablePtr>& nearest,
        Point3d& nearestPoint, double nearestDist) {

    Point3d p;

    if (node.isLeaf()) {
        for (const auto& elem : node.elems) {

            double d = elem->distanceToPoint(pnt, p);
            if (d < nearestDist) {
                nearestDist = d;
                nearest = std::ref(elem);
                nearestPoint = p;
            }
        }


    } else {

        // compute distances
        size_t size = node.children.size();
        int idx = 0;
        std::vector<size_t> nidxs(size);
        std::vector<double> ndists(size);

        // recurse through children
        for (const auto& child : node.children) {
            ndists[idx] = child->bv->distanceToPoint(pnt, p);
            nidxs[idx] = idx;
            if (ndists[idx] < nearestDist) {
                idx++;
            }
        }

        // sort indices by distance
        std::sort(nidxs.begin(), std::next(nidxs.begin(), idx),
                [&ndists](size_t i1, size_t i2) {return ndists[i1] < ndists[i2];}
            );

        // recurse through children in order of shortest distance
        for (int i=0; i<idx; i++) {
            int childIdx = nidxs[i];
            const auto& child = node.children[childIdx];
            if (ndists[childIdx] < nearestDist) {
                nearestDist = nearest_boundable_recursive(pnt, *child,
                                nearest, nearestPoint, nearestDist);
            }
        }

    }

    return nearestDist;
}

template<typename BoundablePtr, typename BV>
double nearest_boundable_recursive(const Point3d& pnt, const Vector3d& dir,
        const BVNode<BoundablePtr,BV>& node,
        std::reference_wrapper<BoundablePtr>& nearest, Point3d& nearestPoint,
        double nearestDist) {

    Point3d p;
    if (node.isLeaf()) {
        for (const BoundablePtr& elem : node.elems) {

            double d = elem->distanceToPoint(pnt, dir, p);
            if (d < nearestDist) {
                nearestDist = d;
                nearest = std::ref(elem);
                nearestPoint = p;
            }
        }
    } else {

        // compute distances
        size_t size = node.children.size();
        int idx = 0;
        std::vector<size_t> nidxs(size);
        std::vector<double> ndists(size);

        // compute distances for children
        for (const auto& child : node.children) {
            ndists[idx] = child->bv->distanceToPoint(pnt, dir, p);
            nidxs[idx] = idx;
            if (ndists[idx] < nearestDist) {
                idx++;
            }
        }

        // sort indices by distance
        std::sort(nidxs.begin(), std::next(nidxs.begin(), idx),
                [&ndists](size_t i1, size_t i2) {return ndists[i1] < ndists[i2];}
            );

        // recurse through children in order of shortest distance
        for (int i=0; i<idx; i++) {
            int childIdx = nidxs[i];
            const auto& child = node.children[childIdx];
            if (ndists[childIdx] < nearestDist) {
                nearestDist = nearest_boundable_recursive(pnt, dir, *child,
                                nearest, nearestPoint, nearestDist);
            }
        }

    }

    return nearestDist;
}

template<typename BoundablePtr, typename BV>
BoundablePtr nearest_boundable(const BVTree<BoundablePtr,BV>& bvh,
        const Point3d& pnt, Point3d& nearestPoint) {

    const BoundablePtr nearest = nullptr;
    Point3d p;  // temporary point

    double dist = math::DOUBLE_INFINITY;

    std::reference_wrapper<const BoundablePtr> nearest_ref = std::ref(nearest);
    dist = nearest_boundable_recursive(pnt, bvh.getRoot(), nearest_ref, nearestPoint, dist);

    return nearest_ref.get();
}

template<typename BoundablePtr, typename BV>
BoundablePtr nearest_boundable(const BVTree<BoundablePtr,BV>& bvh,
        const Point3d& pnt, const Vector3d& dir, Point3d& nearestPoint) {

    const BoundablePtr nearest = nullptr;
    Point3d p;  // temporary point

    double dist = math::DOUBLE_INFINITY;

    std::reference_wrapper<const BoundablePtr> nearest_ref = std::ref(nearest);
    dist = nearest_boundable_recursive(pnt, dir, bvh.getRoot(), nearest_ref, nearestPoint,
            dist);

    return nearest_ref.get();

}

}
}

#endif
