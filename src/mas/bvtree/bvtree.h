#ifndef MAS_BVTREE_H
#define MAS_BVTREE_H

#include "mas/core/base.h"
#include <vector>
#include <memory>

// used to identify bounding volume types
// for optimized static casts for handling intersections
// between volume types
#define BVTREE_BVID_BS		1
#define BVTREE_BVID_AABB	2
#define BVTREE_BVID_OBB		3

namespace mas {
namespace bvtree {

// things that can be bounded
class Boundable;
typedef std::shared_ptr<Boundable> PBoundable;
typedef std::vector<PBoundable> PBoundableList;
typedef std::vector<PBoundableList > PBoundableListList;

// Bounding volume trees
class BoundingVolume;	// Abstract bounding volume object
class BoundingSphere;   // Bounding Sphere
class BoundingBox;		// Abstract Bounding Box
class AABB;				// Axis-Aligned Bounding Box
class OBB;  			// Oriented Bounding Box

typedef std::shared_ptr<BoundingVolume> PBoundingVolume;
typedef std::shared_ptr<BoundingSphere> PBoundingSphere;
typedef std::shared_ptr<AABB> PAABB;
typedef std::shared_ptr<OBB> POBB;

class BVNode;	// abstract bounding-volume node
typedef std::shared_ptr<BVNode> PBVNode;
typedef std::weak_ptr<BVNode> WBVNode;	// weak node for back-pointer
typedef std::vector<PBVNode> PBVNodeList;

class BVTree;
typedef std::shared_ptr<BVTree> PBVTree;

// template node for easy implementation
template <typename BV> class BVNodeT;
typedef BVNodeT<BoundingSphere> BSNode;
typedef BVNodeT<AABB> AABBNode;
typedef BVNodeT<OBB> OBBNode;

// template tree
template <typename BV> class BVTreeT;
typedef BVTreeT<BoundingSphere> BSTree;
typedef BVTreeT<AABB> AABBTree;
typedef BVTreeT<OBB> OBBTree;

typedef std::shared_ptr<BSTree> PBSTree;
typedef std::shared_ptr<AABBTree> PAABBTree;
typedef std::shared_ptr<OBBTree> POBBTree;

// factory for generating volumes and nodes
class BVFactory;
class BVTreeFactory;


// Definitions
// objects that can be bounded
class Boundable {
public:
	// for 'this' call
	virtual bool updateBV(BoundingVolume* bv) const = 0;
	// default passing to pointer version
	virtual bool updateBV(PBoundingVolume bv) const;

	virtual void getCentroid(Point3d &c) const = 0;
	virtual void getCovariance(const Point3d &centre,
			Matrix3d &cov) const = 0;

	virtual double distanceToPoint(const Point3d &pnt,
			Point3d &nearest) const = 0;
	virtual double distanceToPoint(const Point3d &pnt,
			const Vector3d &dir, Point3d &nearest ) const = 0;
};

// Definitions
// objects that can be bounded
class BoundablePointSet : public Boundable {
public:
	std::vector<Point3d> pnts;
	int idx;

public:
	BoundablePointSet(int idx);
	BoundablePointSet(const std::vector<Point3d> &pnts, int idx);
	void setPoints(const std::vector<Point3d> &pnts);
	void addPoint(const Point3d &pnt);

	void setIndex(int idx);
	int getIndex();

	// for 'this' call
	bool updateBV(BoundingVolume* bv) const;
	// default passing to pointer version
	bool updateBV(PBoundingVolume bv) const;

	void getCentroid(Point3d &c) const;
	void getCovariance(const Point3d &centre,
			Matrix3d &cov) const;

	// closest point
	double distanceToPoint(const Point3d &pnt, Point3d &nearest) const;

	// always inf
	double distanceToPoint(const Point3d &pnt, const Vector3d &dir, Point3d &nearest ) const;
};

// volumes
class BoundingVolume {
protected:
	double margin;
protected:
	BoundingVolume();
	BoundingVolume(double margin);
	virtual ~BoundingVolume() {};
public:
	// used for optimized static casting
	virtual unsigned long uniqueClassId() const = 0;
	// margin to maintain around bounded volume
	virtual void setMargin(double m) = 0;
	virtual double getMargin() const;

	virtual bool intersectsPoint(const Point3d &p) const = 0;
	virtual bool intersectsSphere(const Point3d &c, double r) const = 0;
	virtual bool intersectsLine(const Point3d &p, const Vector3d &v)
	const = 0;
	virtual bool intersectsRay(const Point3d &p, const Vector3d &v)
	const = 0;
	virtual bool intersectsPlane(const Plane &p) const = 0;

	// Visitor pattern
	// Dispatch routines, if unknown pass on to bv.intersectsVisitor(*this)
	virtual bool intersects(const PBoundingVolume bv) const;
	// Visitor pattern
	// NOTE: DO NOT CALL bv.intersects(*this) FROM THIS FUNCTION!!
	// Worst case: resort to using bounding spheres
	virtual bool intersectsVisitor(const PBoundingVolume bv) const;

	// for 'this'
	virtual bool intersects(const BoundingVolume* bv) const;
	virtual bool intersectsVisitor(const BoundingVolume* bv) const;


	virtual double distanceToPoint(const Point3d &pnt,
			Point3d &nearest) const = 0;
	virtual double distanceToPoint(const Point3d &pnt,
			const Vector3d &dir, Point3d &nearest ) const = 0;

	// Useful for adding default behaviour for new types
	// if not explicitly implemented
	virtual BoundingSphere getBoundingSphere() const = 0;

	// Bound a set of boundables, can use centroid and covariance
	virtual void bound(const PBoundableList &b) = 0;
	// Split into smaller groups for inserting into a tree
	virtual void split(const PBoundableList &b,
			PBoundableListList &out) const = 0;

	// Update to include supplied info into this volume
	virtual bool updatePoint(const Point3d &p) = 0;
	virtual bool updateSphere(const Point3d &c, double r) = 0;
	// Visitor pattern, if unknown boundable pass on to Boundable
	virtual bool update(const PBoundable b);

	virtual PBoundingVolume copy() const = 0;

};

class BoundingSphere : public BoundingVolume {
public:
	static const unsigned long UNIQUE_ID;

	double r;
	Point3d c;

public:
	BoundingSphere();
	BoundingSphere(const BoundingSphere &copyMe);
	BoundingSphere(const Point3d &c, double r, double margin = 0);

	// BVTREE_BVID_BS
	virtual unsigned long uniqueClassId() const;

	void set(const Point3d &c, double r);
	void setRadius(double r);
	double getRadius() const;
	void setCentre(const Point3d &c);
	void getCentre(Point3d &c) const;
	virtual void setMargin(double m);

	virtual bool intersectsPoint(const Point3d &p) const;
	virtual bool intersectsSphere(const Point3d &c, double r) const;
	virtual bool intersectsLine(const Point3d &p, const Vector3d &v)
	const;
	virtual bool intersectsRay(const Point3d &p, const Vector3d &v)
	const;
	virtual bool intersectsPlane(const Plane &p) const;

	// Overrides to skip visitor (does intersectsSphere instead)
	virtual bool intersects(const PBoundingVolume bv) const;
	virtual bool intersects(const BoundingVolume* bv) const;

	bool intersects(const PBoundingSphere bs) const;
	bool intersects(const BoundingSphere* bs) const;

	virtual bool intersectsVisitor(const PBoundingVolume bv) const;
	virtual bool intersectsVisitor(const BoundingVolume* bv) const;

	virtual double distanceToPoint(const Point3d &pnt,
			Point3d &nearest) const;
	virtual double distanceToPoint(const Point3d &pnt,
			const Vector3d &dir, Point3d &nearest ) const;

	virtual void bound(const PBoundableList &b);
	// split oct-tree style
	virtual void split(const PBoundableList &b,
			PBoundableListList &out) const;

	virtual BoundingSphere getBoundingSphere() const;

	virtual bool updatePoint(const Point3d &p);
	virtual bool updateSphere(const Point3d &c, double r);

	virtual PBoundingVolume copy() const;
};

class BoundingBox : public BoundingVolume {
private:
	static int boxCorners[8][3];
public:
	Point3d c;
	Vector3d halfWidths;

public:
	BoundingBox();
	BoundingBox(const BoundingBox &copyMe);
	BoundingBox(const Point3d &c, const Vector3d &hw,
			double margin = 0);

	void set(const Point3d &c, const Vector3d &hw);
	void setHalfWidths(const Vector3d &hw);
	void getHalfWidths(Vector3d &hw);
	void setCentre(const Point3d &c);
	void getCentre(Point3d &c) const;

	virtual void setMargin(double m);
	// computes a corner, 0 <= idx < 8
	virtual void getCorner(int idx, Point3d &pnt) const;

	virtual void getLocalCoords(const Point3d &p, Point3d &out) const = 0;
	virtual void getLocalCoords(const Vector3d &v, Vector3d &out) const = 0;
	virtual void getWorldCoords(const Point3d &p, Point3d &out) const = 0;
	virtual void getWorldCoords(const Vector3d &p, Vector3d &out) const = 0;

	virtual bool intersectsPoint(const Point3d &p) const;
	virtual bool intersectsSphere(const Point3d &c, double r) const;
	virtual bool intersectsLine(const Point3d &p, const Vector3d &v)
	const;
	virtual bool intersectsRay(const Point3d &p, const Vector3d &v)
	const;
	virtual bool intersectsPlane(const Plane &p) const;

	virtual double distanceToPoint(const Point3d &pnt,
			Point3d &nearest) const;
	virtual double distanceToPoint(const Point3d &pnt,
			const Vector3d &dir, Point3d &nearest ) const;

	virtual BoundingSphere getBoundingSphere() const;

	virtual bool updatePoint(const Point3d &p);
	virtual bool updateSphere(const Point3d &c, double r);

};

class AABB : public BoundingBox {
public:
	static const unsigned long UNIQUE_ID;

public:
	AABB();
	AABB(const AABB &copyMe);
	AABB(const Point3d &c, const Vector3d &hw, double margin = 0);

	// BVTREE_BVID_AABB
	virtual unsigned long uniqueClassId() const;

	virtual void getLocalCoords(const Point3d &p, Point3d &out) const;
	virtual void getLocalCoords(const Vector3d &v, Vector3d &out) const;
	virtual void getWorldCoords(const Point3d &p, Point3d &out) const;
	virtual void getWorldCoords(const Vector3d &p, Vector3d &out) const;

	virtual bool intersects(const PBoundingVolume bv) const;
	virtual bool intersects(const BoundingVolume* bv) const;
	bool intersects(const PAABB bb) const;
	bool intersects(const AABB* bb) const;
	virtual bool intersectsVisitor(const PBoundingVolume bv) const;
	virtual bool intersectsVisitor(const BoundingVolume* bv) const;

	virtual void bound(const PBoundableList &blist);
	// split along longest axis
	virtual void split(const PBoundableList &b,
			PBoundableListList &out) const;

	virtual PBoundingVolume copy() const;
};

class OBB : public BoundingBox {
public:
	static const unsigned long UNIQUE_ID;
public:
	RotationMatrix3d R;

public:
	OBB();
	OBB(const OBB &copyMe);
	OBB(const AABB &copyMe);
	OBB(const Point3d &c, const RotationMatrix3d &R,
			const Vector3d &hw);
	OBB(const RigidTransform3d &trans, const Vector3d &hw);

	// BVTREE_BVID_OBB
	virtual unsigned long uniqueClassId() const;

	void set(const Point3d &c, const RotationMatrix3d &R,
			const Vector3d &hw);
	void set(const RigidTransform3d &trans, const Vector3d &hw);
	void setRotation(const RotationMatrix3d &R);
	void getRotation(RotationMatrix3d &R);

	virtual void getLocalCoords(const Point3d &p, Point3d &out) const;
	virtual void getLocalCoords(const Vector3d &v, Vector3d &out) const;
	virtual void getWorldCoords(const Point3d &p, Point3d &out) const;
	virtual void getWorldCoords(const Vector3d &p, Vector3d &out) const;

	virtual bool intersects(const PBoundingVolume bv) const;
	bool intersects(const POBB bv) const;
	bool intersects(const PAABB bv) const;
	virtual bool intersectsVisitor(const PBoundingVolume bv) const;

	virtual bool intersects(const BoundingVolume* bv) const;
	bool intersects(const OBB* bv) const;
	bool intersects(const AABB* bv) const;
	virtual bool intersectsVisitor(const BoundingVolume* bv) const;

	virtual void bound(const PBoundableList &b);
	// split along longest axis
	virtual void split(const PBoundableList &b,
			PBoundableListList &out) const;

	virtual PBoundingVolume copy()const ;

private:
	static bool boxesIntersect(const Vector3d &hw1, const Vector3d &hw2,
			const RotationMatrix3d &R21, const Vector3d &t21);
};

// basic abstract node
class BVNode {
private:
	WBVNode _this;
	WBVNode _parent;
public:
	PBoundingVolume bv;
	PBoundableList elems;
	PBVNodeList children;

private:
	BVNode(const BVNode &copyMe);
	BVNode& operator=(const BVNode &assignMe);
protected:
	BVNode();
public:
	BVNode(const PBoundingVolume bv, double margin = 0);
	BVNode(const PBoundingVolume bv, PBoundableList elems,
			double margin = 0);
	void attach(PBVNode pthis);
	PBVNode getParent();

	virtual BoundingSphere getBoundingSphere() const;
	virtual const PBoundingVolume getBoundingVolume() const;

	PBoundableList getElements();
	void setElements(const PBoundableList &elems);
	size_t numElements() const;
	void clearElements();

	PBVNodeList getChildren();
	void setChildren(const PBVNodeList &children);

	size_t numChildren() const;
	void clearChildren();
	void clear();

	bool isLeaf() const;
	bool isRoot() const;

	void setMargin(double margin);
	double getMargin() const;

	// split elements into more branches
	bool grow();
	bool growRecursively();

	void updateBounds();

protected:
	PBVNode spawnChild(const PBoundableList &elems);

	virtual void updateBoundsUp(const PBoundable b);
	virtual void updateBoundsDown(PBVNodeList &nodes);

};

// Abstract tree
class BVTree {
protected:
	PBVNode _root;

private:
	BVTree(const BVTree &copyMe);
	BVTree& operator=(const BVTree &assignMe);
protected:
	BVTree();
public:
	BVTree(const PBoundingVolume bv, double margin = 0);
	BVTree(const PBoundingVolume bv,
			const PBoundableList &elems, double margin = 0);
	BVTree(PBVNode root);
	virtual ~BVTree() {};

	PBVNode getRoot();

	virtual void build(const PBoundingVolume rootbv,
			const PBoundableList &elems, double margin = 0);

	// margin around objects, for robustness
	virtual void setMargin(double tol);
	virtual double getMargin() const;

	// intersection, return number of leaves
	virtual size_t intersectPoint(const Point3d &p, PBVNodeList &out)
	const;
	virtual size_t intersectSphere(const Point3d& c, double r,
			PBVNodeList &out) const ;
	virtual size_t intersectLine(const Point3d &p, const Vector3d &dir,
			PBVNodeList &out) const;
	virtual size_t intersectRay(const Point3d &p, const Vector3d &dir,
			PBVNodeList &out) const;
	virtual size_t intersectPlane(const Plane &plane, PBVNodeList &out)
	const;
	virtual size_t intersectBV(const PBoundingVolume bv, PBVNodeList &out) const;
	virtual size_t intersectBV(const BoundingVolume* bv, PBVNodeList &out) const;
	virtual size_t intersectTree(const PBVTree tree, PBVNodeList &mine,
			PBVNodeList &hers) const;

	virtual size_t getLeaves(PBVNodeList &leaves);

	virtual void update();

protected:
	// intersection, return number of leaves
	virtual void intersectPointRecursively(const Point3d &p,
			PBVNodeList &out, const PBVNode node) const;
	virtual void intersectSphereRecursively(const Point3d& c, double r,
			PBVNodeList &out, const PBVNode node) const;
	virtual void intersectLineRecursively(const Point3d &p,
			const Vector3d &dir, PBVNodeList &out, const PBVNode node)
	const;
	virtual void intersectRayRecursively(const Point3d &p,
			const Vector3d &dir, PBVNodeList &out, const PBVNode node) const;
	virtual void intersectPlaneRecursively(const Plane &plane,
			PBVNodeList &out, const PBVNode node) const;

	virtual void intersectBVRecursively(const BoundingVolume* bv,
			PBVNodeList &out, const PBVNode node) const;
	virtual void intersectBVRecursively(const PBoundingVolume bv,
			PBVNodeList &out, const PBVNode node) const;
	virtual void intersectTreeRecursively(const PBVNode me, const PBVNode her,
			PBVNodeList &mine, PBVNodeList &hers) const;

	virtual void getLeavesRecursively(PBVNodeList &leaves, const PBVNode node)
	const;

};

// Templated Node in the BV Tree
template <typename BV>
class BVNodeT : public BVNode {
public:
	typedef BVNodeT<BV> BVTNode;
	typedef std::shared_ptr<BVTNode> PBVTNode;
	typedef std::vector<PBVTNode> PBVTNodeList;
private:
	BVNodeT(const BVNodeT<BV> &copyMe);
	BVNodeT<BV>& operator=(const BVNodeT<BV> &assignMe);
public:
	BVNodeT(double margin = 0);
	BVNodeT(const PBoundableList &elems, double margin = 0);
	virtual ~BVNodeT() {};

	void setChildren(const PBVTNodeList &children);
protected:
	PBVNode spawnChild(const PBoundableList &elems);
};

// Bounding volume tree
template<typename BV>
class BVTreeT : public BVTree {
public:
	typedef BVNodeT<BV> BVTNode;
	typedef std::shared_ptr<BVTNode> PBVTNode;
	typedef std::vector<PBVTNode> PBVTNodeList;
private:
	BVTreeT(const BVTreeT<BV> &copyMe);
	BVTreeT<BV>& operator=(const BVTreeT<BV> &assignMe);

public:
	BVTreeT(double margin = 0);
	BVTreeT(const PBoundableList &elems, double margin = 0);
	BVTreeT(BVTNode root);
	virtual ~BVTreeT() {};

	PBVTNode getRootT();
	virtual void build(const PBoundableList &elems, double margin = 0);
};


class BVFactory {
public:
	static PBoundingSphere createBoundingSphere();
	static PBoundingSphere createBoundingSphere(const BoundingSphere &bs);
	static PAABB createAABB();
	static PAABB createAABB(const AABB &aabb);
	static POBB createOBB();
	static POBB createOBB(const OBB &obb);

	template <typename BV>
	static std::shared_ptr<BV> createBV();
};

class BVTreeFactory {
public:

	// non-template, duplicates bv
	static PBVNode createNode(const PBoundingVolume bv,
			double margin = 0);
	static PBVNode createNode(const PBoundingVolume bv,
			const PBoundableList &elems, double margin = 0);

	// generic template
	template <typename BV>
	static PBVNode createNode(double margin = 0);
	template <typename BV>
	static PBVNode createNode(const PBoundableList &elems, double margin = 0);

	// 'differentiated', fixed template type
	template <typename BV>
	static std::shared_ptr<BVNodeT<BV> > createNodeT(double margin = 0);
	template <typename BV>
	static std::shared_ptr<BVNodeT<BV> > createNodeT(
			const PBoundableList &elems, double margin = 0);

	// non-templated, duplicates bv
	static PBVTree createTree(const PBoundingVolume rootbv,
			double margin = 0);
	static PBVTree createTree(const PBoundingVolume rootbv,
			const PBoundableList &elems, double margin = 0);

	// generic template
	template <typename BV>
	static PBVTree createTree(double margin = 0);
	template <typename BV>
	static PBVTree createTree(const PBoundableList &elems, double margin = 0);

	// 'differentiated' fixed type
	template <typename BV>
	static std::shared_ptr<BVTreeT<BV> > createTreeT(double margin = 0);
	template <typename BV>
	static std::shared_ptr<BVTreeT<BV> > createTreeT(
			const PBoundableList &elems, double margin = 0);
};

// static routines
PBoundable nearest_boundable(const PBVTree bvh, const Point3d &p,
		Point3d &nearestPoint);
PBoundable nearest_boundable(const PBVTree bvh, const Point3d &p,
		const Vector3d &dir, Point3d &nearestPoint);


/*
struct NearestBoundableData {
	PBoundableList nearestBoundables;
	std::vector<Point3d>  nearestPoints;
	double dist;
	double tol;
};

PBoundable nearest_boundable(const PBVTree bvh, const Point3d &p,
		double tol, NearestBoundableData &data);
PBoundable nearest_boundable(const PBVTree bvh, const Point3d &p,
		const Vector3d &dir, double tol, NearestBoundableData &data);
*/


}
}

#include "mas/bvtree/bvtree.hpp"

#endif




