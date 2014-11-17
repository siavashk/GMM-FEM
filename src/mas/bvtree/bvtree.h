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

// useful aliases
using SharedBoundable = std::shared_ptr<Boundable>;
using UniqueBoundable = std::unique_ptr<Boundable>;

// Bounding volume trees
class BoundingVolume;	// Abstract bounding volume object
class BoundingSphere;   // Bounding Sphere
class BoundingBox;		// Abstract Bounding Box
class AABB;				// Axis-Aligned Bounding Box
class OBB;  			// Oriented Bounding Box

// useful aliases
using BV = BoundingVolume;
using SharedBV = std::shared_ptr<BoundingVolume>;
using UniqueBV = std::unique_ptr<BoundingVolume>;
using BS = BoundingSphere;
using SharedBS = std::shared_ptr<BoundingSphere>;
using UniqueBS = std::unique_ptr<BoundingSphere>;
using SharedAABB = std::shared_ptr<AABB>;
using UniqueAABB = std::unique_ptr<AABB>;
using SharedOBB = std::shared_ptr<OBB>;
using UniqueOBB = std::unique_ptr<OBB>;


class BVNode;	// abstract bounding-volume node
using SharedBVNode = std::shared_ptr<BVNode>;
using WeakBVNode = std::weak_ptr<BVNode>;
using UniqueBVNode = std::unique_ptr<BVNode>;

class BVTree;
using UniqueBVTree = std::unique_ptr<BVTree>;

// template node for easy implementation
template <typename BV> class BVNodeT;
using BSNode = BVNodeT<BoundingSphere>;
using AABBNode = BVNodeT<AABB>;
using OBBNode = BVNodeT<OBB>;

// template tree
template <typename BV> class BVTreeT;
using BSTree = BVTreeT<BoundingSphere>;
using AABBTree = BVTreeT<AABB>;
using OBBTree = BVTreeT<OBB>;

using UniqueBSTree = std::unique_ptr<BSTree>;
using UniqueAABBTree = std::unique_ptr<AABBTree>;
using UniqueOBBTree = std::unique_ptr<OBBTree>;

// factory for generating volumes and nodes
class BVFactory;
class BVTreeFactory;

// Definitions
// objects that can be bounded
class Boundable {
public:
	int idx;

public:

	Boundable(int idx);

	int getIndex();
	void setIndex(int idx);

	// Update supplied BV using info from THIS
	virtual bool updateBV(BoundingVolume& bv) const = 0;

	virtual void getCentroid(Point3d& c) const = 0;
	virtual void getCovariance(const Point3d& centre,
			Matrix3d& cov) const = 0;

	virtual double distanceToPoint(const Point3d& pnt,
			Point3d& nearest) const = 0;
	virtual double distanceToPoint(const Point3d& pnt,
			const Vector3d& dir, Point3d& nearest ) const = 0;
};

// Definitions
// objects that can be bounded
class BoundablePointSet : public Boundable {
public:
	std::vector<Point3d> pnts;

public:
	BoundablePointSet(int idx);
	BoundablePointSet(const std::vector<Point3d>& pnts, int idx);
	BoundablePointSet(std::vector<Point3d>&& pnts, int idx);  // move semantics

	void setPoints(const std::vector<Point3d>& pnts);
	void setPoints(std::vector<Point3d>&& pnts);  // move semantics
	void addPoint(const Point3d& pnt);

	bool updateBV(BoundingVolume& bv) const;

	void getCentroid(Point3d& c) const;
	void getCovariance(const Point3d& centre,
			Matrix3d& cov) const;

	// closest point
	double distanceToPoint(const Point3d& pnt, Point3d& nearest) const;

	// always inf
	double distanceToPoint(const Point3d& pnt, const Vector3d& dir, Point3d& nearest ) const;
};

class IndexedBoundablePointSet : public Boundable {
public:
	std::vector<std::shared_ptr<IndexedPoint3d> > pnts;
public:
	IndexedBoundablePointSet(int idx);
	IndexedBoundablePointSet(const std::vector<std::shared_ptr<IndexedPoint3d> >&  pnts, int idx);
	IndexedBoundablePointSet(std::vector<std::shared_ptr<IndexedPoint3d> >&& pnts, int idx);  // move semantics

	void setPoints(const std::vector<std::shared_ptr<IndexedPoint3d> >& pnts);
	void setPoints(std::vector<std::shared_ptr<IndexedPoint3d> >&& pnts);

	void addPoint(const std::shared_ptr<IndexedPoint3d>& pnt);
	void addPoint(std::shared_ptr<IndexedPoint3d>&& pnt);

	bool updateBV(BoundingVolume& bv) const;

	void getCentroid(Point3d& c) const;
	void getCovariance(const Point3d& centre, Matrix3d& cov) const;

	// closest point
	double distanceToPoint(const Point3d& pnt, Point3d& nearest) const;

	// always inf
	double distanceToPoint(const Point3d& pnt, const Vector3d& dir, Point3d& nearest ) const;
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

	virtual bool intersectsPoint(const Point3d& p) const = 0;
	virtual bool intersectsSphere(const Point3d& c, double r) const = 0;
	virtual bool intersectsLine(const Point3d& p, const Vector3d& v)
	const = 0;
	virtual bool intersectsRay(const Point3d& p, const Vector3d& v)
	const = 0;
	virtual bool intersectsPlane(const Plane& p) const = 0;

	// Visitor pattern
	// Dispatch routines, if unknown pass on to bv.intersectsVisitor(*this)
	virtual bool intersects(const BoundingVolume& bv) const;

	// Visitor pattern
	// NOTE: DO NOT CALL bv.intersects(*this) FROM THIS FUNCTION!!
	// Worst case: resort to using bounding spheres
	virtual bool intersectsVisitor(const BoundingVolume& bv) const;

	virtual double distanceToPoint(const Point3d& pnt,
			Point3d& nearest) const = 0;
	virtual double distanceToPoint(const Point3d& pnt,
			const Vector3d& dir, Point3d& nearest ) const = 0;

	// Useful for adding default behaviour for new types
	// if not explicitly implemented
	virtual BoundingSphere getBoundingSphere() const = 0;

	// Update to include supplied info into this volume
	virtual bool updatePoint(const Point3d& p) = 0;
	virtual bool updateSphere(const Point3d& c, double r) = 0;

	// Visitor pattern, if unknown boundable pass on to Boundable
	virtual bool update(const Boundable& b);

	// Bound a set of boundables, can use centroid and covariance
	virtual void bound(const std::vector<SharedBoundable>& b) = 0;  // shared boundables

	// Split into smaller groups for inserting into a tree
	virtual bool split(const std::vector<SharedBoundable>& b,               // copy shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const = 0;
	virtual bool split(std::vector<SharedBoundable>&& b,                    // move shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const = 0;

	virtual BoundingVolume* clone();
	virtual BoundingVolume* newInstance();

};

class BoundingSphere : public BoundingVolume {
public:
	static const unsigned long UNIQUE_ID;

	double r;
	Point3d c;

public:
	BoundingSphere();
	BoundingSphere(const BoundingSphere& copyMe);
	BoundingSphere(const Point3d& c, double r, double margin = 0);

	// BVTREE_BVID_BS
	virtual unsigned long uniqueClassId() const;

	void set(const Point3d& c, double r);
	void setRadius(double r);
	double getRadius() const;
	void setCentre(const Point3d& c);
	void getCentre(Point3d& c) const;
	virtual void setMargin(double m);

	virtual bool intersectsPoint(const Point3d& p) const;
	virtual bool intersectsSphere(const Point3d& c, double r) const;
	virtual bool intersectsLine(const Point3d& p, const Vector3d& v) const;
	virtual bool intersectsRay(const Point3d& p, const Vector3d& v)	const;
	virtual bool intersectsPlane(const Plane& p) const;

	// Overrides to skip visitor (does intersectsSphere instead)
	virtual bool intersects(const BoundingVolume& bv) const;

	bool intersects(const BoundingSphere& bs) const;
	virtual bool intersectsVisitor(const BoundingVolume& bv) const;

	virtual double distanceToPoint(const Point3d& pnt, Point3d& nearest) const;
	virtual double distanceToPoint(const Point3d& pnt, const Vector3d& dir,
			Point3d& nearest ) const;

	virtual BoundingSphere getBoundingSphere() const;

	virtual bool updatePoint(const Point3d& p);
	virtual bool updateSphere(const Point3d& c, double r);

	// Bound a set of boundables, can use centroid and covariance
	virtual void bound(const std::vector<SharedBoundable>& b);  // shared boundables

	// Split into smaller groups for inserting into a tree
	// oct-tree style
	virtual bool split(const std::vector<SharedBoundable>& b,               // copy shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const;
	virtual bool split(std::vector<SharedBoundable>&& b,                    // move shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const;

	virtual BoundingSphere* clone() const;
	virtual BoundingSphere* newInstance() const;
};



class BoundingBox : public BoundingVolume {
private:
	static int boxCorners[8][3];
public:
	Point3d c;
	Vector3d halfWidths;

public:
	BoundingBox();
	BoundingBox(const BoundingBox& copyMe);
	BoundingBox(const Point3d& c, const Vector3d& hw,
			double margin = 0);

	void set(const Point3d& c, const Vector3d& hw);
	void setHalfWidths(const Vector3d& hw);
	void getHalfWidths(Vector3d& hw) const;
	void setCentre(const Point3d& c);
	void getCentre(Point3d& c) const;

	virtual void setMargin(double m);
	// computes a corner, 0 <= idx < 8
	virtual void getCorner(int idx, Point3d& pnt) const;

	virtual void getLocalCoords(const Point3d& p, Point3d& out) const = 0;
	virtual void getLocalCoords(const Vector3d& v, Vector3d& out) const = 0;
	virtual void getWorldCoords(const Point3d& p, Point3d& out) const = 0;
	virtual void getWorldCoords(const Vector3d& p, Vector3d& out) const = 0;

	virtual bool intersectsPoint(const Point3d& p) const;
	virtual bool intersectsSphere(const Point3d& c, double r) const;
	virtual bool intersectsLine(const Point3d& p, const Vector3d& v) const;
	virtual bool intersectsRay(const Point3d& p, const Vector3d& v) const;
	virtual bool intersectsPlane(const Plane& p) const;

	virtual double distanceToPoint(const Point3d& pnt,
			Point3d& nearest) const;
	virtual double distanceToPoint(const Point3d& pnt,
			const Vector3d& dir, Point3d& nearest ) const;

	virtual BoundingSphere getBoundingSphere() const;

	virtual bool updatePoint(const Point3d& p);
	virtual bool updateSphere(const Point3d& c, double r);

};

class AABB : public BoundingBox {
public:
	static const unsigned long UNIQUE_ID;

public:
	AABB();
	AABB(const AABB& copyMe);
	AABB(const Point3d& c, const Vector3d& hw, double margin = 0);

	// BVTREE_BVID_AABB
	virtual unsigned long uniqueClassId() const;

	virtual void getLocalCoords(const Point3d& p, Point3d& out) const;
	virtual void getLocalCoords(const Vector3d& v, Vector3d& out) const;
	virtual void getWorldCoords(const Point3d& p, Point3d& out) const;
	virtual void getWorldCoords(const Vector3d& p, Vector3d& out) const;

	virtual bool intersects(const BoundingVolume& bv) const;
	bool intersects(const AABB& bb) const;
	virtual bool intersectsVisitor(const BoundingVolume& bv) const;


	// Bound a set of boundables, can use centroid and covariance
	virtual void bound(const std::vector<SharedBoundable>& b);  // shared boundables

	// Split into smaller groups for inserting into a tree
	// Split along longest axis
	virtual bool split(const std::vector<SharedBoundable>& b,               // copy shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const;
	virtual bool split(std::vector<SharedBoundable>&& b,                    // move shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const;

	virtual AABB* clone();
	virtual AABB* newInstance();
};

class OBB : public BoundingBox {
public:
	static const unsigned long UNIQUE_ID;
public:
	RotationMatrix3d R;

public:
	OBB();
	OBB(const OBB& copyMe);
	OBB(const AABB& copyMe);
	OBB(const Point3d& c, const RotationMatrix3d& R,
			const Vector3d& hw);
	OBB(const RigidTransform3d& trans, const Vector3d& hw);

	// BVTREE_BVID_OBB
	virtual unsigned long uniqueClassId() const;

	void set(const Point3d& c, const RotationMatrix3d& R,
			const Vector3d& hw);
	void set(const RigidTransform3d& trans, const Vector3d& hw);
	void setRotation(const RotationMatrix3d& R);
	void getRotation(RotationMatrix3d& R);

	virtual void getLocalCoords(const Point3d& p, Point3d& out) const;
	virtual void getLocalCoords(const Vector3d& v, Vector3d& out) const;
	virtual void getWorldCoords(const Point3d& p, Point3d& out) const;
	virtual void getWorldCoords(const Vector3d& p, Vector3d& out) const;

	virtual bool intersects(const BoundingVolume& bv) const;
	bool intersects(const OBB& bv) const;
	bool intersects(const AABB& bv) const;
	virtual bool intersectsVisitor(const BoundingVolume& bv) const;

	// Bound a set of boundables, can use centroid and covariance
	virtual void bound(const std::vector<SharedBoundable>& b);  // shared boundables

	// Split into smaller groups for inserting into a tree
	// Split along longest axis
	virtual bool split(const std::vector<SharedBoundable>& b,               // copy shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const;
	virtual bool split(std::vector<SharedBoundable>&& b,                    // move shared boundables
			std::vector<std::vector<SharedBoundable>>& out) const;

	virtual OBB* clone();
	virtual OBB* newInstance();

private:
	static bool boxesIntersect ( const Vector3d& hw1, const Vector3d& hw2,
			const RotationMatrix3d& R1, const RotationMatrix3d& R2,
			const Vector3d& pd, const Vector3d& px);
	static bool boxesIntersect(const Vector3d& hw1, const Vector3d& hw2,
			const RotationMatrix3d& R21, const Vector3d& t21);
};

// basic abstract node
class BVNode {
private:
	BVNode* parent;  // raw parent node (so can observe strong or weak parent)
public:
	UniqueBV bv;
	std::vector<SharedBoundable> elems;
	std::vector<SharedBVNode> children;

private:
	BVNode(const BVNode& copyMe);
	BVNode& operator=(const BVNode& assignMe);
protected:
	BVNode();
public:
	BVNode(UniqueBV&& bv, double margin = 0);

	// copy elements
	BVNode(UniqueBV&& bv,  const std::vector<SharedBoundable>& elems,
			double margin = 0);

	// move elements
	BVNode(UniqueBV&& bv, std::vector<SharedBoundable>&& elems,
			double margin = 0);

	BVNode* getParent();
	void setParent(BVNode* parent);

	virtual BoundingSphere getBoundingSphere() const;
	virtual const BoundingVolume& getBoundingVolume() const;

	std::vector<SharedBoundable>& getElements();
	void setElements(const std::vector<SharedBoundable>& elems);
	void setElements(std::vector<SharedBoundable>&& elems);

	size_t numElements() const;
	void clearElements();

	std::vector<SharedBVNode>& getChildren();
	void setChildren(const std::vector<SharedBVNode>& children);
	void setChildren(std::vector<SharedBVNode>&& children);

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
	BVNode* spawnChild(const std::vector<SharedBoundable>& elems);
	BVNode* spawnChild(std::vector<SharedBoundable>&& elems);

	virtual void updateBoundsUp(const Boundable& b);
	virtual void updateBoundsDown(BVNode& node);

};

// Abstract tree
class BVTree {
protected:
	SharedBVNode root;

private:
	BVTree(const BVTree& copyMe);
	BVTree& operator=(const BVTree& assignMe);
protected:
	BVTree();
public:
	BVTree(UniqueBV rootbv, double margin = 0);
	BVTree(UniqueBV rootbv,	const std::vector<SharedBoundable>& elems, double margin = 0);
	BVTree(UniqueBV rootbv,	std::vector<SharedBoundable>&& elems, double margin = 0);

	BVTree(SharedBVNode&& root);

	SharedBVNode getRoot();
	double getRadius() const;

	virtual void build(UniqueBV rootbv,	const std::vector<SharedBoundable>& elems, double margin = 0);
	virtual void build(UniqueBV rootbv,	std::vector<SharedBoundable>&& elems, double margin = 0);

	// margin around objects, for robustness
	virtual void setMargin(double tol);
	virtual double getMargin() const;

	// intersection, return number of leaves
	virtual size_t intersectPoint(const Point3d& p, std::vector<SharedBVNode>& out) const;
	virtual size_t intersectPoint(const Point3d& p, std::vector<BVNode*>& out) const;

	virtual size_t intersectSphere(const Point3d& c, double r, std::vector<SharedBVNode>& out) const ;
	virtual size_t intersectSphere(const Point3d& c, double r, std::vector<BVNode*>& out) const ;

	virtual size_t intersectLine(const Point3d& p, const Vector3d& dir,	std::vector<SharedBVNode>& out) const;
	virtual size_t intersectLine(const Point3d& p, const Vector3d& dir,	std::vector<BVNode*>& out) const;
	virtual size_t intersectRay(const Point3d& p, const Vector3d& dir, std::vector<SharedBVNode>& out) const;
	virtual size_t intersectRay(const Point3d& p, const Vector3d& dir, std::vector<BVNode*>& out) const;
	virtual size_t intersectPlane(const Plane& plane, std::vector<SharedBVNode>& out) const;
	virtual size_t intersectPlane(const Plane& plane, std::vector<BVNode*>& out) const;
	virtual size_t intersectBV(const BoundingVolume& bv, std::vector<SharedBVNode>& out) const;
	virtual size_t intersectBV(const BoundingVolume& bv, std::vector<BVNode*>& out) const;

	virtual size_t intersectTree(const BVTree& tree,
			std::vector<SharedBVNode>& mine, std::vector<SharedBVNode>& hers) const;
	virtual size_t intersectTree(const BVTree& tree,
			std::vector<BVNode*>& mine, std::vector<BVNode*>& hers) const;

	virtual size_t getLeaves(std::vector<SharedBVNode>& leaves);
	virtual size_t getLeaves(std::vector<BVNode*>& leaves);

	virtual void update();

protected:
	// intersection, return number of leaves
	virtual void intersectPointRecursively(const Point3d& p,
			std::vector<SharedBVNode>& out, const BVNode& node) const;
	virtual void intersectPointRecursively(const Point3d& p,
				std::vector<BVNode*>& out, const BVNode& node) const;

	virtual void intersectSphereRecursively(const Point3d& c, double r,
			std::vector<SharedBVNode>& out, const BVNode& node) const;
	virtual void intersectSphereRecursively(const Point3d& c, double r,
			std::vector<BVNode*>& out, const BVNode& node) const;

	virtual void intersectLineRecursively(const Point3d& p,	const Vector3d& dir,
			std::vector<SharedBVNode>& out, const BVNode& node) const;
	virtual void intersectLineRecursively(const Point3d& p,	const Vector3d& dir,
			std::vector<BVNode*>& out, const BVNode& node) const;

	virtual void intersectRayRecursively(const Point3d& p, const Vector3d& dir,
			std::vector<SharedBVNode>& out, const BVNode& node) const;
	virtual void intersectPlaneRecursively(const Plane& plane,
			std::vector<BVNode*>& out, const BVNode& node) const;

	virtual void intersectBVRecursively(const BoundingVolume& bv,
			std::vector<SharedBVNode>& out, const BVNode& node) const;
	virtual void intersectBVRecursively(const BoundingVolume& bv,
				std::vector<BVNode*>& out, const BVNode& node) const;

	virtual void intersectTreeRecursively(const BVNode& me, const BVNode& her,
			std::vector<SharedBVNode>& mine, std::vector<SharedBVNode>& hers) const;
	virtual void intersectTreeRecursively(const BVNode& me, const BVNode& her,
				std::vector<BVNode*>& mine, std::vector<BVNode*>& hers) const;


	virtual void getLeavesRecursively(std::vector<SharedBVNode>& leaves, const BVNode& node) const;
	virtual void getLeavesRecursively(std::vector<BVNode*>& leaves, const BVNode& node) const;

};

template <class BV> using SharedBVNodeT = std::shared_ptr<BVNodeT<BV>>;

// Templated Node in the BV Tree
template <typename BV>
class BVNodeT : public BVNode {

private:
	BVNodeT(const BVNodeT<BV>& copyMe);
	BVNodeT<BV>& operator=(const BVNodeT<BV>& assignMe);
public:
	BVNodeT(double margin = 0);

	BVNodeT(const std::vector<SharedBoundable>& elems, double margin = 0); // copy
	BVNodeT(std::vector<SharedBoundable>&& elems, double margin = 0);      // move

	void setChildren(const std::vector<SharedBVNode>& children);  // copy
	void setChildren(std::vector<SharedBVNode>&& children);       // move
protected:

	BVNodeT<BV>* spawnChild(const std::vector<SharedBoundable>& elems);
	BVNodeT<BV>* spawnChild(std::vector<SharedBoundable>&& elems);

};

// Bounding volume tree
template<typename BV>
class BVTreeT : public BVTree {
private:
	BVTreeT(const BVTreeT<BV>& copyMe);
	BVTreeT<BV>& operator=(const BVTreeT<BV>& assignMe);

public:
	BVTreeT(double margin = 0);
	BVTreeT(const std::vector<SharedBoundable>& elems, double margin = 0);
	BVTreeT(std::vector<SharedBoundable>&& elems, double margin = 0);

	BVTreeT(SharedBVNode&& root);

	virtual void build(const std::vector<SharedBoundable>& elems, double margin = 0);
	virtual void build(std::vector<SharedBoundable>&& elems, double margin = 0);
};


class BVFactory {
public:
	static UniqueBS createBoundingSphere();
	static UniqueBS createBoundingSphere(const BoundingSphere& bs);
	static UniqueAABB createAABB();
	static UniqueAABB createAABB(const AABB& aabb);
	static UniqueOBB createOBB();
	static UniqueOBB createOBB(const OBB& obb);

	template <typename BV>
	static std::unique_ptr<BV> createBV();
};

class BVTreeFactory {
public:

	// non-template, duplicates bv
	static UniqueBVNode createNode(const BoundingVolume& bv, double margin = 0);
	static UniqueBVNode createNode(const BoundingVolume& bv, const std::vector<SharedBoundable>& elems, double margin = 0);
	static UniqueBVNode createNode(const BoundingVolume& bv, std::vector<SharedBoundable>&& elems, double margin = 0);
	static UniqueBVNode createNode(UniqueBV&& bv, double margin = 0);
	static UniqueBVNode createNode(UniqueBV&& bv, const std::vector<SharedBoundable>& elems, double margin = 0);
	static UniqueBVNode createNode(UniqueBV&& bv, std::vector<SharedBoundable>&& elems, double margin = 0);

	// generic template
	template <typename BV>
	static UniqueBVNode createNode(double margin = 0);
	template <typename BV>
	static UniqueBVNode createNode(const std::vector<SharedBoundable>& elems, double margin = 0);
	template <typename BV>
	static UniqueBVNode createNode(std::vector<SharedBoundable>&& elems, double margin = 0);

	// 'differentiated', fixed template type
	template <typename BV>
	static std::unique_ptr<BVNodeT<BV> > createNodeT(double margin = 0);
	template <typename BV>
	static std::unique_ptr<BVNodeT<BV> > createNodeT(const std::vector<SharedBoundable>& elems, double margin = 0);
	template <typename BV>
	static std::unique_ptr<BVNodeT<BV> > createNodeT(std::vector<SharedBoundable>&& elems, double margin = 0);

	// non-templated, duplicates bv
	static UniqueBVTree createTree(const BoundingVolume& rootbv, double margin = 0);
	static UniqueBVTree createTree(const BoundingVolume& rootbv, const std::vector<SharedBoundable>& elems, double margin = 0);
	static UniqueBVTree createTree(const BoundingVolume& rootbv, std::vector<SharedBoundable>&& elems, double margin = 0);
	static UniqueBVTree createTree(UniqueBV&& rootbv, double margin = 0);
	static UniqueBVTree createTree(UniqueBV& rootbv, const std::vector<SharedBoundable>& elems, double margin = 0);
	static UniqueBVTree createTree(UniqueBV& rootbv, std::vector<SharedBoundable>&& elems, double margin = 0);

	// generic template
	template <typename BV>
	static UniqueBVTree createTree(double margin = 0);
	template <typename BV>
	static UniqueBVTree createTree(const std::vector<SharedBoundable>& elems, double margin = 0);
	template <typename BV>
	static UniqueBVTree createTree(std::vector<SharedBoundable>&& elems, double margin = 0);

	// 'differentiated' fixed type
	template <typename BV>
	static std::unique_ptr<BVTreeT<BV> > createTreeT(double margin = 0);
	template <typename BV>
	static std::unique_ptr<BVTreeT<BV> > createTreeT(const std::vector<SharedBoundable>& elems, double margin = 0);
	template <typename BV>
	static std::unique_ptr<BVTreeT<BV> > createTreeT(std::vector<SharedBoundable>&& elems, double margin = 0);
};

// static routines
Boundable* nearest_boundable(const BVTree& bvh, const Point3d& p, Point3d& nearestPoint);
Boundable* nearest_boundable(const BVTree& bvh, const Point3d& p, const Vector3d& dir, Point3d& nearestPoint);


/*
struct NearestBoundableData {
	PBoundableList nearestBoundables;
	std::vector<Point3d>  nearestPoints;
	double dist;
	double tol;
};

PBoundable nearest_boundable(const PBVTree bvh, const Point3d& p,
		double tol, NearestBoundableData& data);
PBoundable nearest_boundable(const PBVTree bvh, const Point3d& p,
		const Vector3d& dir, double tol, NearestBoundableData& data);
 */


}
}

#include "mas/bvtree/bvtree.hpp"

#endif




