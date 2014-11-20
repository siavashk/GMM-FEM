#ifndef MAS_CSG_BSP_H
#define MAS_CSG_BSP_H

#include "mas/core/base.h"
#include "mas/mesh/mesh.h"

#define BSP_DEFAULT_EPSILON 1e-15

namespace mas {
namespace csg {
namespace bsp {

using namespace mas::mesh;

class CSGNode {
private:
	double tol;
	CSGNode* frontNode;
	CSGNode* backNode;
	size_t id;
	static size_t nextId;
public:
	Plane* plane;
	PPolygonList polygons;

protected:
	void cleanup();
public:
	CSGNode();
	CSGNode(const CSGNode& copyMe);
	CSGNode(const PPolygonList &polygons);
	CSGNode(const PolygonMesh &mesh);
	virtual ~CSGNode();
	virtual CSGNode& operator=(const CSGNode& assignMe);

	void build(const PPolygonList &polygons);
	void invert();
	PPolygonList clipPolygons(const PPolygonList &polygons) const;
	void clipTo(const CSGNode &csg);
	PPolygonList getAllPolygons() const;

	void setTolerance(double tol);
	double getTolerance() const;
	size_t numPolygons() const;
};

// Splits a polygon across a plane
// front: list of polygons on the +normal side
// back:  list of polygons on the -normal side
// coplanarFront/Back: if coplanar, placed in one of these depending
//        whether the poly's normal is consistent with the plane (front)
//        or opposite (back)
// Returns the number of split polygons
int split_polygon(PPolygon poly, Plane &plane,
		PPolygonList &front, PPolygonList &back,
		PPolygonList &coplanarFront, PPolygonList &coplanarBack,
		double tol);


// compute union of two geometries
PolygonMesh vol_union(const PolygonMesh &a, const PolygonMesh &b,
		double tol = BSP_DEFAULT_EPSILON);
PPolygonList vol_union(const PPolygonList &a, const PPolygonList &b,
		double tol = BSP_DEFAULT_EPSILON);
CSGNode vol_union(CSGNode &a, CSGNode &b);

// compute intersection of two geometries
PolygonMesh vol_intersection(const PolygonMesh &a, const PolygonMesh &b,
		double tol = BSP_DEFAULT_EPSILON);
PPolygonList vol_intersection(const PPolygonList &a, const PPolygonList &b,
		double tol = BSP_DEFAULT_EPSILON);
CSGNode vol_intersection(CSGNode &a, CSGNode &b);

// subtract volume b from a
PolygonMesh vol_subtraction(const PolygonMesh &a, const PolygonMesh &b,
		double tol = BSP_DEFAULT_EPSILON);
PPolygonList vol_subtraction(const PPolygonList &a, const PPolygonList &b,
		double tol = BSP_DEFAULT_EPSILON);
CSGNode vol_subtraction(CSGNode &a, CSGNode &b);

}
}
}

#endif
