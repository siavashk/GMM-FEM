#ifndef MAS_MESH_BV_H
#define MAS_MESH_BV_H

#include "mas/mesh/mesh.h"
#include "mas/bvtree/bvtree.h"

// adds bvtree functionality to mesh objects

namespace mas {
namespace mesh {

// wrapper for polygon structure
class BoundablePolygon;
typedef std::shared_ptr<BoundablePolygon> SharedBoundablePolygon;

using namespace bvtree;

// wrapper for polygon
class BoundablePolygon {
public:
	SharedPolygon polygon;
	std::vector<SharedPolygon> tris; // triangulation of polygon if necessary
	Point3d centroid;
private:
	// disable copying
	BoundablePolygon(const BoundablePolygon& copyMe);
	BoundablePolygon& operator=(const BoundablePolygon& assignMe);
	void triangulate();

public:
	BoundablePolygon(const SharedPolygon& poly);

	bool updateBV(BoundingVolume& bv) const;
	void getCentroid(Point3d& c) const;
	void getCovariance(const Point3d& centre, Matrix3d& cov) const;

	void setPolygon(const SharedPolygon& poly);

	double distanceToPoint(const Point3d& pnt, Point3d& nearest) const;
	double distanceToPoint(const Point3d& pnt, const Vector3d& dir,
			Point3d& nearest) const;

	double distanceToPoint(const Point3d& pnt, Point3d& nearest,
			Vector3d& bary, SharedPolygon& tri) const;

	double distanceToPoint(const Point3d& pnt, const Vector3d& dir,
			Point3d& nearest, Vector3d& bary, SharedPolygon& tri) const;

	const std::vector<SharedPolygon>& getTriangulation() const;

	// stored for faster access later
	void computeCentroid();
	void update();

};

bool poly_contains_coordinate(Point3d& pnt, const BoundablePolygon& bpoly,
        Vector3d& bary, SharedPolygon& tri);

struct InsideMeshQueryData {
public:
	InsideMeshQueryData();
	void reset();
public:
	bool on;
	bool in;
	bool unsure;
	SharedBoundablePolygon nearestFace;
	Point3d nearestPoint;
	int nHits;
	int nRetries;
};

// get generic tree
template<typename BV>
BVTree<SharedBoundablePolygon,BV>* get_bv_tree(const PolygonMesh& mesh, double margin = 0);

// specifics
BVTree<SharedBoundablePolygon,BoundingSphere>* get_bs_tree(const PolygonMesh& mesh, double margin = 0);
BVTree<SharedBoundablePolygon,AABB>* get_aabb_tree(const PolygonMesh& mesh, double margin = 0);
BVTree<SharedBoundablePolygon,OBB>* get_obb_tree(const PolygonMesh& mesh, double margin = 0);

struct NearestPolygonData {
	Point3d nearestPoint;
	std::vector<SharedBoundablePolygon> polygons;
	double tol;
};

// static methods
template<typename BV = OBB>
SharedPolygon nearest_polygon(const Point3d& pnt, const PolygonMesh& mesh,
		Point3d& nearestPoint);
template<typename BV = OBB>
SharedPolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
		const PolygonMesh& mesh, Point3d& nearestPoint);

template<typename BV>
SharedBoundablePolygon nearest_polygon(const Point3d& pnt,
        const BVTree<SharedBoundablePolygon,BV>& bvt,
		Point3d& nearestPoint);
template<typename BV>
SharedBoundablePolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
		const BVTree<SharedBoundablePolygon,BV>& bvt, Point3d& nearestPoint);

template<typename BV = OBB>
bool is_inside(const Point3d& pnt, const PolygonMesh& mesh, double tol = -1,
		int maxRetries = 10);
template<typename BV = OBB>
bool is_inside(const Point3d& pnt, const PolygonMesh& mesh,
		InsideMeshQueryData& data, double tol = -1, int maxRetries = 10);

template<typename BV>
bool is_inside(const Point3d& pnt, const BVTree<SharedBoundablePolygon,BV>& bvt,
        double tol = -1, int maxRetries = 10);
template<typename BV>
bool is_inside(const Point3d& pnt, const BVTree<SharedBoundablePolygon,BV>& bvt,
        InsideMeshQueryData& data, double tol = -1, int maxRetries = 10);

}
}

// template code
#include "mas/mesh/meshbv.hpp"

#endif
