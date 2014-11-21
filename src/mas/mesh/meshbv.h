#ifndef MAS_MESH_BV_H
#define MAS_MESH_BV_H

#include "mas/mesh/mesh.h"
#include "mas/bvtree/bvtree.h"

// adds bvtree functionality to mesh objects

namespace mas {
namespace mesh {

enum class MeshQueryResult {
	ON, INSIDE, OUTSIDE, UNSURE
};

// wrapper for polygon structure
class BoundablePolygon;
typedef std::shared_ptr<BoundablePolygon> SharedBoundablePolygon;

using namespace bvtree;

// wrapper for polygon
class BoundablePolygon: public Boundable {
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

	virtual double distanceToPoint(const Point3d& pnt, Point3d& nearest) const;
	virtual double distanceToPoint(const Point3d& pnt, const Vector3d& dir,
			Point3d& nearest) const;

	virtual double distanceToPoint(const Point3d& pnt, Point3d& nearest,
			Vector3d& bary, SharedPolygon& tri) const;

	virtual double distanceToPoint(const Point3d& pnt, const Vector3d& dir,
			Point3d& nearest, Vector3d& bary, SharedPolygon& tri) const;

	virtual const std::vector<SharedPolygon>& getTriangulation() const;

	// stored for faster access later
	void computeCentroid();
	void update();

};

struct InsideMeshQueryData {
	InsideMeshQueryData() :
			on(false), in(false), unsure(false), nearestFace(nullptr), nearestPoint(
					0, 0, 0), nHits(0), nRetries(0) {
	}
	void reset() {
		in = false;
		on = false;
		unsure = false;
		nHits = 0;
		nRetries = 0;
		nearestFace = nullptr;
		nearestPoint.setZero();
	}
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
BVTree* get_bv_tree(const PolygonMesh& mesh, double margin = 0);

template<typename BV>
BVTreeT<BV>* get_bv_tree_T(const PolygonMesh& mesh, double margin = 0);

// specifics
BSTree* get_bs_tree(const PolygonMesh& mesh, double margin = 0);
AABBTree* get_aabb_tree(const PolygonMesh& mesh, double margin = 0);
OBBTree* get_obb_tree(const PolygonMesh& mesh, double margin = 0);

struct NearestPolygonData {
	Point3d nearestPoint;
	std::vector<SharedBoundablePolygon> polygons;
	double tol;
};

// static methods
SharedPolygon nearest_polygon(const Point3d& pnt, const PolygonMesh& mesh,
		Point3d& nearestPoint);
SharedPolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
		const PolygonMesh& mesh, Point3d& nearestPoint);
SharedBoundablePolygon nearest_polygon(const Point3d& pnt, const BVTree& bvt,
		Point3d& nearestPoint);
SharedBoundablePolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
		const BVTree& bvt, Point3d& nearestPoint);
SharedBoundablePolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
		const BVTree& bvt, double tol, NearestPolygonData& data);

MeshQueryResult is_inside(const Point3d& pnt, const PolygonMesh& mesh,
		double tol = -1, int numRetries = 100, double baryEpsilon = 1e-12);
MeshQueryResult is_inside(const Point3d& pnt, const PolygonMesh& mesh,
		const BVTree& bvt, double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);
MeshQueryResult is_inside(const Point3d& pnt, const PolygonMesh& mesh,
        const BVTree& bvt, InsideMeshQueryData& data, double tol = -1,
        int numRetries = 100, double baryEpsilon = 1e-12);

MeshQueryResult is_inside_or_on(const Point3d& pnt, const PolygonMesh& mesh,
		const BVTree& bvt, double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);
MeshQueryResult is_inside_or_on(const Point3d& pnt, const PolygonMesh& mesh,
		const BVTree& bvt, InsideMeshQueryData& data, double tol,
		int numRetries, double baryEpsilon);

bool is_inside(const Point3d& pnt, const BVTree& bvt, InsideMeshQueryData& data,
		double tol = -1, int maxRetries = 10);

}
}

// template code
#include "mas/mesh/meshbv.hpp"

#endif
