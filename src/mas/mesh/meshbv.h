#ifndef MAS_MESH_BV_H
#define MAS_MESH_BV_H

#include "mas/mesh/mesh.h"
#include "mas/bvtree/bvtree.h"

// adds bvtree functionality to mesh objects

#define MESH_INSIDE_ON 2
#define MESH_INSIDE_TRUE 1
#define MESH_INSIDE_FALSE 0
#define MESH_INSIDE_UNSURE -1

namespace mas {
namespace mesh {

// wrapper for polygon structure
class BoundablePolygon;
typedef std::shared_ptr<BoundablePolygon> PBoundablePolygon;

class BoundableFactory;

using namespace bvtree;

// wrapper for polygon
class BoundablePolygon : public Boundable {
public:
	PPolygon polygon;
	PPolygonList tris;	// triangulation of polygon if necessary
	Point3d centroid;
private:
	// disable copying
	BoundablePolygon(const BoundablePolygon &copyMe);
	BoundablePolygon &operator=(const BoundablePolygon &assignMe);
	void triangulate();
public:
	BoundablePolygon(const PPolygon poly);
	bool updateBV(BoundingVolume* bv) const;
	void getCentroid(Point3d &c) const;
	void getCovariance(const Point3d &centre, Matrix3d &cov) const;

	void setPolygon(const PPolygon poly);

	virtual double distanceToPoint(const Point3d &pnt,
			Point3d &nearest) const;
	virtual double distanceToPoint(const Point3d &pnt,
			const Vector3d &dir, Point3d &nearest ) const;

	virtual double distanceToPoint(const Point3d &pnt,
			Point3d &nearest, Polygon &tri, Vector3d &bary) const;
	virtual double distanceToPoint(const Point3d &pnt,
			const Vector3d &dir, Point3d &nearest,
			Polygon &tri, Vector3d &bary ) const;

	virtual PPolygonList getTriangulation() const;

	// stored for faster access later
	void computeCentroid();
	void update();

};

class BoundableFactory {
public:
	static PBoundablePolygon createBoundablePolygon(const PPolygon poly);
};

struct InsideMeshQueryData {
	InsideMeshQueryData() : on(false), in(false), unsure(false),
			nearestFace(NULL), nearestPoint(0,0,0), nHits(0), nRetries(0) {};
	bool on;
	bool in;
	bool unsure;
	PBoundablePolygon nearestFace;
	Point3d nearestPoint;
	int nHits;
	int nRetries;
};

// get generic tree
template<typename BV>
PBVTree get_bv_tree(const PolygonMesh &mesh, double margin = 0);

template<typename BV>
std::shared_ptr<BVTreeT<BV> > get_bv_tree_T(const PolygonMesh &mesh, double margin = 0);

// specifics
POBBTree get_obb_tree(const PolygonMesh &mesh, double margin = 0);
PAABBTree get_aabb_tree(const PolygonMesh &mesh, double margin = 0);
PBSTree get_bs_tree(const PolygonMesh &mesh, double margin = 0);

struct NearestPolygonData {
	Point3d nearestPoint;
	std::vector<PBoundablePolygon> polygons;
	double tol;
};

// static methods
PPolygon nearest_polygon(const Point3d &pnt, const PolygonMesh &mesh, Point3d &nearestPoint);
PPolygon nearest_polygon(const Point3d &pnt, const Vector3d &dir, const PolygonMesh &mesh, Point3d &nearestPoint);
PBoundablePolygon nearest_polygon(const Point3d &pnt, const PBVTree bvt, Point3d &nearestPoint);
PBoundablePolygon nearest_polygon(const Point3d &pnt, const Vector3d &dir, const PBVTree bvt, Point3d &nearestPoint);
PBoundablePolygon nearest_polygon(const Point3d &pnt, const Vector3d &dir, const PBVTree bvt, double tol,
		NearestPolygonData &data);

int is_inside(const Point3d &pnt, const PolygonMesh &mesh,
		double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);
int is_inside(const Point3d &pnt, const PolygonMesh &mesh,
		const PBVTree bvt, double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);
int is_inside_or_on(const Point3d &pnt, const PolygonMesh &mesh,
		const PBVTree bvt, double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);
int is_inside_or_on(const Point3d &pnt, const PolygonMesh &mesh,
		const PBVTree bvt, InsideMeshQueryData &data, double tol, int numRetries,
		double baryEpsilon);

bool is_inside(const Point3d &pnt, const PBVTree bvt, InsideMeshQueryData &data, double tol = -1, int maxRetries = 10);

}
}

// template code
#include "mas/mesh/meshbv.hpp"

#endif
