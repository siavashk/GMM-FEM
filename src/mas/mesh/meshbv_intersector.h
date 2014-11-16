#ifndef MAS_MESH_BV_INTERSECTOR_H
#define MAS_MESH_BV_INTERSECTOR_H

#include "mas/mesh/meshbv.h"

namespace mas {
namespace mesh {

class TriangleIntersector;
class TriangleLineIntersection;
class TrianglePlaneIntersection;
class TriangleTriangleIntersection;
class BVIntersector;

using namespace bvtree;

class TriangleIntersector {
private:
	double epsilon;

	double dp1, dq1, dr1, dp2, dq2, dr2;
   	Vector3d v1, v2, v, N1, N2, N;
	Vector2d P1, Q1, R1, P2, Q2, R2;
    double alpha;

	Vector3d edge0, edge1, tvec, pvec, qvec;
    double det, inv_det;

private:
	int check_min_max (
      const Vector3d &p1, const Vector3d &q1, const Vector3d &r1, 
		const Vector3d &p2, const Vector3d &q2, const Vector3d &r2);
	int construct_intersection (
      const Vector3d &p1, const Vector3d &q1, const Vector3d &r1, 
		const Vector3d &p2, const Vector3d &q2, const Vector3d &r2, 
		Point3d pnts[]);
	int tri_tri_inter_3d (
      const Vector3d &p1, const Vector3d &q1, const Vector3d &r1, 
		const Vector3d &p2, const Vector3d &q2, const Vector3d &r2, 
		double dp2, double dq2, double dr2, Point3d pnts[]);
public:
	TriangleIntersector();
	TriangleIntersector(double eps);
	void setEpsilon(double eps);
	double getEpsilon();
	int intersectTriangleTriangle (
		const Vector3d &p1, const Vector3d &q1, const Vector3d &r1, 
		const Vector3d &p2, const Vector3d &q2, const Vector3d &r2, Point3d pnts[]);
	int intersectTriangleLine (
      const Point3d &v0, const Point3d &v1, const Point3d &v2, 
	  const Point3d &pos, const Vector3d &dir, Vector3d &duv);
	int intersectTrianglePlane(const Point3d &p0, const Point3d &p1,
      const Point3d &p2, const Plane &plane, Point3d pnts[]);
	double nearestpoint (
      const Point3d &v0, const Point3d &v1, const Point3d &v2, const Point3d &p, 
		Point3d &closest, Vector3d &duv);

};

class TriangleLineIntersection {
public:
	PPolygon face;
	std::shared_ptr<Line> line;
	Point3d *points;
	int numPoints;

public:
	TriangleLineIntersection(PPolygon face, std::shared_ptr<Line> line, 
		Point3d pnts[], int numPoints);
	~TriangleLineIntersection();
};

class TrianglePlaneIntersection {
public:
	PPolygon face;
	std::shared_ptr<Plane> plane;
	Point3d *points;
	int numPoints;

public:
	TrianglePlaneIntersection(PPolygon face, std::shared_ptr<Plane> plane, 
		Point3d pnts[], int numPoints);
	~TrianglePlaneIntersection();
};

class TriangleTriangleIntersection {
public:
	PPolygon triangle0;
	PPolygon triangle1;
	Point3d *points;
	int numPoints;

public:
	TriangleTriangleIntersection(PPolygon tri0, PPolygon tri1, 
		Point3d pnts[], int numPoints);
	~TriangleTriangleIntersection();
};

class BVIntersector {
private:  
 	double epsilon;
    TriangleIntersector myTriIntersector;

    // vectors for storing transformed triangle vertices
    Point3d myP0, myP1, myP2;
private:
	void intersectBoundingVolumeTriangles (
      std::vector<TriangleTriangleIntersection> &intersections,
      PBVNode node1, PBVNode node2);
	void intersectBoundingVolumeTriangleLines (
      std::vector<TriangleLineIntersection> &intersections,
      PBVNode node, Line &l);
	void intersectBoundingVolumeTrianglePlanes (
      std::vector<TrianglePlaneIntersection> &intersections,
      PBVNode node, const Plane &p);

public:
	BVIntersector();
	BVIntersector(double epsilon);
	void setEpsilon(double eps);
	double getEpsilon();

	std::vector<TriangleTriangleIntersection> intersectMeshMesh (
      PolygonMesh &mesh1, PolygonMesh &mesh2);
	std::vector<TriangleTriangleIntersection> intersectMeshMesh (
      PBVTree bvh1, PBVTree bvh2);

    std::vector<TrianglePlaneIntersection> intersectMeshPlane (
      PolygonMesh &mesh, Plane &plane);
    std::vector<TrianglePlaneIntersection> intersectMeshPlane (
      PBVTree bvh, Plane &plane);

	std::vector<TriangleLineIntersection> intersectMeshLine (
    	PBVTree bvh, Line &line);
	std::vector<TriangleLineIntersection> intersectMeshLine (
      PolygonMesh &mesh, Line &line);
	
};

}
}

#endif
