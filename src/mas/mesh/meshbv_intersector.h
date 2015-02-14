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

private:
	int check_min_max (
      const Vector3d& p1, const Vector3d& q1, const Vector3d& r1,
		const Vector3d& p2, const Vector3d& q2, const Vector3d& r2) const;
	int construct_intersection (
      const Vector3d& p1, const Vector3d& q1, const Vector3d& r1,
		const Vector3d& p2, const Vector3d& q2, const Vector3d& r2,
		std::vector<Point3d>& pnts) const;
	int tri_tri_inter_3d (
      const Vector3d& p1, const Vector3d& q1, const Vector3d& r1,
		const Vector3d& p2, const Vector3d& q2, const Vector3d& r2,
		double dp2, double dq2, double dr2, std::vector<Point3d>& pnts) const;
public:
	TriangleIntersector();
	TriangleIntersector(double eps);

	void setEpsilon(double eps);
	double getEpsilon() const;

	int intersectTriangleTriangle (
		const Vector3d& p1, const Vector3d& q1, const Vector3d& r1,
		const Vector3d& p2, const Vector3d& q2, const Vector3d& r2,
		std::vector<Point3d>& pnts) const;
	int intersectTriangleLine (
      const Point3d& v0, const Point3d& v1, const Point3d& v2,
	  const Point3d& pos, const Vector3d& dir, Vector3d& duv) const;
	int intersectTrianglePlane(const Point3d& p0, const Point3d& p1,
      const Point3d& p2, const Plane& plane, std::vector<Point3d>& pnts) const;
	double nearestpoint (
      const Point3d& v0, const Point3d& v1, const Point3d& v2, const Point3d& p,
		Point3d& closest, Vector3d& duv) const;

};

class TriangleLineIntersection {
public:
	SharedPolygon face;
	std::shared_ptr<Line> line;
	std::vector<Point3d> points;

public:
	TriangleLineIntersection(const SharedPolygon& face, const std::shared_ptr<Line>& line,
		std::vector<Point3d>&& points);
};

class TrianglePlaneIntersection {
public:
	SharedPolygon face;
	std::shared_ptr<Plane> plane;
	std::vector<Point3d> points;

public:
	TrianglePlaneIntersection(const SharedPolygon& face, const std::shared_ptr<Plane>& plane,
		std::vector<Point3d>&& pnts);
};

class TriangleTriangleIntersection {
public:
	SharedPolygon triangle0;
	SharedPolygon triangle1;
	std::vector<Point3d> points;

public:
	TriangleTriangleIntersection(const SharedPolygon& tri0, const SharedPolygon& tri1,
		std::vector<Point3d>&& pnts);
};

class BVIntersector {
private:  
 	double epsilon;
    TriangleIntersector myTriIntersector;

    // vectors for storing transformed triangle vertices
    Point3d myP0, myP1, myP2;
private:
    template<typename BV1, typename BV2>
	void intersectBoundingVolumeTriangles (
      std::vector<TriangleTriangleIntersection>& intersections,
      const BVNode<SharedBoundablePolygon,BV1>& node1,
      const BVNode<SharedBoundablePolygon,BV2>& node2) const;
    template<typename BV>
	void intersectBoundingVolumeTriangleLines (
      std::vector<TriangleLineIntersection>& intersections,
      const BVNode<SharedBoundablePolygon,BV>& node, const Line& l) const;
    template<typename BV>
	void intersectBoundingVolumeTrianglePlanes (
      std::vector<TrianglePlaneIntersection>& intersections,
      const BVNode<SharedBoundablePolygon,BV>& node, const Plane& p) const;

public:
	BVIntersector();
	BVIntersector(double epsilon);

	void setEpsilon(double eps);
	double getEpsilon();

	template<typename BV = OBB>
	std::vector<TriangleTriangleIntersection> intersectMeshMesh (
      const PolygonMesh& mesh1, const PolygonMesh& mesh2) const;

	template<typename BV1, typename BV2>
	std::vector<TriangleTriangleIntersection> intersectMeshMesh (
      const BVTree<SharedBoundablePolygon,BV1>& bvh1,
      const BVTree<SharedBoundablePolygon,BV2>& bvh2) const;

	template<typename BV = OBB>
    std::vector<TrianglePlaneIntersection> intersectMeshPlane (
      const PolygonMesh& mesh, const Plane& plane) const;
	template<typename BV>
    std::vector<TrianglePlaneIntersection> intersectMeshPlane (
      const BVTree<SharedBoundablePolygon,BV>& bvh, const Plane& plane) const;

	template<typename BV = OBB>
	std::vector<TriangleLineIntersection> intersectMeshLine (
	      const PolygonMesh& mesh, const Line& line) const;
	template<typename BV>
	std::vector<TriangleLineIntersection> intersectMeshLine (
    	const BVTree<SharedBoundablePolygon,BV>& bvh, const Line& line) const;

	
};

}
}

#include "mas/mesh/meshbv_intersector.hpp"

#endif
