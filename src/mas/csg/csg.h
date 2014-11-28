#ifndef MAS_CSG_H
#define MAS_CSG_H

#include "mas/mesh/mesh.h"
#include "mas/bvtree/bvtree.h"
#include <unordered_map>

#define MAS_CSG_DEFAULT_MAX_RAYCASTS 100

namespace mas {
namespace mesh {

class CSG {
private:
	double myTol;
	static double EPS;

private:

	static void computeInteriorPoint(const SharedPolygon& polygon,
			Point3d& pnt) const;

	static void sliceMeshes(const PolygonMesh& mesh1, const PolygonMesh& mesh2,
			std::vector<SharedPolygon>& outPolys1,
			std::vector<SharedPolygon> & outPolys2, double tol = -1,
			int maxRayCasts = MAS_CSG_DEFAULT_MAX_RAYCASTS);

	static void clipPolygons(const std::vector<SharedPolygon>& polys,
			const Plane& plane, std::vector<SharedPolygon>& front,
			std::vector<SharedPolygon>& back,
			std::vector<SharedPolygon>& coplanarFront,
			std::vector<SharedPolygon>& coplanarBack, double tol);

	static void clipPolygons(const std::vector<SharedPolygon>& polys,
			const std::vector<SharedPolygon>& cutters,
			std::vector<SharedPolygon>& out, double tol);

	static void doMeshSlicing(const BVTree& bvt1,
			std::unordered_map<SharedBVNode, std::vector<SharedPolygon> >& polyMap1,
			const BVTree& bvt2,
			std::unordered_map<SharedBVNode, std::vector<SharedPolygon> >& polyMap2,
			double tol = -1);

public:
	void setTolerance(double tol);
	double getTolerance() const;

	static double computeVolume(const std::vector<SharedVolume>& polys);
	static double computeArea(const std::vector<SharedVolume>& polys);
	static double computeDice(const std::vector<SharedVolume>& polys);

	/**
	 * @brief Fast method of intersection, but resulting mesh will not be closed or manifold.
	 *
	 * @param mesh1 first mesh for intersection
	 * @param mesh2 second mesh for intersection
	 * @param tol world-coordinate point tolerance
	 * @param maxRayCasts max iteration count for determining if a section of a polygon
	 *        is inside the other mesh
	 * @return
	 */
	static PolygonMesh* quickAndDirtyIntersection(const PolygonMesh& mesh1,
			const PolygonMesh& mesh2, double tol = -1, int maxRayCasts =
			MAS_CSG_DEFAULT_MAX_RAYCASTS);

	/**
	 * @brief Fast method of intersection, but resulting mesh will not be closed or manifold.
	 *
	 * @param mesh1 first mesh for intersection
	 * @param bvt1 bounding volume hierarchy for mesh1
	 * @param mesh2 second mesh for intersection
	 * @param bvt1 bounding volume hierarchy for mesh2
	 * @param tol world-coordinate point tolerance
	 * @param maxRayCasts max iteration count for determining if a section of a polygon
	 *        is inside the other mesh
	 * @return
	 */
	static PolygonMesh* quickAndDirtyIntersection(const PolygonMesh& mesh1,
			const BVTree& bvt1, const PolygonMesh& mesh2, const BVTree& bvt2,
			double tol = -1, int maxRayCasts = MAS_CSG_DEFAULT_MAX_RAYCASTS);

};

} // end mesh
} // end mas

#endif
