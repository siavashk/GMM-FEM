#ifndef MAS_CSG_H
#define MAS_CSG_H

#include "mas/mesh/mesh.h"
#include "mas/bvtree/bvtree.h"

namespace mas {
namespace csg {

using namespace mas::mesh;
using namespace mas::bvtree;

double intersection_volume(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		double tol = -1, int numRetries = 100, double baryEpsilon = 1e-12);

double dice(const PolygonMesh &mesh1, const PolygonMesh &mesh2,
		double tol = -1, int numRetries = 100, double baryEpsilon = 1e-12);

double dice(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		double tol = -1, int numRetries = 100, double baryEpsilon = 1e-12);

double dice_estimate(const PolygonMesh &mesh1, const PolygonMesh &mesh2,
		int resolution[], double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);

double dice_estimate(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		int resolution[], double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);

bool cheap_intersect(const PolygonMesh &mesh1, const PolygonMesh &mesh2,
		PolygonMesh &out, double tol = -1,
		int numRetries = 100, double baryEpsilon = 1e-12);

bool cheap_intersect(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		PolygonMesh &out,
		double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);

bool cheap_intersect(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		PPolygonList &out,
		double tol = -1, int numRetries = 100,
		double baryEpsilon = 1e-12);

}
}

#endif
