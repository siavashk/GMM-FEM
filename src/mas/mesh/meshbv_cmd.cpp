#include "mas/bvtree/bvtree.h"
#include "mas/mesh/meshbv.h"
#include <math.h>

using namespace mas::bvtree;
using namespace mas::mesh;

int main(int argc, const char* argv[]) {

	double pnts[] = {0.0, 0.0, -1.0,
			-1.0, 0.0, 0.0,
			0.0, -1.0, 0.0,
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0};
	double faces[] = {1, 2, 5,
			1, 5, 4,
			1, 4, 3,
			1, 3, 2,
			6, 5, 2,
			6, 4, 5,
			6, 2, 3,
			6, 3, 4 };

	int nPoints = 6;
	int nFaces = 8;

	std::vector<SharedVertex3d> vtxs;
	std::vector<SharedBoundablePolygon> boundableGroups;

	if (nPoints > 0) {
		vtxs.reserve(nPoints);
		int dim = 3;

		for (int i=0; i<nPoints; i++) {
			mas::Point3d loc(pnts[3*i], pnts[3*i+1], pnts[3*i+2]);
			SharedVertex3d vtx = std::make_shared<Vertex3d>(loc, i);
			vtxs.push_back(std::move(vtx));
		}
	}

	if (nFaces > 0) {
		int idx = 1; // start at index 1 (matlab rules)
		int N = nFaces;
		int M = 3;

		for (int n = 0; n < N; n++) {
		    std::vector<SharedVertex3d> polyvtxs;
			for (int m = 0; m<M; m++ ) {
				unsigned int gidx = (int)faces[n*M+m];
				if (gidx > vtxs.size()) {
					printf("Bad index!!");
					return -1;
				}
				polyvtxs.push_back( vtxs[ gidx-1 ] );
			}
			SharedPolygon poly = std::make_shared<Polygon>(std::move(polyvtxs));
			printf("Poly created, %ld verts\n", poly->verts.size());
			SharedBoundablePolygon bpoly = std::make_shared<BoundablePolygon>(std::move(poly));
			bpoly->polygon->setIndex(idx++);

			if (bpoly->polygon.get() != poly.get()) {
				printf("What's going on?!?!\n");
				printf("Polygon: %p\nBPolygon: %p\n", poly.get(), bpoly->polygon.get());
			}
			printf("Bounded polygon has %ld verts (%ld)\n", bpoly->polygon->verts.size(), poly->verts.size());
			fflush(stdout);
			boundableGroups.push_back(bpoly);
		}
	}

	int pidx = 1;
	for (SharedBoundablePolygon& b : boundableGroups) {
		SharedBoundablePolygon bpoly = std::static_pointer_cast<BoundablePolygon>(b);
		printf("Polygon %d, %ld verts\n", pidx++, bpoly->polygon->verts.size());
		fflush(stdout);
	}

	// construct the actual BVTree using an OBB as a base
	auto tree = std::unique_ptr<BVTree<SharedBoundablePolygon,OBB>>(new BVTree<SharedBoundablePolygon,OBB>(boundableGroups, 1e-15));

	return 0;
}
