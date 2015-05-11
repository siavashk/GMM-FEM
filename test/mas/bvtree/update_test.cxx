#include <stdio.h>
#include "mas/bvtree/bvtree.h"

using namespace mas;
using namespace mas::bvtree;

void do_update_test() {

	using BPS = BoundablePointPtrSet<IndexedPoint3d*>;
	IndexedPoint3d pnts[8];
	pnts[0].set(-1, -1, -1, 0);
	pnts[1].set( 1, -1, -1, 1);
	pnts[2].set( 1,  1, -1, 2);
	pnts[3].set(-1,  1, -1, 3);
	pnts[4].set(-1, -1,  1, 4);
	pnts[5].set( 1, -1,  1, 5);
	pnts[6].set( 1,  1,  1, 6);
	pnts[7].set(-1,  1,  1, 7);

	BPS bpps(0);
	for (int i=0; i<8; i++) {
		bpps.addPoint(pnts+i);
	}
	std::vector<BPS*> elems;
	elems.push_back(&bpps);

	BVTree<BPS*,AABB> aabb(elems);

	for (int i=0; i<8; i++) {
		pnts[i].z += 3;
	}
	aabb.update();

}

int main(int argc, char **argv) {
	do_update_test();
}
