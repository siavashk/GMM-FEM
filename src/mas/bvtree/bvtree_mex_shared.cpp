#include "mas/bvtree/bvtree_mex_shared.h"

namespace mas {
namespace bvtree {
namespace mex {

BVTreeManager mxBVTreeManager;

BVTreeManager::BVTreeManager()
	: trees(), nidx(1) {}

BVTreeManager::~BVTreeManager() {
	clear();
}

int BVTreeManager::addTree(mas::bvtree::PBVTree addMe) {
	ManagedBVTree mtree;
	mtree.idx = nidx++;
	mtree.tree = addMe;
	trees.push_back(mtree);
	return mtree.idx;
}

mas::bvtree::PBVTree BVTreeManager::getTree(int idx) {
	for (ManagedBVTree m : trees) {
		if (m.idx == idx) {
			return m.tree;
		}
	}
	return NULL;
}

void BVTreeManager::removeTree(int idx) {

	std::vector<ManagedBVTree>::iterator it = trees.begin();
	do {
		if (it->idx == idx) {
			trees.erase(it);
			return;
		}
		it++;
	} while (it != trees.end());

}

void BVTreeManager::clear() {
	trees.clear();
	nidx = 1;
}

}
}
}
