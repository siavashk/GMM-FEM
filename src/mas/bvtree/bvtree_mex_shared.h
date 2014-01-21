#ifndef MAS_BVTREE_SHARED_H
#define MAS_BVTREE_SHARED_H

#include "mas/bvtree/bvtree.h"
#include <vector>

namespace mas {
namespace bvtree {
namespace mex {

struct ManagedBVTree {
	mas::bvtree::PBVTree tree;
	int idx;
};


class BVTreeManager {
private:
	int nidx;
	std::vector<ManagedBVTree> trees;
private:
	BVTreeManager(const BVTreeManager &manager);
public:
	BVTreeManager();
	~BVTreeManager();
	int addTree(mas::bvtree::PBVTree addMe);
	mas::bvtree::PBVTree getTree(int idx);
	void removeTree(int idx);
	void clear();

	int getNumTrees() { return trees.size(); }
};

extern BVTreeManager mxBVTreeManager;

}
}
}

#endif
