#include "mas/bvtree/bvtree.h"
#include "mas/bvtree/bvtree_mex.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0
#define PLANE_IDX 1

#define IDXS_OUT 0

#define DIM 3

using namespace mas::bvtree;
using SharedPoint = std::shared_ptr<mas::IndexedPoint3d>;
using BoundablePoints = mas::bvtree::BoundablePointPtrSet<SharedPoint>;
using SharedBoundablePoints = std::shared_ptr<BoundablePoints>;
using AABBTree = mas::bvtree::BVTree<SharedBoundablePoints, AABB>;

// Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

	if (nrhs < 2) {
		mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_plane:invalidNumInputs",
				"Must have at least 2 inputs.");
	}
	if (nlhs > 1) {
		mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_plane:maxlhs",
				"Too many output arguments.");
	}

	// Get tree
	mex::class_handle<AABBTree> *tree = nullptr;
	if (nrhs > TREE_IDX) {
		tree = mex::get_class_handle<AABBTree>(POINTSET_TREE_SIGNATURE,
				prhs[TREE_IDX]);

		if (tree == nullptr) {
			mexPrintf("Unable to recover tree");
		}

		if (tree == nullptr) {
			mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_plane:invalidInput",
					"Cannot find BVTree with supplied id.");
		}

		// printTree(tree);

	} else {
		mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_plane:invalidInputType",
				"Expecting an integer id.");
	}

	// get plane (a*x+b*y+c*z+d = 0)
	mas::Plane plane;
	if (nrhs > PLANE_IDX && !mxIsEmpty(prhs[PLANE_IDX])
			&& mxIsDouble(prhs[PLANE_IDX])) {
		double *vals = mxGetPr(prhs[PLANE_IDX]);
		int nvals = mxGetNumberOfElements(prhs[PLANE_IDX]);
		if (nval != 4) {
			mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_plane:invalidInput",
			"Plane input must consist of 4 values: [a,b,c,d] for a*x+b*y+c*z+d=0.");
		}
		plane.set(vals[0], vals[1], vals[2], vals[3]);

	} else {
		mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_plane:invalidInputType",
				"Plane must be of type double.");
	}

	std::vector<BVNode<SharedBoundablePoints, AABB>*> bvnodes;
	// intersect with plane
	tree->intersectPlane(plane, bvnodes);

	// count elems
	int nelems = 0;
	for (BVNode<SharedBoundablePoints, AABB>* node : bvnodes) {
		nelems += node->numElements();
	}

	mxArray *elemIdxsArray = mxCreateDoubleMatrix(nelems, 1, mxREAL);
	double *elemIdxs = mxGetPr(elemIdxsArray);

	int eidx = 0;
	for (BVNode<SharedBoundablePoints, AABB>* node : bvnodes) {
		for (SharedBoundablePoints& elem : node->elems) {
			elemIdxs[eidx] = elem->idx+1; // +1 for matlab indexing
			eidx++;
		}
	}

	if (nlhs > IDXS_OUT) {
		plhs[0] = elemIdxsArray;
	}

}
