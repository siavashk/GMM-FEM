#include "mas/bvtree/bvtree.h"
#include "mas/bvtree/bvtree_mex.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0
#define PNTS_IDX 1

#define DIM 3

using namespace mas::bvtree;
using SharedPoint = std::shared_ptr<mas::IndexedPoint3d>;
using BoundablePoints = mas::bvtree::BoundablePointPtrSet<SharedPoint>;
using SharedBoundablePoints = std::shared_ptr<BoundablePoints>;
using AABBTree = mas::bvtree::BVTree<SharedBoundablePoints, AABB>;

// Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

    if (nrhs < PNTS_IDX) {
        mexErrMsgIdAndTxt("MATLAB:bvtree_update:invalidNumInputs",
                "Must have at least 1 input.");
    }
    if (nlhs > 0) {
        mexErrMsgIdAndTxt("MATLAB:bvtree_update:maxlhs",
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
            mexErrMsgIdAndTxt("MATLAB:bvtree_update:invalidInput",
                    "Cannot find BVTree with supplied id.");
        }

    } else {
        mexErrMsgIdAndTxt("MATLAB:bvtree_update:invalidInputType",
                "Expecting an integer id.");
    }

    // get updated points vector
    double *pnts = nullptr;
    if (nrhs > PNTS_IDX && !mxIsEmpty(prhs[PNTS_IDX])
            && mxIsDouble(prhs[PNTS_IDX])) {
        pnts = mxGetPr(prhs[PNTS_IDX]);

        int dim = mxGetM(prhs[PNTS_IDX]);
        if (dim != DIM) {
            mexErrMsgIdAndTxt("MATLAB:bvtree_update:invalidInput",
                    "Point array must be of size 3xN.");
        }
    } else {
        mexErrMsgIdAndTxt("MATLAB:bvtree_update:invalidInputType",
                "Point array must be of type double.");
    }

    // recursively update points
    if (pnts != nullptr) {

    	// Update all elements in leaves
    	for (size_t i = 0; i<tree->numLeaves(); i++) {
    		auto& node = tree->getLeaf(i);
    		for (SharedBoundablePoints& bps : node.elems) {
    			for (SharedPoint& bip : bps->pnts) {
    				int i = bip->getIndex();
    				bip->set(pnts[3 * i], pnts[3 * i + 1], pnts[3 * i + 2]);
    			}
    		}
    	}

    	tree->parallel_update();

    }

}
