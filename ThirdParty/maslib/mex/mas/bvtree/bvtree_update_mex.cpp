#include "mas/bvtree/bvtree.h"
#include "mas/bvtree/bvtree_mex.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0
#define PNTS_IDX 1

#define DIM 3

using namespace mas::bvtree;

void updateNode(BVNode& node, double *pnts) {

    // If leaf, update all elements
    if (node.isLeaf()) {

        for (SharedBoundable& elem : node.getElements()) {

            std::shared_ptr<IndexedBoundablePointSet> bps =
                    std::static_pointer_cast<IndexedBoundablePointSet>(elem);

            for (std::shared_ptr<mas::IndexedPoint3d>& bip : bps->pnts) {
                int i = bip->getIndex();
                bip->set(pnts[3 * i], pnts[3 * i + 1], pnts[3 * i + 2]);
            }

            // update bounds up
            node.updateBoundsUp(*bps);

        }

    } else {
        for (SharedBVNode& child : node.getChildren()) {
            updateNode(*child, pnts);
        }
    }

}

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
    mex::class_handle<BVTree> *tree = NULL;
    if (nrhs > TREE_IDX) {
        tree = mex::get_class_handle<BVTree>(POINTSET_TREE_SIGNATURE,
                prhs[TREE_IDX]);

        if (tree == NULL) {
            mexPrintf("Unable to recover tree");
        }

        if (tree == NULL) {
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
    	updateNode(*(tree->getRoot()), pnts);
    }

}
