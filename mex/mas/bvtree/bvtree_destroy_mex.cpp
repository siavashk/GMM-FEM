#include "mas/bvtree/bvtree.h"
#include "mas/bvtree/bvtree_mex.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0

using namespace mas::bvtree;

// Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

    if (nrhs < 1) {
        mexErrMsgIdAndTxt("MATLAB:bvtree_destroy:invalidNumInputs",
                "Must have at least 1 input.");
    }
    if (nlhs > 0) {
        mexErrMsgIdAndTxt("MATLAB:bvtree_destroy:maxlhs",
                "Too many output arguments.");
    }

    // Create shared indexed points
    using SharedPoint = std::shared_ptr<mas::IndexedPoint3d>;
    using BoundablePoints = mas::bvtree::BoundablePointPtrSet<SharedPoint>;
    using SharedBoundablePoints = std::shared_ptr<BoundablePoints>;
    using OBBTree = mas::bvtree::BVTree<SharedBoundablePoints, OBB>;

    // Get data
    if (nrhs > TREE_IDX) {
        // mexPrintf("Finding tree to cut down... \n");
        mex::class_handle<OBBTree> *tree = mex::get_class_handle<OBBTree>(
                POINTSET_TREE_SIGNATURE, prhs[TREE_IDX]);
        // mexPrintf("Destroying tree\n");
        if (tree->isValid(POINTSET_TREE_SIGNATURE)) {
            delete tree;
            tree = nullptr;
        }
        // mexPrintf("No access violation :(\n");
    }

}
