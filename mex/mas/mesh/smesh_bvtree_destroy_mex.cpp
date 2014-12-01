#include "mas/bvtree/bvtree.h"
#include "mas/mesh/smesh_bvtree_mex.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0

using namespace mas::bvtree;

// Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

    if (nrhs < 1) {
        mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_destroy:invalidNumInputs",
                "Must have at least 1 input.");
    }
    if (nlhs > 0) {
        mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_destroy:maxlhs",
                "Too many output arguments.");
    }

    // Get data
    if (nrhs > TREE_IDX) {
        // mexPrintf("Finding tree to cut down... \n");
        mex::class_handle<BVTree> *tree = mex::get_class_handle<BVTree>(
                MESH_TREE_SIGNATURE, prhs[TREE_IDX]);
        // mexPrintf("Destroying tree\n");
        if (tree->isValid(MESH_TREE_SIGNATURE)) {
            delete tree;
            tree = nullptr;
        }
        // mexPrintf("No access violation :(\n");
    }

}
