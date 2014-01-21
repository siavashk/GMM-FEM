#include "mas/bvtree/bvtree.h"
#include "mas/bvtree/bvtree_mex_shared.h"
#include "mex.h"
#include <math.h>

#define TREE_IDX 0

using namespace mas::bvtree;

 // Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


    if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:bvtree_destroy_mex:invalidNumInputs",
                        "Must have at least 1 input.");
    }
    if (nlhs > 0){
        mexErrMsgIdAndTxt( "MATLAB:bvtree_destroy_mex:maxlhs",
                "Too many output arguments.");
    }

    // Get data
    if (nrhs > TREE_IDX && mxIsDouble(prhs[TREE_IDX])) {

    	double *vals = mxGetPr(prhs[TREE_IDX]);
    	int id = (int)vals[0];
    	mas::bvtree::mex::mxBVTreeManager.removeTree(id);

    } else {
        mexErrMsgIdAndTxt( "MATLAB:bvtree_destroy_mex:invalidInputType",
            "Expecting an integer id.");
    }

}
