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
        mexErrMsgIdAndTxt( "MATLAB:bvtree_destroy_mex:invalidNumInputs",
                        "Must have at least 1 input.");
    }
    if (nlhs > 0){
        mexErrMsgIdAndTxt( "MATLAB:bvtree_destroy_mex:maxlhs",
                "Too many output arguments.");
    }

    // Get data
    if (nrhs > TREE_IDX) {
    	// mexPrintf("Finding tree to cut down... \n");
    	mex::class_holder<PBVTree> *tree = mex::get_class_holder<PBVTree>(MESH_TREE_SIGNATURE, prhs[TREE_IDX]);
    	//    	mexPrintf("Destroying smesh tree\n");
    	if (tree->isValid(MESH_TREE_SIGNATURE)) {
    		delete tree;
			tree = NULL;
    	}
		// mexPrintf("No access violation :(\n");
    }

}
