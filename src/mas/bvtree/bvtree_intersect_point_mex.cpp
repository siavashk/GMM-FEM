#include "mas/bvtree/bvtree.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0
#define PNTS_IDX 1
#define RAD_IDX 2

#define IDXS_OUT 0

#define DIM 3

using namespace mas::bvtree;

void printNode(PBVNode node, int depth) {

	std::shared_ptr<OBB> obb = std::static_pointer_cast<OBB>(node->bv);
	mas::Point3d &c = obb->c;
	mas::Vector3d &hw = obb->halfWidths;

	for (int i=0; i<depth; i++) {
		printf("----");
	}
	printf("c: %.3lf %.3lf %.3lf\n", c.x, c.y, c.z);
	for (int i=0; i<depth; i++) {
		printf("    ");
	}
	printf("hw: %.3lf %.3lf %.3lf\n", hw.x, hw.y, hw.z);

	for (PBVNode child : node->children) {
		printNode(child, depth+1);
	}


}

void printTree(PBVTree obbt) {

	PBVNode root = obbt->getRoot();
	printNode(root, 0);

}


 // Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


    if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:bvtree_intersect_point_mex:invalidNumInputs",
                        "Must have at least 1 input.");
    }
    if (nlhs > 1){
        mexErrMsgIdAndTxt( "MATLAB:bvtree_destroy_mex:maxlhs",
                "Too many output arguments.");
    }

    // Get data
    mex::class_handle<BVTree> *tree = NULL;
    if (nrhs > TREE_IDX) {
    	tree = mex::get_class_handle<BVTree>(prhs[TREE_IDX]);

    	if (tree == NULL) {
    		mexPrintf("Unable to recover tree");
    	}

    	if (tree == NULL) {
    		mexErrMsgIdAndTxt( "MATLAB:bvtree_intersect_point_mex:invalidInput",
    		            "Cannot find BVTree with supplied id.");
    	}

    	// printTree(tree);

    } else {
        mexErrMsgIdAndTxt( "MATLAB:bvtree_intersect_point_mex:invalidInputType",
            "Expecting an integer id.");
    }

    // get Points
    // Get data
    double *pnts;
    int nPoints = 0;
    if (nrhs > PNTS_IDX && !mxIsEmpty(prhs[PNTS_IDX]) && mxIsDouble(prhs[PNTS_IDX])) {
    	pnts = mxGetPr(prhs[PNTS_IDX]);
    	nPoints = mxGetN(prhs[PNTS_IDX]);

    	int dim = mxGetM(prhs[PNTS_IDX]);
    	if (dim != DIM) {
    		mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:invalidInput",
    				"Point array must be of size 3xN.");
    	}
    } else {
    	mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:invalidInputType",
    			"Point array must be of type double.");
    }

    // radius
    double rad = 0;
    if (nrhs > RAD_IDX && !mxIsEmpty(prhs[RAD_IDX])
        && mxIsDouble(prhs[RAD_IDX])) {
        double *eptr = mxGetPr(prhs[RAD_IDX]);
        rad = eptr[0];
    }


    // build output
    mxArray *cells = mxCreateCellMatrix(1, nPoints);

	// #pragma omp parallel for
    for (int i=0; i<nPoints; i++) {
   		mas::Point3d pnt(pnts[3*i], pnts[3*i+1], pnts[3*i+2]);
   		PBVNodeList bvnodes;
   		tree->intersectSphere(pnt, rad, bvnodes);

   		// mexPrintf(" Found %i nodes that intersect point (%lf, %lf, %lf)\n", bvnodes.size(), pnt.x, pnt.y, pnt.z);

   		// count elems
   		int nelems = 0;
   		for (PBVNode node : bvnodes) {
   			nelems += node->numElements();
   		}

   		mxArray *elemIdxsArray = mxCreateDoubleMatrix(nelems, 1, mxREAL);
   		double *elemIdxs = mxGetPr(elemIdxsArray);

   		int eidx = 0;
   		for (PBVNode node : bvnodes) {
   			for (PBoundable elem : node->elems) {
   				std::shared_ptr<BoundablePointSet> bps = std::static_pointer_cast<BoundablePointSet>(elem);
   				elemIdxs[eidx++] = bps->idx;

   				//   				mexPrintf("Points (%i): \n", bps->pnts.size());
   				//   				for (mas::Point3d pnt : bps->pnts) {
   				//   					mexPrintf("  (%lf, %lf, %lf) ", pnt.x, pnt.y, pnt.z);
   				//   				}
   				//   				mexPrintf("\n");
   			}
   		}

   		mxSetCell(cells, i, elemIdxsArray);
    }

    if (nlhs > IDXS_OUT) {
    	plhs[0] = cells;
    }



}
