#include "mas/bvtree/bvtree.h"
#include "mas/bvtree/bvtree_mex.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0
#define PNTS_IDX 1
#define RAD_IDX 2

#define IDXS_OUT 0

#define DIM 3

using namespace mas::bvtree;

// Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

    if (nrhs < 1) {
        mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_point:invalidNumInputs",
                "Must have at least 1 input.");
    }
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_point:maxlhs",
                "Too many output arguments.");
    }

    using SharedPoint = std::shared_ptr<mas::IndexedPoint3d>;
    using BoundablePoints = mas::bvtree::BoundablePointPtrSet<SharedPoint>;
    using SharedBoundablePoints = std::shared_ptr<BoundablePoints>;
    using AABBTree = mas::bvtree::BVTree<SharedBoundablePoints, AABB>;

    // Get tree
    mex::class_handle<AABBTree> *tree = nullptr;
    if (nrhs > TREE_IDX) {
        tree = mex::get_class_handle<AABBTree>(POINTSET_TREE_SIGNATURE, prhs[TREE_IDX]);

        if (tree == nullptr) {
            mexPrintf("Unable to recover tree");
        }

        if (tree == nullptr) {
            mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_point:invalidInput",
                    "Cannot find BVTree with supplied id.");
        }

        // printTree(tree);

    } else {
        mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_point:invalidInputType",
                "Expecting an integer id.");
    }

    // get Points
    // Get data
    double *pnts;
    int nPoints = 0;
    if (nrhs > PNTS_IDX && !mxIsEmpty(prhs[PNTS_IDX])
            && mxIsDouble(prhs[PNTS_IDX])) {
        pnts = mxGetPr(prhs[PNTS_IDX]);
        nPoints = mxGetN(prhs[PNTS_IDX]);

        int dim = mxGetM(prhs[PNTS_IDX]);
        if (dim != DIM) {
            mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_point:invalidInput",
                    "Point array must be of size 3xN.");
        }
    } else {
        mexErrMsgIdAndTxt("MATLAB:bvtree_intersect_point:invalidInputType",
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
    for (int i = 0; i < nPoints; i++) {
        mas::Point3d pnt(pnts[3 * i], pnts[3 * i + 1], pnts[3 * i + 2]);
        std::vector<BVNode<SharedBoundablePoints,AABB>*> bvnodes;
        tree->intersectSphere(pnt, rad, bvnodes);

        // mexPrintf(" Found %i nodes that intersect point (%lf, %lf, %lf)\n", bvnodes.size(), pnt.x, pnt.y, pnt.z);

        // count elems
        int nelems = 0;
        for (BVNode<SharedBoundablePoints,AABB>* node : bvnodes) {
            nelems += node->numElements();
        }

        mxArray *elemIdxsArray = mxCreateDoubleMatrix(nelems, 1, mxREAL);
        double *elemIdxs = mxGetPr(elemIdxsArray);

        int eidx = 0;

        //   		mexPrintf("Point:  (%.2lf, %.2lf, %.2lf)\n", pnt.x, pnt.y, pnt.z);
        for (BVNode<SharedBoundablePoints,AABB>* node : bvnodes) {
            for (SharedBoundablePoints& elem : node->elems) {
                elemIdxs[eidx++] = (elem->idx + 1); // add one for matlab indexing
            }
        }

        mxSetCell(cells, i, elemIdxsArray);
    }

    if (nlhs > IDXS_OUT) {
        plhs[0] = cells;
    }

}
