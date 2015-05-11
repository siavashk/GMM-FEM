#include "mas/bvtree/bvtree.h"
#include "mex.h"
#include "mas/mesh/meshbv.h"
#include "mas/mesh/smesh_bvtree_mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define TREE_IDX 0
#define PNTS_IDX 1
#define TOL_IDX 2
#define RETRY_IDX 3
#define DEFAULT_RETRIES 100   // 100 retries if is_inside fails

#define INSIDE_OUT 0

#define DIM 3

using namespace mas::bvtree;

// Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

    if (nrhs < 1) {
        mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_is_inside:invalidNumInputs",
                "Must have at least 1 input.");
    }
    if (nrhs > 4) {
        mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_is_inside:invalidNumInputs",
                "Too many input argments (max 3).");
    }
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_is_inside:maxlhs",
                "Too many output arguments.");
    }

    // Get data
    // Get data
    mex::class_handle<BVTreeType> *tree = nullptr;
    if (nrhs > TREE_IDX) {
        tree = mex::get_class_handle<BVTreeType>(MESH_TREE_SIGNATURE, prhs[TREE_IDX]);

        if (tree == nullptr) {
            mexPrintf("Unable to recover tree");
        }

        if (tree == nullptr) {
            mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_is_inside:invalidInput",
                    "Cannot find BVTree with supplied id.");
        }

    } else {
        mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_is_inside:invalidInputType",
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
            mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_is_inside:invalidInput",
                    "Point array must be of size 3xN.");
        }
    } else {
        mexErrMsgIdAndTxt("MATLAB:smesh_bvtree_is_inside:invalidInputType",
                "Point array must be of type double.");
    }

    // get tolerance and retries
    double tol = -1;
    if (nrhs > TOL_IDX && !mxIsEmpty(prhs[TOL_IDX])
            && mxIsDouble(prhs[TOL_IDX])) {
        double *eptr = mxGetPr(prhs[TOL_IDX]);
        tol = eptr[0];
    }

    // get tolerance and retries
    int retries = -1;
    if (nrhs > RETRY_IDX && !mxIsEmpty(prhs[RETRY_IDX])
            && mxIsDouble(prhs[RETRY_IDX])) {
        double *eptr = mxGetPr(prhs[RETRY_IDX]);
        retries = (int) eptr[0];
    }

    // build output
    mxArray* inArray = mxCreateDoubleMatrix(nPoints, 1, mxREAL);
    double *in = mxGetPr(inArray);

    // #pragma omp parallel for
    for (int i = 0; i < nPoints; i++) {
        mas::Point3d pnt(pnts[3 * i], pnts[3 * i + 1], pnts[3 * i + 2]);

        mas::mesh::InsideMeshQueryData qdata;

        // determine whether is inside or out
        bool inside = mas::mesh::is_inside(pnt, *tree, qdata, tol, retries);
        if (!qdata.unsure) {
            if (inside) {
                in[i] = 1;
            } else {
                in[i] = 0;
            }
        } else {
            in[i] = -1; // unsure
        }
    }

    if (nlhs > INSIDE_OUT) {
        plhs[INSIDE_OUT] = inArray;
    }

}
