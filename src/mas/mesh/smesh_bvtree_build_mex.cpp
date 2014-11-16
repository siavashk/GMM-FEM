/*=================================================================
 * csg_dice_mex.cpp - matlab interface to csg for computing dice
 *
 * Inputs:  vertices #1, faces #1, vertices #2, faces #2, [epsilon]
 * Outputs: dice, intersection volume, vol #1, vol #2
 *
 * Copyright 2013 C. Antonio Sanchez <antonios@ece.ubc.ca>
 * $Revision: 0.0.0.1 $ 
 *  
 *=================================================================*/

#include "mas/bvtree/bvtree.h"
#include "mas/mesh/smesh_bvtree_mex.h"
#include "mas/mesh/meshbv.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define PNTS_IDX 0
#define GROUP_IDX 1
#define TOL_IDX 2

#define TREE_IDX 0

#define DIM 3

using namespace mas::bvtree;
using namespace mas::mesh;

 // Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


    if (nrhs < 2) {
        mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:invalidNumInputs",
                        "Must have at least 2 inputs.");
    }
    if (nlhs > 1){
        mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:maxlhs",
                "Too many output arguments.");
    }

    PVertex3dList vtxs;
    std::vector<PBoundable> boundableGroups;

    // Get data
    if (nrhs > PNTS_IDX && mxIsDouble(prhs[PNTS_IDX])) {
    	double *pnts = mxGetPr(prhs[PNTS_IDX]);
        int nPnts = mxGetN(prhs[PNTS_IDX]);
        vtxs.reserve(nPnts);

        int dim = mxGetM(prhs[PNTS_IDX]);
        if (dim != DIM) {
            mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:invalidInput",
                "Point array must be of size 3xN.");
        }

        for (int i=0; i<nPnts; i++) {
        	mas::Point3d loc(pnts[3*i], pnts[3*i+1], pnts[3*i+2]);
        	PVertex3d vtx = MeshFactory::createVertex(loc);
        	vtx->idx = i;
        	vtxs.push_back(vtx);
        }

    } else {
        mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:invalidInputType",
            "Point array must be of type double.");
    }


    if (nrhs > GROUP_IDX) {

    	if (mxIsCell(prhs[GROUP_IDX])) {

    		double  *p;
    		mxArray *cellElement;
    		int N = mxGetNumberOfElements(prhs[GROUP_IDX]);

    		int idx = 1;	// start at index 1 (matlab rules)
    		for (int i=0; i<N; i++) {
    			cellElement = mxGetCell(prhs[GROUP_IDX],i);
    			p = mxGetPr(cellElement);
    			int M = mxGetNumberOfElements(cellElement);

    			PVertex3dList polyvtxs;
    			for (int m = 0; m<M; m++) {
    				polyvtxs.push_back(vtxs[ (int)p[m]-1 ]);
    			}

    			PPolygon poly = MeshFactory::createPolygon(polyvtxs);
    			PBoundablePolygon bpoly = std::make_shared<mas::mesh::BoundablePolygon>(poly);
    			bpoly->setIndex(idx++);
    			boundableGroups.push_back(bpoly);
    		}

    	} else if (mxIsDouble(prhs[GROUP_IDX])) {
    		double *dgroup = mxGetPr(prhs[GROUP_IDX]);
    		int M = mxGetM(prhs[GROUP_IDX]);
    		int N = mxGetN(prhs[GROUP_IDX]);

    		int idx = 1; // start at index 1 (matlab rules)

			for (int n = 0; n < N; n++) {
				PVertex3dList polyvtxs;
				for (int m = 0; m<M; m++ ) {
					unsigned int gidx = (int)dgroup[n*M+m];
					if (gidx > vtxs.size()) {
						mexPrintf("Bad index!!");
						return;
					}
					polyvtxs.push_back( vtxs[ gidx-1 ] );
				}
				PPolygon poly = MeshFactory::createPolygon(polyvtxs);
				PBoundablePolygon bpoly = std::make_shared<mas::mesh::BoundablePolygon>(poly);
				bpoly->setIndex(idx++);
				boundableGroups.push_back(bpoly);
			}
    	} else {
            mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:invalidInputType",
                "Groups of points must be an array or cell array of doubles (with integer values).");
        }
    }

    double tol = -1;
    if (nrhs > TOL_IDX && !mxIsEmpty(prhs[TOL_IDX])
        && mxIsDouble(prhs[TOL_IDX])) {
        double *eptr = mxGetPr(prhs[TOL_IDX]);
        tol = eptr[0];
    }

    if (tol < 0) {
    	mas::Point3d sum(0,0,0);
    	for (PVertex3d vtx : vtxs) {
    		sum.add(*vtx);
    	}
    	sum.scale(1.0/vtxs.size());
    	double r = 0;
    	for (PVertex3d vtx : vtxs) {
    		double d = vtx->distance(sum);
    		if (d > r) {
    			r = d;
    		}
    	}

    	tol = r*1e-12;
    }

    // construct the actual BVTree using an OBB as a base
    PBVTree tree = BVTreeFactory::createTree<OBB>(boundableGroups, tol);
    mex::class_holder<PBVTree> *treeHolder = new mex::class_holder<PBVTree>(MESH_TREE_SIGNATURE, tree);

    // return tree
    if (nlhs > TREE_IDX) {
    	mxArray* mhandle = mex::get_mex_handle<PBVTree>(treeHolder);
    	plhs[TREE_IDX]  = mhandle;

    	// recover
    	mex::class_holder<PBVTree>* treeHolderTest = mex::get_class_holder<PBVTree>(MESH_TREE_SIGNATURE, mhandle);
    	if (!treeHolderTest->isValid(MESH_TREE_SIGNATURE)) {
    		mexPrintf("Signature invalid.\n");
    	}

    }

}
