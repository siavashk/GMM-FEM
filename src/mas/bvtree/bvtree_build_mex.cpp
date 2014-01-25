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
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>

#define PNTS_IDX 0
#define GROUP_IDX 1
#define TOL_IDX 2

#define TREE_IDX 0

#define DIM 3

using namespace mas::bvtree;

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

    std::vector<mas::Point3d> points;
    std::vector<PBoundable> boundableGroups;

    // Get data
    if (nrhs > PNTS_IDX && mxIsDouble(prhs[PNTS_IDX])) {
    	double *pnts = mxGetPr(prhs[PNTS_IDX]);
        int nPnts = mxGetN(prhs[PNTS_IDX]);
        points.reserve(nPnts);

        int dim = mxGetM(prhs[PNTS_IDX]);
        if (dim != DIM) {
            mexErrMsgIdAndTxt( "MATLAB:bvtree_mex:invalidInput",
                "Point array must be of size 3xN.");
        }

        for (int i=0; i<nPnts; i++) {
        	points.push_back(mas::Point3d(pnts[3*i], pnts[3*i+1], pnts[3*i+2]));
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

    			std::vector<mas::Point3d> pointset;
    			for (int m = 0; m<M; m++) {
    				pointset.push_back(points[ (int)p[m]-1 ]);
    			}
    			boundableGroups.push_back(std::make_shared<BoundablePointSet>(pointset, idx++));
    		}
    	} else if (mxIsDouble(prhs[GROUP_IDX])) {
    		double *dgroup = mxGetPr(prhs[GROUP_IDX]);
    		int M = mxGetM(prhs[GROUP_IDX]);
    		int N = mxGetN(prhs[GROUP_IDX]);

    		int idx = 1; // start at index 1 (matlab rules)

			for (int n = 0; n < N; n++) {
				std::vector<mas::Point3d> pointset;
				for (int m = 0; m<M; m++ ) {
					pointset.push_back( points[ (int)dgroup[n*M+m]-1 ] );
				}
				boundableGroups.push_back(std::make_shared<BoundablePointSet>(pointset, idx++));
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
    	for (mas::Point3d pnt : points) {
    		sum.add(pnt);
    	}
    	sum.scale(1.0/points.size());
    	double r = 0;
    	for (mas::Point3d pnt : points) {
    		double d = pnt.distance(sum);
    		if (d > r) {
    			r = d;
    		}
    	}

    	tol = r*1e-12;
    }

    // construct the actual BVTree using an AABB as a base
    POBB bv = BVFactory::createBV<AABB>();
    mex::class_handle<BVTree> *tree = new mex::class_handle<BVTree>(bv, boundableGroups, tol);

    // return tree
    if (nlhs > TREE_IDX) {
    	plhs[TREE_IDX] = mex::get_mex_handle<BVTree>(tree);
    }

}
