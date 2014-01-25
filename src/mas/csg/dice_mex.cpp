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

#include "mas/mesh/meshbv.h"
#include "mas/csg/csg.h"
#include "mex.h"
#include <math.h>

#define VERTS_1_IDX 0
#define FACES_1_IDX 1
#define VERTS_2_IDX 2
#define FACES_2_IDX 3
#define EPSILON_IDX 4

#define DICE_IDX 0
#define VOLI_IDX 1
#define VOL1_IDX 2
#define VOL2_IDX 3

#define DIM 3

 using mas::mesh::PolygonMesh;

 void fillMesh(PolygonMesh &mesh, double *verts, int nVerts, double *faceIdxs, 
 	int nVertsPerFace, int nFaces) {

 	using namespace mas::mesh;
 	PVertex3dList vtxList;
 	IndexListList faceIdxList;

 	for (int i=0; i<nVerts; i++) {
 		PVertex3d vtx = MeshFactory::createVertex(verts[i*DIM], verts[i*DIM+1],
 			verts[i*DIM+2], i);
 		vtxList.push_back(vtx);
 	}

 	for (int i=0; i<nFaces; i++) {
 		IndexList face;
 		for (int j=0; j<nVertsPerFace; j++) {
 			int vidx = (int)faceIdxs[i*nVertsPerFace+j]-1;
 			face.push_back(vidx);
 		}
 		faceIdxList.push_back(face);
 	}

 	mesh.set(vtxList, faceIdxList);

 }

 // Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

    double* verts1 = NULL;
    int nVerts1 = 0;

    double* faces1 = NULL;
    int nFaces1 = 0;
    int nFaceVerts1 = 3;

 	double* verts2 = NULL;
    int nVerts2 = 0;

    double* faces2 = NULL;
    int nFaces2 = 0;
    int nFaceVerts2 = 3;
   
   	//printf("Hello world");

    // Check number of arguments
    if (nrhs > 5) {
        mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidNumInputs",
                "Too many input arguments.");
    } else if (nrhs < 4) {
        mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidNumInputs",
                        "Must have at least 4 inputs.");
    }
    if (nlhs > 4){
        mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:maxlhs",
                "Too many output arguments.");
    }

    // Get data
    if (nrhs > VERTS_1_IDX && mxIsDouble(prhs[VERTS_1_IDX])) {
        verts1 = mxGetPr(prhs[VERTS_1_IDX]);
        nVerts1 = mxGetN(prhs[VERTS_1_IDX]);
        int dim = mxGetM(prhs[VERTS_1_IDX]);
        if (dim != DIM) {
            mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidInput",
                "Vertex array must be of size 3xN.");    
        }
    } else {
        mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidInputType",
            "Vertex array must be of type double.");
    }

    if (nrhs > FACES_1_IDX && mxIsDouble(prhs[FACES_1_IDX])) {
        faces1 = mxGetPr(prhs[FACES_1_IDX]);
        nFaceVerts1 = mxGetM(prhs[FACES_1_IDX]);
        nFaces1 = mxGetN(prhs[FACES_1_IDX]);
    } else {
        mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidInputType",
            "Faces array must be of type double (with integer values).");
    }

    if (nrhs > VERTS_2_IDX && mxIsDouble(prhs[VERTS_2_IDX])) {
        verts2 = mxGetPr(prhs[VERTS_2_IDX]);
        nVerts2 = mxGetN(prhs[VERTS_2_IDX]);
        int dim = mxGetM(prhs[VERTS_2_IDX]);
        if (dim != DIM) {
            mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidInput",
                "Vertex array must be of size 3xN.");    
        }
    } else {
        mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidInputType",
            "Vertex array must be of type double.");
    }

    if (nrhs > FACES_2_IDX && mxIsDouble(prhs[FACES_2_IDX])) {
        faces2 = mxGetPr(prhs[FACES_2_IDX]);
        nFaceVerts2 = mxGetM(prhs[FACES_2_IDX]);
        nFaces2 = mxGetN(prhs[FACES_2_IDX]);
    } else {
        mexErrMsgIdAndTxt( "MATLAB:csg_dice_mex:invalidInputType",
            "Faces array must be of type double (with integer values).");
    }

    double eps = -1;
    if (nrhs > EPSILON_IDX && !mxIsEmpty(prhs[EPSILON_IDX]) 
        && mxIsDouble(prhs[EPSILON_IDX])) {
        double *eptr = mxGetPr(prhs[EPSILON_IDX]);
        eps = eptr[0];
    }

    // smaller of two bounding radii, x 1e-12
    if (eps < 0) {

    	double c1[3] = {0,0,0};
    	double c2[3] = {0,0,0};
    	double r1 = 0;
    	double r2 = 0;

    	// estimate radius from vertices
    	for (int i=0; i<nVerts1; i++) {
    		c1[0] += verts1[DIM*i];
    		c1[1] += verts1[DIM*i+1];
    		c1[2] += verts1[DIM*i+2];
    	}
    	c1[0] = c1[0]/nVerts1;
    	c1[1] = c1[1]/nVerts1;
    	c1[2] = c1[2]/nVerts1;
    	for (int i=0; i<nVerts1; i++) {
    		double x = verts1[DIM*i] - c1[0];
    		double y = verts1[DIM*i+1] - c1[1];
    		double z = verts1[DIM*i+2] - c1[2];
    		x = sqrt(x*x + y*y + z*z);
    		if (x > r1) {
    			r1 = x;
    		}
    	}

    	// estimate radius from vertices
    	for (int i=0; i<nVerts2; i++) {
    		c2[0] += verts2[DIM*i];
    		c2[1] += verts2[DIM*i+1];
    		c2[2] += verts2[DIM*i+2];
    	}
    	c2[0] = c2[0]/nVerts2;
    	c2[1] = c2[1]/nVerts2;
    	c2[2] = c2[2]/nVerts2;
    	for (int i=0; i<nVerts2; i++) {
    		double x = verts2[DIM*i] - c2[0];
    		double y = verts2[DIM*i+1] - c2[1];
    		double z = verts2[DIM*i+2] - c2[2];
    		x = sqrt(x*x + y*y + z*z);
    		if (x > r2) {
    			r2 = x;
    		}
    	}

    	double r = r1;
    	if (r2 < r1) {
    		r = r2;
    	}

    	eps = r*1e-12;
    }

    // create meshes and do dice test
    PolygonMesh mesh1;
    PolygonMesh mesh2;

    fillMesh(mesh1, verts1, nVerts1, faces1, nFaceVerts1, nFaces1);
    fillMesh(mesh2, verts2, nVerts2, faces2, nFaceVerts2, nFaces2);

    using mas::bvtree::PAABBTree;
    PAABBTree bvtree1 = mas::mesh::get_aabb_tree(mesh1, eps);
    PAABBTree bvtree2 = mas::mesh::get_aabb_tree(mesh2, eps);

    double voli = mas::csg::intersection_volume(mesh1, bvtree1,
    	mesh2, bvtree2, eps);

    double vol1 = mesh1.volumeIntegral();
    double vol2 = mesh2.volumeIntegral();
    double dice = 2*voli/(vol1+vol2);
    if (dice > 1) {
    	dice = 1;
    }

    // fill outputs
    if (nlhs > DICE_IDX) {
    	plhs[DICE_IDX] = mxCreateDoubleScalar(dice);
    }
    if (nlhs > VOLI_IDX) {
    	plhs[VOLI_IDX] = mxCreateDoubleScalar(voli);
    }
    if (nlhs > VOL1_IDX) {
    	plhs[VOL1_IDX] = mxCreateDoubleScalar(vol1);
    }
    if (nlhs > VOL2_IDX) {
    	plhs[VOL2_IDX] = mxCreateDoubleScalar(vol2);
    }

}
