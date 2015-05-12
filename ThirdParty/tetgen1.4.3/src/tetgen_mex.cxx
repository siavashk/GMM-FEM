/*=================================================================
 * tetgen.cpp - matlab interface to tetgen
 *
 * Inputs:  vertices, [faces], [quality], [switches]
 * Outputs: nodes, faces, elements
 *
 * Copyright 2013 C. Antonio Sanchez <antonios@ece.ubc.ca>
 * $Revision: 0.0.0.1 $ 
 *  
 *=================================================================*/

#include <stdlib.h>
#include <stdio.h>

#ifndef TETLIBRARY
#define TETLIBRARY
#endif

#include "tetgen.h"
#include "mex.h"

// input/output indices
#define VERTS_IDX     0
#define FACES_IDX     1
#define QUALITY_IDX   2
#define SWITCH_IDX    3
#define NODE_IDX      0
#define TET_IDX       1
#define TRIFACE_IDX   2

#define DEFAULT_QUALITY 2

#define DIM               3
#define VERTS_PER_TRIFACE 3
#define NODES_PER_TET     4


// function definitions
void buildFromPoints(tetgenio* out, 
    double *coords, int numPnts, char *switches);

void buildFromMesh( tetgenio* out, 
    double *coords, int numPnts, double *faces, int numFaces,
    int numVertsPerFace, double quality, char *switches);

// Main entry function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    double* vertices = NULL;
    int numVertices = 0;

    double* faces = NULL;
    int numFaces = 0;
    int numVertsPerFace = 3;
    
    double quality = DEFAULT_QUALITY;
    char* switches = NULL;

    bool pointsOnly = true;

    // Check number of arguments
    if (nrhs > 4) {
        mexErrMsgIdAndTxt( "MATLAB:tetgen:invalidNumInputs",
                "Too many input arguments.");
    } else if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:tetgen:invalidNumInputs",
                        "Must have at least 1 inputs.");
    }
    if (nlhs > 3){
        mexErrMsgIdAndTxt( "MATLAB:tetgen:maxlhs",
                "Too many output arguments.");
    }

    // Get data
    if (nrhs > VERTS_IDX && mxIsDouble(prhs[VERTS_IDX])) {
        vertices = mxGetPr(prhs[VERTS_IDX]);
        numVertices = mxGetN(prhs[VERTS_IDX]);
        int dim = mxGetM(prhs[VERTS_IDX]);
        if (dim != DIM) {
            mexErrMsgIdAndTxt( "MATLAB:tetgen:invalidInput",
                "Vertex array must be of size 3xN.");    
        }
    } else {
        mexErrMsgIdAndTxt( "MATLAB:tetgen:invalidInputType",
            "Vertex array must be of type double.");
    }

    if (nrhs > FACES_IDX && !mxIsEmpty(prhs[FACES_IDX]) ) {
        pointsOnly = false;
    
        if (mxIsDouble(prhs[FACES_IDX])) {
            faces = mxGetPr(prhs[FACES_IDX]);
            numVertsPerFace = mxGetM(prhs[FACES_IDX]);
            numFaces = mxGetN(prhs[FACES_IDX]);
        } else if (!mxIsDouble(prhs[FACES_IDX])){
            mexErrMsgIdAndTxt( "MATLAB:tetgen:invalidInputType",
                "Faces array must be of type double (with integer values).");
        }
    }

    // quality
    if (nrhs > QUALITY_IDX && !mxIsEmpty(prhs[QUALITY_IDX]) 
        && mxIsDouble(prhs[QUALITY_IDX])) {
        double *qptr = mxGetPr(prhs[QUALITY_IDX]);
        quality = qptr[0];
    }

    // switches
    if (nrhs > SWITCH_IDX && !mxIsEmpty(prhs[SWITCH_IDX]) 
        && mxIsChar(prhs[SWITCH_IDX])) {
        switches = mxArrayToString(prhs[SWITCH_IDX]);
    }

    // output data structures
    tetgenio *out = new tetgenio();

    // do tetgen call
    try {
        if (pointsOnly) {
            buildFromPoints(out, vertices, numVertices, switches);
        } else {
            buildFromMesh(out, vertices, numVertices, faces, numFaces, 
                numVertsPerFace, quality, switches);
        }
    } catch (...) {
        // free memory
        delete out;
        if (switches != NULL) {
            mxFree(switches);
        }
        mexErrMsgIdAndTxt( "MATLAB:tetgen:tetgen_error",
                    "TetGen failed with an unknown error");
    } 

    // copy out results 
    // vertices
    if (nlhs > NODE_IDX) {
        int numPoints = out->numberofpoints;
        plhs[NODE_IDX]=mxCreateDoubleMatrix(DIM,numPoints,mxREAL);
        double *npntr = mxGetPr(plhs[NODE_IDX]);

        for (int i=0; i<DIM*numPoints; i++) {
            npntr[i] = out->pointlist[i];
        }
    }

    // elements
    if (nlhs > TET_IDX) {
        int numTets = out->numberoftetrahedra;
        plhs[TET_IDX]=mxCreateDoubleMatrix(NODES_PER_TET,numTets,mxREAL);
        double *npntr = mxGetPr(plhs[TET_IDX]);

        for (int i=0; i<NODES_PER_TET*numTets; i++) {
            npntr[i] = out->tetrahedronlist[i];
        }
    } 

    // hull
    if (nlhs > TRIFACE_IDX) {
        int numFaces = out->numberoftrifaces;
        plhs[TRIFACE_IDX]=mxCreateDoubleMatrix(VERTS_PER_TRIFACE,numFaces,mxREAL);
        double *npntr = mxGetPr(plhs[TRIFACE_IDX]);

        for (int i=0; i<VERTS_PER_TRIFACE*numFaces; i++) {
            npntr[i] = out->trifacelist[i];
        }
    }


    // free memory
    delete out;
    if (switches != NULL) {
        mxFree(switches);
    }
}

void buildFromPoints(tetgenio* out, 
    double *coords, int numPnts, char *switches) {

    tetgenio *in = new tetgenio();
    in->firstnumber = 1;            // because matlab patches start at 1
    in->numberofpoints = numPnts;

    out->firstnumber = 1;
   
    // copy vertices
    in->pointlist = new REAL[DIM*numPnts];
    for (int i=0; i<DIM*numPnts; i++) {
        in->pointlist[i] = coords[i];
    }

    // copy switches, default to Q
    char mySwitches[56];
    if (switches != NULL) {
        strncpy(mySwitches, switches, strlen(switches)+1);
    } else {
        strncpy(mySwitches, "Q", 2);  // default to Q switch
    }

    // tetgen call
    tetrahedralize (mySwitches, in, out);

    // cleanup
    delete in;
}

void buildFromMesh( tetgenio* out, 
    double *coords, int numPnts, double *faces, int numFaces,
    int numVertsPerFace, double quality, char *switches) {

    tetgenio *in = new tetgenio();
    in->firstnumber = 1;            // because matlab patches start at 1
    in->numberofpoints = numPnts;

    out->firstnumber = 1;
   
    // copy vertices
    in->pointlist = new REAL[DIM*numPnts];
    for (int i=0; i<DIM*numPnts; i++) {
        in->pointlist[i] = coords[i];
    }

    // copy faces
    in->numberoffacets = numFaces;
    in->facetlist = new tetgenio::facet[numFaces];
    in->facetmarkerlist = new int[numFaces];
        
    int k=0;
    for (int i=0; i<numFaces; i++) {
        in->facetmarkerlist[i] = 0;

        tetgenio::facet *f = &(in->facetlist[i]);
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = (REAL *) NULL;

        tetgenio::polygon *p = &(f->polygonlist[0]);
        p->numberofvertices = numVertsPerFace;
        p->vertexlist = new int[numVertsPerFace];
        for (int j=0; j<numVertsPerFace; j++) {
            p->vertexlist[j] = (int)faces[k++];
        }
    }

    // copy switches, default to Qp
    char mySwitches[56];
    if (switches != NULL) {
        strncpy(mySwitches, switches, strlen(switches)+1);

        // if doesn't contain p, add it
        bool hasP = false;
        for (size_t i=0; i<strlen(mySwitches); i++) {
            if (mySwitches[i]=='p') {
                hasP = true;
                break;
            }
        }
        if (!hasP) {
            strncat(mySwitches, "p", 1);
        }

    } else {
        strncpy(mySwitches, "Qp", 3);  // default to Qp switches
    }

    // add quality switch
    if (quality > 0) {
        char buf[16];
        sprintf (buf, "q%4.2f", quality);
        strncat(mySwitches, buf, strlen(buf));
    }

    // tetgen call
    tetrahedralize (mySwitches, in, out);

    // clear memory
    delete in;

}
