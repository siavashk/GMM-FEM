#include "sharethis.h"
#include "mex.h"
#include "mexhandle.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

	// grab first input for 'id'
	if (nrhs > 0) {
	    double *eptr = mxGetPr(prhs[0]);
	    int id = (int)eptr[0];

	    // create and return
	    mex::class_handle<ShareThis> *share = new mex::class_handle<ShareThis>(id);
	    mxArray *out = mex::get_mex_handle<ShareThis>(share);

	    plhs[0] = out;
	}


}
