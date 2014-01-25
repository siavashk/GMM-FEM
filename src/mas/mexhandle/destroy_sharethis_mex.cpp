#include "sharethis.h"
#include "mex.h"
#include "mexhandle.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

	if (nrhs > 0) {
		mex::class_handle<ShareThis> *share = mex::get_class_handle<ShareThis>(prhs[0]);
		delete share;
	}

}
