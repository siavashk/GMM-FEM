#include "mas/bvtree/bvtree.h"
#include "mas/bvtree/bvtree_mex.h"
#include "mex.h"
#include "mas/mexhandle/mexhandle.h"
#include <math.h>
#include "mas/concurrency/thread.h"
#include <atomic>

#define TREE_IDX 0
#define PNTS_IDX 1
#define RAD_IDX 2
#define PARALLEL_CORES_IDX 3
#define PARALLEL_BLOCK_IDX 4

#define DEFAULT_PARALLEL_CORES std::max(std::thread::hardware_concurrency(), (unsigned int)2)
#define DEFAULT_PARALLEL_BLOCK 64

#define IDXS_OUT 0

#define DIM 3

using namespace mas::bvtree;
using SharedPoint = std::shared_ptr<mas::IndexedPoint3d>;
using BoundablePoints = mas::bvtree::BoundablePointPtrSet<SharedPoint>;
using SharedBoundablePoints = std::shared_ptr<BoundablePoints>;
using AABBTree = mas::bvtree::BVTree<SharedBoundablePoints, AABB>;

void __block_intersect(size_t start, size_t len, AABBTree* tree, double* pnts,
		double rad, mxArray* cells, std::mutex& cellmutex) {

	for (size_t i = start; i < start + len; ++i) {
		mas::Point3d pnt(pnts[3 * i], pnts[3 * i + 1], pnts[3 * i + 2]);
		std::vector<BVNode<SharedBoundablePoints, AABB>*> bvnodes;
		tree->intersectSphere(pnt, rad, bvnodes);

		// mexPrintf(" Found %i nodes that intersect point (%lf, %lf, %lf)\n", bvnodes.size(), pnt.x, pnt.y, pnt.z);

		// count elems
		int nelems = 0;
		for (BVNode<SharedBoundablePoints, AABB>* node : bvnodes) {
			nelems += node->numElements();
		}

		//		printf(" [start, len, i] = [%lu, %lu, %lu]\n", start, len, i);
		//		printf("  nelems = %i", nelems);
		//		fflush(stdout);

		mxArray *elemIdxsArray = nullptr;
		{
			// prevent segfault from Matlab memory management
			std::lock_guard<std::mutex> lock(cellmutex);
			elemIdxsArray = mxCreateDoubleMatrix(nelems, 1, mxREAL);
		}
		double *elemIdxs = mxGetPr(elemIdxsArray);

		//		std::vector<size_t>& elemIdxs = out[i];
		//		elemIdxs.reserve(nelems);

		 int eidx = 0;
		//   		mexPrintf("Point:  (%.2lf, %.2lf, %.2lf)\n", pnt.x, pnt.y, pnt.z);
		for (BVNode<SharedBoundablePoints, AABB>* node : bvnodes) {
			for (SharedBoundablePoints& elem : node->elems) {
				// elemIdxs.push_back(elem->idx);
				elemIdxs[eidx] = elem->idx+1; // +1 for matlab indexing
				eidx++;
			}
		}

		{
			// prevent segfault from Matlab memory management
			std::lock_guard<std::mutex> lock(cellmutex);
			mxSetCell(cells, i, elemIdxsArray);
		}
	}
}

void __thread_worker(std::atomic<size_t>& blocksRemaining, size_t nthreads,
		size_t threadIdx,
		size_t blockSize, AABBTree* tree, double* pnts, double rad,
		mxArray* cells, std::mutex& cellmutex) {

	bool complete = false;
	while (!complete) {
		size_t processBlock = --blocksRemaining;

		//		printf("Block: %lu\n", processBlock);
		//		fflush(stdout);
		// check for termination condition
		if (processBlock + nthreads < nthreads) {
			complete = true;
			break;
		}

		size_t blockFront = processBlock * blockSize;
		__block_intersect(blockFront, blockSize, tree, pnts, rad, cells, cellmutex);

	}
	//	printf(" closing worker %lu\n", threadIdx);
	//	fflush(stdout);

}

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

	// Get tree
	mex::class_handle<AABBTree> *tree = nullptr;
	if (nrhs > TREE_IDX) {
		tree = mex::get_class_handle<AABBTree>(POINTSET_TREE_SIGNATURE,
				prhs[TREE_IDX]);

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
	double *pnts = nullptr;
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

	// parallel cores
	int ncores = DEFAULT_PARALLEL_CORES;
	if (nrhs > PARALLEL_CORES_IDX && !mxIsEmpty(prhs[PARALLEL_CORES_IDX])
			&& mxIsDouble(prhs[PARALLEL_CORES_IDX])) {
		double *eptr = mxGetPr(prhs[PARALLEL_CORES_IDX]);
		ncores = ((int) eptr[0]);
	}

	// parallel blocksize
	int blockSize = DEFAULT_PARALLEL_BLOCK;
	if (nrhs > PARALLEL_BLOCK_IDX && !mxIsEmpty(prhs[PARALLEL_BLOCK_IDX])
			&& mxIsDouble(prhs[PARALLEL_BLOCK_IDX])) {
		double *eptr = mxGetPr(prhs[PARALLEL_BLOCK_IDX]);
		blockSize = (int) eptr[0];
	}

	// build output
	// initialize with a set number of points
	// std::vector<std::vector<size_t>> out(nPoints);
	mxArray *cells = mxCreateCellMatrix(1, nPoints);
	std::mutex cellmutex;

	// parallel intersect
	size_t nblocks = nPoints / blockSize;
	if (nPoints % blockSize != 0) {
		nblocks++; // one more for remainder
	}

	if (ncores <= 0) {
		ncores = std::max(std::thread::hardware_concurrency(),
				(unsigned int) 2); // try 2 threads if hardware_concurrency returns 0
	}

	// reserve ncores-1 threads
	size_t nthreads = std::min(nblocks, (size_t) ncores);
	std::vector<std::thread> threads;
	threads.reserve(nthreads - 1);

	// track number of blocks remaining
	std::atomic<size_t> blocksRemaining;
	blocksRemaining.store(nblocks);

	// current thread's work
	{
		size_t threadIdx = nthreads-1;
		size_t processBlock = --blocksRemaining;

		// front/back of last block
		size_t blockFront = processBlock * blockSize;
		size_t blockLength = nPoints - blockFront;

		// initialize other threads
		for (size_t i = 0; i < nthreads - 1; i++) {
			threads.push_back(
					std::thread(__thread_worker, std::ref(blocksRemaining),
							nthreads, i, blockSize, tree, pnts, rad, cells, std::ref(cellmutex)));
		}
		mas::concurrency::thread_group<std::vector<std::thread>> threadGroup(
				threads); // for closing off threads

		// process current block
		__block_intersect(blockFront, blockLength, tree, pnts, rad, cells, cellmutex);

		// continue processing other blocks
		__thread_worker(std::ref(blocksRemaining), nthreads, threadIdx, blockSize, tree,
				pnts, rad, cells, std::ref(cellmutex));
	}

	// Separately copy results into mxArray
	// NOTE: matlab's code is not thread-safe, trying
	//           to build mxArray in parallel leads to
	//           SEGFAULT... it looks like it tries
	//           to rebalance an internal tree, moving
	//           memory around
//	mxArray *cells = mxCreateCellMatrix(1, nPoints);
//	std::mutex mxarraym;
//
//	for (size_t i=0; i<out.size(); ++i) {
//		size_t rsize = out[i].size();
//
//		mxArray *elemIdxsArray = mxCreateDoubleMatrix(rsize, 1, mxREAL);
//		double *elemIdxs = mxGetPr(elemIdxsArray);
//		for (size_t j=0; j<rsize; j++) {
//			elemIdxs[j] = out[i][j] + 1; // add one for matlab indexing
//		}
//		mxSetCell(cells, i, elemIdxsArray);
//	}

	if (nlhs > IDXS_OUT) {
		plhs[0] = cells;
	}

}
