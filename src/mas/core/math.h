#ifndef MAS_MATH_H
#define MAS_MATH_H

#include "mas/core/base.h"
#include <limits>

#ifndef MAS_MATH_MACHINE_PRECISION
#define MAS_MATH_MACHINE_PRECISION 5e-16
#endif

#ifndef MAS_MATH_SVD_ITERATIONS
#define MAS_MATH_SVD_ITERATIONS 100
#endif

namespace mas {
namespace math {

const double DOUBLE_INFINITY = std::numeric_limits<double>::infinity();

inline int signum(int a){ return (a > 0) ? 1 : ((a < 0) ? -1 : 0); }

// true if converged
bool svd3(const Matrix3d &A, Matrix3d &U, Vector3d &s, Matrix3d &V,
		size_t maxIters = MAS_MATH_SVD_ITERATIONS);

}
}

#endif
