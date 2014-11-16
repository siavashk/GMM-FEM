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

/**
 * @brief Infinity
 *
 * Infinity value
 */
const double DOUBLE_INFINITY = std::numeric_limits<double>::infinity();

/**
 * @brief Standard signum function
 *
 * The signum function: f(x<0)=-1, f(x=0)=0, f(x>0)=1
 * @param a integer argument
 * @return the sign of the supplied integer
 */
inline int signum(int a){ return (a > 0) ? 1 : ((a < 0) ? -1 : 0); }

// true if converged
/**
 * @brief SVD for 3x3 systems
 *
 * SVD for 3x3 systems
 * @param A 3x3 matrix to analyze
 * @param U left singular vectors
 * @param s vector of singular values
 * @param V right singular vectors
 * @param maxIters
 * @return true if converged
 */
bool svd3(const Matrix3d &A, Matrix3d &U, Vector3d &s, Matrix3d &V,
		size_t maxIters = MAS_MATH_SVD_ITERATIONS);

}
}

#endif
