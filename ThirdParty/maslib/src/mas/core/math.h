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
inline int signum(double a){ return (a > 0) ? 1 : ((a < 0) ? -1 : 0); }

inline int sign(int a){ return (a < 0) ? -1 : 1; }
inline int sign(double a){ return (a < 0) ? -1 : 1; }

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
bool svd3(const Matrix3d& A, Matrix3d& U, Vector3d& s, Matrix3d& V,
		size_t maxIters = MAS_MATH_SVD_ITERATIONS);

/**
 * Compressed QR decomposition
 * @param A matrix to decompose
 * @param QR compressed storage of QR, with upper-right elements representing R,
 *        and tau(k)/2*[1, QR(k,k), ... ,QR(k,m)] the kth Householder flip vector
 * @param tau vector of scalars to fully reconstruct the QR-decomposition
 *
 * Matrix must support:
 * -rows(): # of rows
 * -cols(): # of columns
 * -(r, c): access element at (r, c) by reference
 *
 * Vector must support:
 * -size(): # of elements
 * -(idx): access to element [idx] by reference
 */
template<typename Matrix, typename Vector>
void qr(const Matrix& A, Matrix& QR, Vector& tau);

template<typename Matrix, typename Vector>
void qr(Matrix& QR, Vector& tau);

/*
template <typename Matrix, typename Vector>
class QR {
private:
   Matrix qrc;
   Vector tau;

public:
   void mulQ(const Vector& x, Vector& qx);
   void mulQt(const Vector& x, Vector& qtx);
   void solve(const Vector& b, Vector& x);

   void eigs(Vector& v);

   void determinant();
   void condition();

   void get(Matrix& Q, Matrix& R);
};
*/

} // math
} // mas

#include "mas/core/math.hpp"

#endif
