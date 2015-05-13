#include <math.h>

namespace mas {
namespace math {

template<typename Matrix, typename Vector>
void qr(const Matrix& A, Matrix& QR, Vector& tau) {
   QR = A; // copy data over
   qr(QR, tau);
}

template<typename Matrix, typename Vector>
void qr(Matrix& QR, Vector& tau) {

   size_t m = QR.rows();
   size_t n = QR.cols();
   size_t l = n;
   if (m < n) {
      l = m;
   }

   // temporary storage
   std::vector<double> tmp(n);

   // Do successive Householder transformations
   for (size_t j=0; j<l; j++) {

      // norm of x
      double normx = 0;
      for (size_t i=j; i<m; i++) {
         double x = QR(i, j);
         normx += x*x;
      }

      if (normx > 0) {
         normx = sqrt(normx);

         // sign
         int s = 1;
         if (QR(j,j) < 0) {
            s = -1;
         }

         double u1 = QR(j,j) + s*normx;

         // create Householder vector
         for (size_t i=j+1; i<m; i++) {
            QR(i, j) = QR(i, j)/u1;
         }
         tau(j) = s*u1/normx;

         // diagonal part of R
         QR(j,j) = -s*normx;

         // fill in rest of R
         // tau w'*A
         for (size_t k=j+1; k<n; k++) {
            tmp[k] = QR(j, k);
            for (size_t i=j+1; i<m; i++) {
               tmp[k] += QR(i, j)*QR(i, k);
            }
            tmp[k] = tmp[k]*tau(j);
         }

         // A = A-tau w (w'A)
         for (size_t k=j+1; k<n; k++) {
            QR(j, k) = QR(j, k) - tmp[k];
            for (size_t i=j+1; i<m; i++) {
               QR(i, k) = QR(i, k) - QR(i,j)*tmp[k];
            }
         }

      } else {
         tau(j) = 0;
      }

   }

}

} // math
} // mas
