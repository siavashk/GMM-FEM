/*
 * matrixnd_test.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: antonio
 */

#include <iostream>
#include "mas/core/base.h"

void printmat(const mas::MatrixNd& m) {

   for (int i = 0; i < m.rows(); i++) {
      for (int j = 0; j < m.cols(); j++) {
         std::cout << m(i, j) << " ";
      }
      std::cout << std::endl;
   }

}

int main(int argc, char **argv) {

   mas::MatrixNd A(5, 3);
   int idx = 1;
   for (int i = 0; i < A.rows(); i++) {
      for (int j = 0; j < A.cols(); j++) {
         A(i, j) = idx++;
      }
   }

   printmat(A);
   std::cout << std::endl;
   A.transpose();
   printmat(A);
   std::cout << std::endl;

   mas::MatrixNd B = A;
   B.transpose();

   B.multiply(A);
   printmat(B);
   std::cout << std::endl;

}

