/*
 * heap_test_cmd.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: antonio
 */

#include <iostream>
#include <random>
#include "mas/core/base.h"
#include "mas/core/math.h"

size_t safeprintd(std::vector<char>& cbuff, size_t pos, const char* fmt,
      double d) {
   int len = snprintf(cbuff.data() + pos, cbuff.size() - pos, fmt, d);
   if (len >= 0) {
      if ((unsigned)len >= cbuff.size()) {
         cbuff.resize(pos + len + 1);
         len = snprintf(cbuff.data() + pos, cbuff.size() - pos, fmt, d);
      }
   }
   return pos + len;
}

std::string safeprintd(const char* fmt, double d) {
   std::vector<char> cbuff(80);
   safeprintd(cbuff, 0, fmt, d);
   return std::string(cbuff.data());
}

template<typename Matrix>
void printmat(const Matrix& m, const char* fmt) {

   for (size_t i = 0; i < m.rows(); i++) {
      for (size_t j = 0; j < m.cols(); j++) {
         std::cout << safeprintd(fmt, m(i, j)) << " ";
      }
      std::cout << std::endl;
   }
}

template<typename Matrix>
void printmat(const Matrix& m) {
   printmat(m, "%f");
}

template<typename Vector>
void printvec(const Vector& v) {

   for (size_t i = 0; i < v.size(); i++) {
      std::cout << v[i] << " ";
   }
   std::cout << std::endl;
}

void doRandomQRTest(size_t m, size_t n) {

   std::default_random_engine generator;
   std::uniform_real_distribution<double> distribution(-1.0, 1.0);

   mas::MatrixNd A(m, n);
   mas::MatrixNd QR(m, n);
   mas::VectorNd tau(n);

   mas::math::qr(A, QR, tau);

   printmat(QR);

}

void doBasicQRTest() {

   mas::MatrixNd A(5, 3);
   int idx = 1;
   for (size_t i = 0; i < A.rows(); i++) {
      for (size_t j = 0; j < A.cols(); j++) {
         A(i, j) = idx++;
      }
   }

   mas::MatrixNd QR(A.rows(), A.cols());
   mas::VectorNd tau(A.cols());

   mas::math::qr(A, QR, tau);

   std::cout << "A: " << std::endl;
   printmat(A);
   std::cout << "QR: " << std::endl;
   printmat(QR);
   std::cout << "T: " << std::endl;
   printvec(tau);
   std::cout << std::endl;

}

int main(int argc, char **argv) {

   doBasicQRTest();
   // doRandomQRTest(5, 3);
   // doRandomQRTest(3, 5);

}

