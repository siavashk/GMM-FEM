/*
 * heap_test_cmd.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: antonio
 */

#include <iostream>
#include <random>
#include "mas/core/base.h"

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
   printmat(m, "%g");
}

template<typename Vector>
void printvec(const Vector& v) {

   for (size_t i = 0; i < v.size(); i++) {
      std::cout << v[i] << " ";
   }
   std::cout << std::endl;
}

void doMatrix4dTest() {

   std::default_random_engine generator;
   std::uniform_real_distribution<double> distribution(-1.0, 1.0);

   mas::Matrix4d A;
   for (size_t i=0; i<A.rows(); i++) {
      for (size_t j=0; j<A.cols(); j++) {
         A(i,j) = distribution(generator);
      }
   }

   std::cout << "A: " << std::endl;
   printmat(A);

   mas::Matrix4d B = A;
   B.invert();

   std::cout << "A^(-1): " << std::endl;
   printmat(B);

   mas::Matrix4d C = B;
   C.multiply(A);
   std::cout << "A^(-1)A: " << std::endl;
   printmat(C);

   C = A;
   C.multiply(B);
   std::cout << "AA^(-1): " << std::endl;
   printmat(C);
}

int main(int argc, char **argv) {

   doMatrix4dTest();

}

