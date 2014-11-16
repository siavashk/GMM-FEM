#include "mas/core/base.h"
#include "mas/core/math.h"
#include <math.h>
#include <algorithm>

namespace mas {

// Vector implementation
const Vector3d Vector3d::ZERO(0,0,0);
const Vector3d Vector3d::X_AXIS(1,0,0);
const Vector3d Vector3d::Y_AXIS(0,1,0);
const Vector3d Vector3d::Z_AXIS(0,0,1);

Vector3d::Vector3d()
: x(0), y(0), z(0) {}

Vector3d::Vector3d(const Vector3d& copyMe)
: x(copyMe.x), y(copyMe.y), z(copyMe.z) {}

Vector3d::Vector3d(double x, double y, double z)
: x(x), y(y), z(z) {}

Vector3d& Vector3d::operator=(const Vector3d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   z = assignMe.z;
   return *this;
}

void Vector3d::set(const Vector3d& pnt) {
   x = pnt.x;
   y = pnt.y;
   z = pnt.z;
}

void Vector3d::set(double x, double y, double z) {
   this->x = x;
   this->y = y;
   this->z = z;
}

void Vector3d::set(int idx, double val) {
   switch ( idx ) {
   case 0:
      x = val;
      return;
   case 1:
      y = val;
      return;
   case 2:
      z = val;
      return;
   }
}

void Vector3d::setZero() {
   x = 0;
   y = 0;
   z = 0;
}

double Vector3d::get(int idx) const {
   switch ( idx ) {
   case 0:
      return x;
   case 1:
      return y;
   case 2:
      return z;
   }
   return 0;
}

void Vector3d::add(const Vector3d& v) {
   x += v.x;
   y += v.y;
   z += v.z;
}

void Vector3d::add(double x, double y, double z) {
   this->x += x;
   this->y += y;
   this->z += z;
}

/**
 * @brief Adds two vectors
 *
 * Replaces the contents of this vector with v1+v2
 * @param v1
 * @param v2
 */
void Vector3d::add(const Vector3d& v1, const Vector3d& v2) {
   x = v1.x + v2.x;
   y = v1.y + v2.y;
   z = v1.z + v2.z;
}

void Vector3d::subtract(const Vector3d& v) {
   x -= v.x;
   y -= v.y;
   z -= v.z;
}

/**
 * @brief Subtracts v1-v2
 *
 * Replaces contents of this vector with v1-v2
 * @param v1
 * @param v2
 */
void Vector3d::subtract(const Vector3d& v1, const Vector3d& v2) {
   x = v1.x - v2.x;
   y = v1.y - v2.y;
   z = v1.z - v2.z;
}


/**
 * @brief v1 + s*v2
 *
 * Replaces contents of this vector with v1+s*v2
 *
 * @param v1
 * @param s scale factor
 * @param v2 scaled vector
 */
void Vector3d::scaledAdd(const Vector3d& v1, double s,
      const Vector3d& v2) {
   x = v1.x + s*v2.x;
   y = v1.y + s*v2.y;
   z = v1.z + s*v2.z;
}

/**
 * @brief adds s*v
 *
 * Adds s*v to this vector
 *
 * @param s scale factor
 * @param v scaled vector
 */
void Vector3d::scaledAdd(double s, const Vector3d& v) {
   scaledAdd(*this, s, v);
}

void Vector3d::scale(double s) {
   x *= s;
   y *= s;
   z *= s;
}

/**
 * @brief s*v
 *
 * Sets contents of this vector to s*v
 *
 * @param s scale factor
 * @param v scaled vector
 */
void Vector3d::scale(double s, const Vector3d& v) {
   x = s*v.x;
   y = s*v.y;
   z = s*v.z;
}

void Vector3d::negate() {
   x = -x;
   y = -y;
   z = -z;
}

double Vector3d::dot(double x, double y, double z) const {
   return (this->x*x + this->y*y + this->z*z);
}

double Vector3d::dot(const Vector3d& v) const {
   return (x*v.x+y*v.y+z*v.z);
}

// cross product in safe way, so v1 or v2 can be 'this'
/**
 * @brief v1 x v2
 *
 * Sets contents of this vector to the cross product of v1 x v2
 *
 * @param v1
 * @param v2
 */
void Vector3d::cross(const Vector3d& v1, const Vector3d& v2) {
   double tmpx = v1.y * v2.z - v1.z * v2.y;
   double tmpy = v1.z * v2.x - v1.x * v2.z;
   double tmpz = v1.x * v2.y - v1.y * v2.x;
   x = tmpx;
   y = tmpy;
   z = tmpz;
}

/**
 * @brief this x v
 *
 * Takes the cross product: this x v
 *
 * @param v right-hand vector
 */
void Vector3d::cross(const Vector3d& v) {
   cross(*this, v);
}

double Vector3d::norm() const {
   return sqrt(x*x+y*y+z*z);
}

double Vector3d::normSquared() const {
   return (x*x+y*y+z*z);
}

// return false if norm is zero
/**
 * @brief Normalizes vector if possible
 *
 * @return false if norm is zero
 */
bool Vector3d::normalize() {
   double myNorm = norm();
   if (myNorm != 0) {
      scale(1.0/myNorm);
      return true;
   }
   return false;
}

/**
 * @brief (1-t)p1 + t*p2
 *
 * Interpolates between two vectors, this = (1-t)p1 + t*p2
 *
 * @param p1
 * @param t interpolation factor
 * @param p2
 */
void Vector3d::interpolate(const Vector3d& p1, double t, const Vector3d& p2) {
   x = (1 - t) * p1.x + t * p2.x;
   y = (1 - t) * p1.y + t * p2.y;
   z = (1 - t) * p1.z + t * p2.z;
}

std::string Vector3d::toString(std::string fmt) const {
   char buffer[80];
   snprintf(buffer, 80, fmt.c_str(), x, y, z);
   std::string out(buffer);
   return out;
}

// Vector2D
const Vector2d Vector2d::ZERO = Vector2d(0,0);
const Vector2d Vector2d::X_AXIS = Vector2d(1,0);
const Vector2d Vector2d::Y_AXIS = Vector2d(0,1);

Vector2d::Vector2d()
: x(0), y(0) {}

Vector2d::Vector2d(const Vector2d& copyMe)
: x(copyMe.x), y(copyMe.y){}

Vector2d::Vector2d(double x, double y)
: x(x), y(y) {}

Vector2d& Vector2d::operator=(const Vector2d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   return *this;
}

void Vector2d::set(const Vector2d& pnt) {
   x = pnt.x;
   y = pnt.y;
}

void Vector2d::set(double x, double y) {
   this->x = x;
   this->y = y;
}

void Vector2d::set(int idx, double val) {
   switch ( idx ) {
   case 0:
      x = val;
      return;
   case 1:
      y = val;
      return;
   }
}

void Vector2d::setZero() {
   x = 0;
   y = 0;
}

double Vector2d::get(int idx) const {
   switch ( idx ) {
   case 0:
      return x;
   case 1:
      return y;
   }
   return 0;
}

void Vector2d::add(const Vector2d& v) {
   x += v.x;
   y += v.y;
}

void Vector2d::add(double x, double y) {
   this->x += x;
   this->y += y;
}

void Vector2d::add(const Vector2d& v1, const Vector2d& v2) {
   x = v1.x + v2.x;
   y = v1.y + v2.y;
}

void Vector2d::subtract(const Vector2d& v) {
   x -= v.x;
   y -= v.y;
}

/**
 * @brief v1-v2
 *
 * Sets the contents of this vector to v1-v2
 *
 * @param v1
 * @param v2
 */
void Vector2d::subtract(const Vector2d& v1, const Vector2d& v2) {
   x = v1.x - v2.x;
   y = v1.y - v2.y;
}

// v1 + s*v2
void Vector2d::scaledAdd(const Vector2d& v1, double s,
      const Vector2d& v2) {
   x = v1.x + s*v2.x;
   y = v1.y + s*v2.y;
}

void Vector2d::scaledAdd(double s, const Vector2d& v) {
   scaledAdd(*this, s, v);
}

void Vector2d::scale(double s) {
   x *= s;
   y *= s;
}

void Vector2d::scale(double s, const Vector2d& v) {
   x = s*v.x;
   y = s*v.y;
}

void Vector2d::negate() {
   x = -x;
   y = -y;
}

double Vector2d::dot(double x, double y) const {
   return this->x*x + this->y*y;
}

double Vector2d::dot(const Vector2d& v) const {
   return x*v.x+y*v.y;
}

double Vector2d::norm() const {
   return sqrt(x*x+y*y);
}

double Vector2d::normSquared() const {
   return x*x+y*y;
}

// return false if norm is zero
bool Vector2d::normalize() {
   double myNorm = norm();
   if (myNorm != 0) {
      scale(1.0/myNorm);
      return true;
   }
   return false;
}

void Vector2d::interpolate(const Vector2d& p1, double t, const Vector2d& p2) {
   x = (1 - t) * p1.x + t * p2.x;
   y = (1 - t) * p1.y + t * p2.y;
}

std::string Vector2d::toString(std::string fmt) const {
   char buffer[80];
   snprintf(buffer, 80, fmt.c_str(), x, y);
   std::string out(buffer);
   return out;
}

// Point implementation
Point3d::Point3d()
: Vector3d(0,0,0) {}

Point3d::Point3d(const Point3d& copyMe)
: Vector3d(copyMe) {}

Point3d::Point3d(const Vector3d& pointMe)
: Vector3d(pointMe) {}

Point3d::Point3d(double x, double y, double z)
: Vector3d(x,y,z) {}

Point3d& Point3d::operator=(const Point3d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   z = assignMe.z;
   return *this;
}

double Point3d::distance(const Point3d& d) const {
   double dx = x-d.x;
   double dy = y-d.y;
   double dz = z-d.z;
   return sqrt(dx*dx+dy*dy+dz*dz);
}

double Point3d::distanceSquared(const Point3d& d) const {
   double dx = x-d.x;
   double dy = y-d.y;
   double dz = z-d.z;
   return dx*dx+dy*dy+dz*dz;
}

double Point3d::distance(double x, double y, double z) const {
   double dx = this->x-x;
   double dy = this->y-y;
   double dz = this->z-z;
   return sqrt(dx*dx+dy*dy+dz*dz);
}

double Point3d::distanceSquared(double x, double y, double z) const {
   double dx = this->x-x;
   double dy = this->y-y;
   double dz = this->z-z;
   return dx*dx+dy*dy+dz*dz;
}

// Matrix implementation

const Matrix3d Matrix3d::IDENTITY = Matrix3d(1,0,0,0,1,0,0,0,1);

Matrix3d::Matrix3d()
: m() {}

Matrix3d::Matrix3d(const Matrix3d& copyMe) {
   set(copyMe.m);
}

Matrix3d::Matrix3d(const double m[]) {
   set(m);
}

Matrix3d::Matrix3d( double m00, double m01, double m02,
      double m10, double m11, double m12,
      double m20, double m21, double m22 ) {
   set(m00,m01,m02,m10,m11,m12,m20,m21,m22);
}

Matrix3d& Matrix3d::operator=(const Matrix3d& assignMe) {
   set(assignMe.m);
   return *this;
}

double Matrix3d::get(int i, int j) const {
   return m[IDX3D(i,j)];
}

void Matrix3d::set(int i, int j, double val) {
   m[IDX3D(i,j)] = val;
}

void Matrix3d::set(const Matrix3d& mat) {
   set(mat.m);
}

void Matrix3d::set(const double mat[]) {
   std::copy(mat, mat+IDX3D_N, m);
}

void Matrix3d::set( double m00, double m01, double m02,
      double m10, double m11, double m12,
      double m20, double m21, double m22 ) {
   m[IDX3D_00] = m00;
   m[IDX3D_01] = m01;
   m[IDX3D_02] = m02;
   m[IDX3D_10] = m10;
   m[IDX3D_11] = m11;
   m[IDX3D_12] = m12;
   m[IDX3D_20] = m20;
   m[IDX3D_21] = m21;
   m[IDX3D_22] = m22;
}

void Matrix3d::setColumn(int col, const Vector3d& v) {
   m[IDX3D(0, col)] = v.x;
   m[IDX3D(1, col)] = v.y;
   m[IDX3D(2, col)] = v.z;
}

void Matrix3d::getColumn(int col, Vector3d& v) const {
   v.x = m[IDX3D(0, col)];
   v.y = m[IDX3D(1, col)];
   v.z = m[IDX3D(2, col)];
}

void Matrix3d::setRow(int row, const Vector3d& v) {
   m[IDX3D(row, 0)] = v.x;
   m[IDX3D(row, 1)] = v.y;
   m[IDX3D(row, 2)] = v.z;
}

void Matrix3d::getRow(int row, Vector3d& v) const {
   v.x = m[IDX3D(row, 0)];
   v.y = m[IDX3D(row, 1)];
   v.z = m[IDX3D(row, 2)];
}

void Matrix3d::add(const Matrix3d& mat) {
   for (int i=0; i<IDX3D_N; i++) {
      m[i] += mat.m[i];
   }
}

void Matrix3d::subtract(const Matrix3d& mat) {
   for (int i=0; i<IDX3D_N; i++) {
      m[i] -= mat.m[i];
   }
}

void Matrix3d::scaledAdd(double s, const Matrix3d& mat) {
   for (int i=0; i<IDX3D_N; i++) {
      m[i] += s*mat.m[i];
   }
}

void Matrix3d::scale(double s) {
   for (int i=0; i<IDX3D_N; i++) {
      m[i] = s*m[i];
   }
}

void Matrix3d::scaleColumn(int col, double s) {
   m[IDX3D(0, col)] = s*m[IDX3D(0, col)];
   m[IDX3D(1, col)] = s*m[IDX3D(1, col)];
   m[IDX3D(2, col)] = s*m[IDX3D(2, col)];
}

void Matrix3d::scaleRow(int row, double s) {
   m[IDX3D(row, 0)] = s*m[IDX3D(row, 0)];
   m[IDX3D(row, 1)] = s*m[IDX3D(row, 1)];
   m[IDX3D(row, 2)] = s*m[IDX3D(row, 2)];
}

void Matrix3d::multiply(const Matrix3d& right) {
   multiply(*this, right);
}

void Matrix3d::multiply(const Matrix3d& left, const Matrix3d& right) {
   // store for safety, so can multiply by one's self
   double m00 =  left.m[IDX3D_00]*right.m[IDX3D_00]
                                          + left.m[IDX3D_01]*right.m[IDX3D_10]
                                                                     + left.m[IDX3D_02]*right.m[IDX3D_20];
   double m10 =  left.m[IDX3D_10]*right.m[IDX3D_00]
                                          + left.m[IDX3D_11]*right.m[IDX3D_10]
                                                                     + left.m[IDX3D_12]*right.m[IDX3D_20];
   double m20 =  left.m[IDX3D_20]*right.m[IDX3D_00]
                                          + left.m[IDX3D_21]*right.m[IDX3D_10]
                                                                     + left.m[IDX3D_22]*right.m[IDX3D_20];

   double m01 =  left.m[IDX3D_00]*right.m[IDX3D_01]
                                          + left.m[IDX3D_01]*right.m[IDX3D_11]
                                                                     + left.m[IDX3D_02]*right.m[IDX3D_21];
   double m11 =  left.m[IDX3D_10]*right.m[IDX3D_01]
                                          + left.m[IDX3D_11]*right.m[IDX3D_11]
                                                                     + left.m[IDX3D_12]*right.m[IDX3D_21];
   double m21 =  left.m[IDX3D_20]*right.m[IDX3D_01]
                                          + left.m[IDX3D_21]*right.m[IDX3D_11]
                                                                     + left.m[IDX3D_22]*right.m[IDX3D_21];

   double m02 =  left.m[IDX3D_00]*right.m[IDX3D_02]
                                          + left.m[IDX3D_01]*right.m[IDX3D_12]
                                                                     + left.m[IDX3D_02]*right.m[IDX3D_22];
   double m12 =  left.m[IDX3D_10]*right.m[IDX3D_02]
                                          + left.m[IDX3D_11]*right.m[IDX3D_12]
                                                                     + left.m[IDX3D_12]*right.m[IDX3D_22];
   double m22 =  left.m[IDX3D_20]*right.m[IDX3D_02]
                                          + left.m[IDX3D_21]*right.m[IDX3D_12]
                                                                     + left.m[IDX3D_22]*right.m[IDX3D_22];

   m[IDX3D_00] = m00;
   m[IDX3D_10] = m10;
   m[IDX3D_20] = m20;

   m[IDX3D_01] = m01;
   m[IDX3D_11] = m11;
   m[IDX3D_21] = m21;

   m[IDX3D_02] = m02;
   m[IDX3D_12] = m12;
   m[IDX3D_22] = m22;
}

void Matrix3d::multiply(const Vector3d& pnt, Vector3d& out) const {

   double x = pnt.x;
   double y = pnt.y;
   double z = pnt.z;

   out.x = m[IDX3D_00]*x + m[IDX3D_01]*y + m[IDX3D_02]*z;
   out.y = m[IDX3D_10]*x + m[IDX3D_11]*y + m[IDX3D_12]*z;
   out.z = m[IDX3D_20]*x + m[IDX3D_21]*y + m[IDX3D_22]*z;
}

void Matrix3d::multiplyLeft(const Vector3d& pnt, Vector3d& out) const {
   double x = pnt.x;
   double y = pnt.y;
   double z = pnt.z;

   out.x = m[IDX3D_00]*x + m[IDX3D_10]*y + m[IDX3D_20]*z;
   out.y = m[IDX3D_01]*x + m[IDX3D_11]*y + m[IDX3D_21]*z;
   out.z = m[IDX3D_02]*x + m[IDX3D_12]*y + m[IDX3D_22]*z;
}

void Matrix3d::outerProduct(const Vector3d& v1, const Vector3d& v2) {
   m[IDX3D_00] = v1.x*v2.x;
   m[IDX3D_01] = v1.x*v2.y;
   m[IDX3D_02] = v1.x*v2.z;
   m[IDX3D_10] = v1.y*v2.x;
   m[IDX3D_11] = v1.y*v2.y;
   m[IDX3D_12] = v1.y*v2.z;
   m[IDX3D_20] = v1.z*v2.x;
   m[IDX3D_21] = v1.z*v2.y;
   m[IDX3D_22] = v1.z*v2.z;
}

void Matrix3d::addOuterProduct(const Vector3d& v1, const Vector3d& v2) {
   m[IDX3D_00] += v1.x*v2.x;
   m[IDX3D_01] += v1.x*v2.y;
   m[IDX3D_02] += v1.x*v2.z;
   m[IDX3D_10] += v1.y*v2.x;
   m[IDX3D_11] += v1.y*v2.y;
   m[IDX3D_12] += v1.y*v2.z;
   m[IDX3D_20] += v1.z*v2.x;
   m[IDX3D_21] += v1.z*v2.y;
   m[IDX3D_22] += v1.z*v2.z;
}

double Matrix3d::determinant() const {
   double det =  m[IDX3D_00]*m[IDX3D_11]*m[IDX3D_22]
                                           + m[IDX3D_10]*m[IDX3D_21]*m[IDX3D_02]
                                                                       + m[IDX3D_01]*m[IDX3D_12]*m[IDX3D_20]
                                                                                                   - m[IDX3D_11]*m[IDX3D_20]*m[IDX3D_02]
                                                                                                                               - m[IDX3D_00]*m[IDX3D_21]*m[IDX3D_12]
                                                                                                                                                           - m[IDX3D_10]*m[IDX3D_01]*m[IDX3D_22];
   return det;
}

double Matrix3d::condition() const {
   Matrix3d U;
   Matrix3d V;
   Vector3d s;
   math::svd3(*this, U, s, V);
   return s.x/s.z;
}

void Matrix3d::transpose() {
   double m01 = m[IDX3D_01];
   double m02 = m[IDX3D_02];
   double m12 = m[IDX3D_12];

   m[IDX3D_01] = m[IDX3D_10];
   m[IDX3D_02] = m[IDX3D_20];
   m[IDX3D_12] = m[IDX3D_21];

   m[IDX3D_10] = m01;
   m[IDX3D_20] = m02;
   m[IDX3D_21] = m12;
}

// returns determinant
double Matrix3d::pseudoInvert() {
   // pseudo-inverse
   Matrix3d U;
   Matrix3d V;
   Vector3d s;
   math::svd3(*this, U, s, V);
   double det = s.x*s.y*s.z;

   // threshold taken from Matlab recommendation
   if (s.x > 3*s.x*MAS_MATH_MACHINE_PRECISION) {
      s.x = 1.0/s.x;
   } else {
      s.x = 0;
      det = 0;
   }
   if (s.y > 3*s.x*MAS_MATH_MACHINE_PRECISION) {
      s.y = 1.0/s.y;
   } else {
      s.y = 0;
      det = 0;
   }
   if (s.z > 3*s.x*MAS_MATH_MACHINE_PRECISION) {
      s.z = 1.0/s.z;
   } else {
      s.z = 0;
      det = 0;
   }

   m[IDX3D_00] = U.m[IDX3D_00]*s.x;
   m[IDX3D_10] = U.m[IDX3D_10]*s.x;
   m[IDX3D_20] = U.m[IDX3D_20]*s.x;

   m[IDX3D_01] = U.m[IDX3D_01]*s.y;
   m[IDX3D_11] = U.m[IDX3D_11]*s.y;
   m[IDX3D_21] = U.m[IDX3D_21]*s.y;

   m[IDX3D_02] = U.m[IDX3D_02]*s.z;
   m[IDX3D_12] = U.m[IDX3D_12]*s.z;
   m[IDX3D_22] = U.m[IDX3D_22]*s.z;

   V.transpose();
   multiply(V);

   return det;

}

double Matrix3d::invert() {

   double det = determinant();

   if (det == 0) {
      return pseudoInvert();
   }

   // use determinant method
   double m00 = m[IDX3D_00];
   double m10 = m[IDX3D_10];
   double m20 = m[IDX3D_20];

   double m01 = m[IDX3D_01];
   double m11 = m[IDX3D_11];
   double m21 = m[IDX3D_21];

   double m02 = m[IDX3D_02];
   double m12 = m[IDX3D_12];
   double m22 = m[IDX3D_22];

   double idet = 1.0/det;

   m[IDX3D_00] =  idet*(m11*m22 - m12*m21);
   m[IDX3D_10] =  idet*(m12*m20 - m10*m22);
   m[IDX3D_20] =  idet*(m10*m21 - m11*m20);

   m[IDX3D_01] =  idet*(m02*m21 - m01*m22);
   m[IDX3D_11] =  idet*(m00*m22 - m02*m20);
   m[IDX3D_21] =  idet*(m01*m20 - m00*m21);

   m[IDX3D_02] =  idet*(m01*m12 - m11*m02);
   m[IDX3D_12] =  idet*(m02*m10 - m00*m12);
   m[IDX3D_22] =  idet*(m00*m11 - m01*m10);

   return det;
}

void Matrix3d::setIdentity() {
   m[IDX3D_00] =  1;
   m[IDX3D_10] =  0;
   m[IDX3D_20] =  0;

   m[IDX3D_01] =  0;
   m[IDX3D_11] =  1;
   m[IDX3D_21] =  0;

   m[IDX3D_02] =  0;
   m[IDX3D_12] =  0;
   m[IDX3D_22] =  1;
}

void Matrix3d::setZero() {
   m[IDX3D_00] =  0;
   m[IDX3D_10] =  0;
   m[IDX3D_20] =  0;

   m[IDX3D_01] =  0;
   m[IDX3D_11] =  0;
   m[IDX3D_21] =  0;

   m[IDX3D_02] =  0;
   m[IDX3D_12] =  0;
   m[IDX3D_22] =  0;
}

const RotationMatrix3d RotationMatrix3d::IDENTITY =
      RotationMatrix3d(1,0,0,0,1,0,0,0,1);

RotationMatrix3d::RotationMatrix3d() {
   setIdentity();
}

RotationMatrix3d::RotationMatrix3d(const RotationMatrix3d& copyMe) {
   set(copyMe.m);
}

RotationMatrix3d::RotationMatrix3d(const double m[]) {
   set(m);
}

RotationMatrix3d::RotationMatrix3d(double m00, double m01, double m02,
      double m10, double m11, double m12,
      double m20, double m21, double m22) {
   set(m00,m01,m02,m10,m11,m12,m20,m21,m22);
}

RotationMatrix3d& RotationMatrix3d::operator=(
      const RotationMatrix3d& assignMe) {
   set(assignMe.m);
   return *this;
}

double RotationMatrix3d::invert() {
   transpose();
   return 1;
}

double RotationMatrix3d::pseudoInvert() {
   transpose();
   return 1;
}

bool RotationMatrix3d::isValid() {
   return ( fabs(determinant()-1)<MAS_MATH_MACHINE_PRECISION );
}

void RotationMatrix3d::setZero() {
   setIdentity();
}

const RigidTransform3d RigidTransform3d::IDENTITY =
      RigidTransform3d(RotationMatrix3d::IDENTITY, Vector3d::ZERO);

RigidTransform3d::RigidTransform3d()
: R(RotationMatrix3d::IDENTITY), t(Vector3d::ZERO) {}

RigidTransform3d::RigidTransform3d(const RigidTransform3d& copyMe)
: R(copyMe.R), t(copyMe.t) {}

RigidTransform3d::RigidTransform3d(const RotationMatrix3d& R,
      const Vector3d& t)
:  R(R), t(t) {}

RigidTransform3d& RigidTransform3d::operator=(
      const RigidTransform3d& assignMe) {
   set(assignMe.R, assignMe.t);
   return *this;
}

void RigidTransform3d::set(const RotationMatrix3d& R, const Vector3d& t) {
   this->R.set(R);
   this->t.set(t);
}

void RigidTransform3d::setRotation(const RotationMatrix3d& R) {
   this->R.set(R);
}

void RigidTransform3d::setTranslation(const Vector3d& t) {
   this->t.set(t);
}

void RigidTransform3d::transform(Point3d& point) const {
   R.multiply(point, point);
   point.add(t);
}

void RigidTransform3d::transform(Vector3d& vec) const {
   R.multiply(vec, vec);
}

void RigidTransform3d::inverseTransform(Point3d& point) const {
   point.subtract(t);
   R.multiplyLeft(point, point);
}

void RigidTransform3d::inverseTransform(Vector3d& vec) const {
   R.multiplyLeft(vec, vec);
}

void RigidTransform3d::invert() {
   R.multiplyLeft(t, t);
   t.negate();
   R.transpose();
}

void RigidTransform3d::multiply(const RigidTransform3d& right) {
   multiply(*this, right);
}

void RigidTransform3d::multiplyLeft(const RigidTransform3d& left) {
   multiply(left, *this);
}

void RigidTransform3d::multiply(const RigidTransform3d& left,
      const RigidTransform3d& right) {

   // store for safety
   Vector3d lt = left.t;

   left.R.multiply(right.t, t);
   t.add(lt);
   R.multiply(left.R, right.R);

}

// Line implementation
Line::Line()   // default to x-axis
: dir(1,0,0), origin(0,0,0) {}

Line::Line(const Line& other)
: dir(other.dir), origin(other.origin) {}

Line::Line(const Point3d& origin, const Vector3d& dir)
: dir(dir), origin(origin) {
   this->dir.normalize();
}

Line::Line(const Point3d& p0, const Point3d& p1) {
   set(p0, p1);
}

Line& Line::operator=(const Line& assignMe) {
   dir = assignMe.dir;
   origin = assignMe.origin;
   return *this;
}

void Line::set (const Point3d& origin, const Vector3d& dir) {
   this->origin.set(origin);
   this->dir.set(dir);
   this->dir.normalize();
}

bool Line::set (const Point3d& p0, const Point3d& p1) {
   dir.subtract(p1, p0);
   origin.set(p0);
   bool success = dir.normalize();
   return success;
}

Point3d Line::getOrigin() const {
   return origin;
}

Vector3d Line::getDirection() const {
   return dir;
}

void Line::flip() {
   dir.negate();
}

double Line::distance(const Point3d& p) const {
   Point3d p1(p);
   p1.subtract (origin);
   p1.scaledAdd(origin, p1.dot(dir), dir);
   return p1.distance (p);
}

double Line::distance(double x, double y, double z) const {
   Point3d p1(x,y,z);
   p1.subtract (origin);
   p1.scaledAdd(origin, p1.dot(dir), dir);
   return p1.distance (x, y, z);
}

// Plane implementation
Plane::Plane() // default to x-y plane
: normal(Vector3d(0,0,1)), d(0) {}

Plane::Plane(const Plane& other)
: normal(other.normal), d(other.d) {}

Plane::Plane(const Vector3d& normal, double d)
: normal(normal), d(d) {
   this->normal.normalize();
}

Plane::Plane(const Vector3d& normal, const Point3d& pnt)
: normal(normal), d(-normal.dot(pnt.x, pnt.y, pnt.z)) {
   this->normal.normalize();
}

Plane::Plane(const Point3d& p0, const Point3d& p1, const Point3d& p2) {
   set(p0, p1, p2);
}

Plane& Plane::operator=(const Plane& assignMe) {
   normal = assignMe.normal;
   d = assignMe.d;
   return *this;
}

void Plane::set (const Vector3d& normal, double d) {
   this->normal.set(normal);
   this->d = d;
   this->normal.normalize();
}

bool Plane::set (const Point3d& p0, const Point3d& p1,
      const Point3d& p2) {
   Vector3d u1 = Vector3d(p1);
   u1.subtract(p0);

   Vector3d u2 = Vector3d(p2);
   u2.subtract(p0);

   normal.cross(u1, u2);
   bool success = normal.normalize();

   d = -normal.dot(p0);
   return success;
}

void Plane::flip() {
   normal.negate();
   d = -d;
}

double Plane::distanceSigned(const Point3d& p) const {
   return normal.dot(p)+d;
}

double Plane::distanceSigned(double x, double y, double z) const {
   return normal.x*x + normal.y*y + normal.z*z + d;
}

double Plane::distance(const Point3d& p) const {
   return fabs(distanceSigned(p));
}

double Plane::distance(double x, double y, double z) const {
   return fabs(distanceSigned(x,y,z));
}

}
