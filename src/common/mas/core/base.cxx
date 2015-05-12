#include "mas/core/base.h"
#include "mas/core/math.h"
#include <math.h>
#include <algorithm>

namespace mas {

/**
 * Prints to a buffer, resizing if necessary
 * @param cbuff buffer to print to
 * @param pos position in buffer to print
 * @param fmt number format
 * @param d value to print
 * @return next free position
 */
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

// Vector2D
const Vector2d Vector2d::ZERO = Vector2d(0, 0);
const Vector2d Vector2d::X_AXIS = Vector2d(1, 0);
const Vector2d Vector2d::Y_AXIS = Vector2d(0, 1);

Vector2d::Vector2d() :
      x(0), y(0) {
}

Vector2d::Vector2d(const Vector2d& copyMe) :
      x(copyMe.x), y(copyMe.y) {
}

Vector2d::Vector2d(double x, double y) :
      x(x), y(y) {
}

Vector2d& Vector2d::operator=(const Vector2d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   return *this;
}

size_t Vector2d::size() const {
   return 2;
}

void Vector2d::set(const Vector2d& v) {
   x = v.x;
   y = v.y;
}

void Vector2d::set(double x, double y) {
   this->x = x;
   this->y = y;
}

void Vector2d::setZero() {
   x = 0;
   y = 0;
}

const double& Vector2d::operator()(size_t idx) const {
   if (idx == 1) {
      return y;
   }
   return x;
}

double& Vector2d::operator()(size_t idx) {
   if (idx == 1) {
      return y;
   }
   return x;
}

const double& Vector2d::operator[](size_t idx) const {
   if (idx == 1) {
      return y;
   }
   return x;
}

double& Vector2d::operator[](size_t idx) {
   if (idx == 1) {
      return y;
   }
   return x;
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
void Vector2d::scaledAdd(const Vector2d& v1, double s, const Vector2d& v2) {
   x = v1.x + s * v2.x;
   y = v1.y + s * v2.y;
}

void Vector2d::scaledAdd(double s, const Vector2d& v) {
   scaledAdd(*this, s, v);
}

void Vector2d::scale(double s) {
   x *= s;
   y *= s;
}

void Vector2d::scale(double s, const Vector2d& v) {
   x = s * v.x;
   y = s * v.y;
}

void Vector2d::negate() {
   x = -x;
   y = -y;
}

double Vector2d::dot(double x, double y) const {
   return this->x * x + this->y * y;
}

double Vector2d::dot(const Vector2d& v) const {
   return x * v.x + y * v.y;
}

double Vector2d::norm() const {
   return sqrt(x * x + y * y);
}

double Vector2d::normSquared() const {
   return x * x + y * y;
}

// return false if norm is zero
bool Vector2d::normalize() {
   double myNorm = norm();
   if (myNorm != 0) {
      scale(1.0 / myNorm);
      return true;
   }
   return false;
}

void Vector2d::interpolate(const Vector2d& p1, double t, const Vector2d& p2) {
   x = (1 - t) * p1.x + t * p2.x;
   y = (1 - t) * p1.y + t * p2.y;
}

std::string Vector2d::toString(const std::string fmt) const {
   std::vector<char> cbuff(80);
   size_t pos = safeprintd(cbuff, 0, fmt.c_str(), x);
   safeprintd(cbuff, pos, fmt.c_str(), y);
   return std::string(cbuff.data());
}

std::string Vector2d::toString(const std::string fmt,
      const std::string backfmt) const {
   std::vector<char> cbuff(80);
   size_t pos = safeprintd(cbuff, 0, fmt.c_str(), x);
   safeprintd(cbuff, pos, backfmt.c_str(), y);
   return std::string(cbuff.data());
}

// Vector implementation
const Vector3d Vector3d::ZERO(0, 0, 0);
const Vector3d Vector3d::X_AXIS(1, 0, 0);
const Vector3d Vector3d::Y_AXIS(0, 1, 0);
const Vector3d Vector3d::Z_AXIS(0, 0, 1);

Vector3d::Vector3d() :
      x(0), y(0), z(0) {
}

Vector3d::Vector3d(const Vector3d& copyMe) :
      x(copyMe.x), y(copyMe.y), z(copyMe.z) {
}

Vector3d::Vector3d(double x, double y, double z) :
      x(x), y(y), z(z) {
}

Vector3d& Vector3d::operator=(const Vector3d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   z = assignMe.z;
   return *this;
}

size_t Vector3d::size() const {
   return 3;
}

void Vector3d::set(const Vector3d& v) {
   x = v.x;
   y = v.y;
   z = v.z;
}

void Vector3d::set(double x, double y, double z) {
   this->x = x;
   this->y = y;
   this->z = z;
}

void Vector3d::setZero() {
   x = 0;
   y = 0;
   z = 0;
}

const double& Vector3d::operator()(size_t idx) const {
   switch (idx) {
   case 1:
      return y;
   case 2:
      return z;
   default:
      return x;
   }
}

double& Vector3d::operator()(size_t idx) {
   switch (idx) {
   case 1:
      return y;
   case 2:
      return z;
   default:
      return x;
   }
}

const double& Vector3d::operator[](size_t idx) const {
   switch (idx) {
   case 1:
      return y;
   case 2:
      return z;
   default:
      return x;
   }
}

double& Vector3d::operator[](size_t idx) {
   switch (idx) {
   case 1:
      return y;
   case 2:
      return z;
   default:
      return x;
   }
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
void Vector3d::scaledAdd(const Vector3d& v1, double s, const Vector3d& v2) {
   x = v1.x + s * v2.x;
   y = v1.y + s * v2.y;
   z = v1.z + s * v2.z;
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
   x = s * v.x;
   y = s * v.y;
   z = s * v.z;
}

void Vector3d::negate() {
   x = -x;
   y = -y;
   z = -z;
}

double Vector3d::dot(double x, double y, double z) const {
   return (this->x * x + this->y * y + this->z * z);
}

double Vector3d::dot(const Vector3d& v) const {
   return (x * v.x + y * v.y + z * v.z);
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
   return sqrt(x * x + y * y + z * z);
}

double Vector3d::normSquared() const {
   return (x * x + y * y + z * z);
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
      scale(1.0 / myNorm);
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

std::string Vector3d::toString(const std::string fmt) const {
   std::vector<char> cbuff(80);
   size_t pos = safeprintd(cbuff, 0, fmt.c_str(), x);
   pos = safeprintd(cbuff, pos, fmt.c_str(), y);
   safeprintd(cbuff, pos, fmt.c_str(), z);
   return std::string(cbuff.data());
}

std::string Vector3d::toString(const std::string fmt,
      const std::string backfmt) const {
   std::vector<char> cbuff(80);
   size_t pos = safeprintd(cbuff, 0, fmt.c_str(), x);
   pos = safeprintd(cbuff, pos, fmt.c_str(), y);
   safeprintd(cbuff, pos, backfmt.c_str(), z);
   return std::string(cbuff.data());
}

// Vector4D
const Vector4d Vector4d::ZERO = Vector4d(0, 0, 0, 0);
const Vector4d Vector4d::W_AXIS = Vector4d(1, 0, 0, 0);
const Vector4d Vector4d::X_AXIS = Vector4d(0, 1, 0, 0);
const Vector4d Vector4d::Y_AXIS = Vector4d(0, 0, 1, 0);
const Vector4d Vector4d::Z_AXIS = Vector4d(0, 0, 0, 1);

Vector4d::Vector4d() :
      w(0), x(0), y(0), z(0) {
}

Vector4d::Vector4d(const Vector4d& copyMe) :
      w(copyMe.w), x(copyMe.x), y(copyMe.y), z(copyMe.z) {
}

Vector4d::Vector4d(double w, double x, double y, double z) :
      w(w), x(x), y(y), z(z) {
}

Vector4d& Vector4d::operator=(const Vector4d& assignMe) {
   w = assignMe.w;
   x = assignMe.x;
   y = assignMe.y;
   z = assignMe.z;
   return *this;
}

size_t Vector4d::size() const {
   return 4;
}

void Vector4d::set(const Vector4d& pnt) {
   w = pnt.w;
   x = pnt.x;
   y = pnt.y;
   z = pnt.z;
}

void Vector4d::set(double w, double x, double y, double z) {
   this->w = w;
   this->x = x;
   this->y = y;
   this->z = z;
}

void Vector4d::setZero() {
   w = 0;
   x = 0;
   y = 0;
   z = 0;
}

const double& Vector4d::operator()(size_t idx) const {
   switch (idx) {
   case 1:
      return x;
   case 2:
      return y;
   case 3:
      return z;
   default:
      return w;
   }
}

double& Vector4d::operator()(size_t idx) {
   switch (idx) {
   case 1:
      return x;
   case 2:
      return y;
   case 3:
      return z;
   default:
      return w;
   }
}

const double& Vector4d::operator[](size_t idx) const {
   switch (idx) {
   case 1:
      return x;
   case 2:
      return y;
   case 3:
      return z;
   default:
      return w;
   }
}

double& Vector4d::operator[](size_t idx) {
   switch (idx) {
   case 1:
      return x;
   case 2:
      return y;
   case 3:
      return z;
   default:
      return w;
   }
}

void Vector4d::add(const Vector4d& v) {
   w += v.w;
   x += v.x;
   y += v.y;
   z += v.z;
}

void Vector4d::add(double w, double x, double y, double z) {
   this->w += w;
   this->x += x;
   this->y += y;
   this->z += z;
}

void Vector4d::add(const Vector4d& v1, const Vector4d& v2) {
   w = v1.w + v2.w;
   x = v1.x + v2.x;
   y = v1.y + v2.y;
   z = v1.z + v2.z;
}

void Vector4d::subtract(const Vector4d& v) {
   w -= v.w;
   x -= v.x;
   y -= v.y;
   z -= v.z;
}

/**
 * @brief v1-v2
 *
 * Sets the contents of this vector to v1-v2
 *
 * @param v1
 * @param v2
 */
void Vector4d::subtract(const Vector4d& v1, const Vector4d& v2) {
   w = v1.w - v2.w;
   x = v1.x - v2.x;
   y = v1.y - v2.y;
   z = v1.z - v2.z;
}

// v1 + s*v2
void Vector4d::scaledAdd(const Vector4d& v1, double s, const Vector4d& v2) {

   w = v1.w + s * v2.w;
   x = v1.x + s * v2.x;
   y = v1.y + s * v2.y;
   z = v1.z + s * v2.z;

}

void Vector4d::scaledAdd(double s, const Vector4d& v) {
   scaledAdd(*this, s, v);
}

void Vector4d::scale(double s) {
   w *= s;
   x *= s;
   y *= s;
   z *= s;
}

void Vector4d::scale(double s, const Vector4d& v) {
   w = s * v.w;
   x = s * v.x;
   y = s * v.y;
   z = s * v.z;
}

void Vector4d::negate() {
   w = -w;
   x = -x;
   y = -y;
   z = -z;
}

double Vector4d::dot(double w, double x, double y, double z) const {
   return this->w * w + this->x * x + this->y * y + this->z * z;
}

double Vector4d::dot(const Vector4d& v) const {
   return w * v.w + x * v.x + y * v.y + z * v.z;
}

double Vector4d::norm() const {
   return sqrt(w * w + x * x + y * y + z * z);
}

double Vector4d::normSquared() const {
   return w * w + x * x + y * y + z * z;
}

// return false if norm is zero
bool Vector4d::normalize() {
   double myNorm = norm();
   if (myNorm != 0) {
      scale(1.0 / myNorm);
      return true;
   }
   return false;
}

void Vector4d::interpolate(const Vector4d& p1, double t, const Vector4d& p2) {
   w = (1 - t) * p1.w + t * p2.w;
   x = (1 - t) * p1.x + t * p2.x;
   y = (1 - t) * p1.y + t * p2.y;
   z = (1 - t) * p1.z + t * p2.z;
}

std::string Vector4d::toString(const std::string fmt) const {
   std::vector<char> cbuff(80);
   size_t pos = safeprintd(cbuff, 0, fmt.c_str(), w);
   pos = safeprintd(cbuff, pos, fmt.c_str(), x);
   pos = safeprintd(cbuff, pos, fmt.c_str(), y);
   safeprintd(cbuff, pos, fmt.c_str(), z);
   return std::string(cbuff.data());
}

std::string Vector4d::toString(const std::string fmt,
      const std::string backfmt) const {
   std::vector<char> cbuff(80);
   size_t pos = safeprintd(cbuff, 0, fmt.c_str(), w);
   pos = safeprintd(cbuff, pos, fmt.c_str(), x);
   pos = safeprintd(cbuff, pos, fmt.c_str(), y);
   safeprintd(cbuff, pos, backfmt.c_str(), z);
   return std::string(cbuff.data());
}

VectorNd::VectorNd() :
      v() {
}

VectorNd::VectorNd(size_t size) :
      v(size) {
}

VectorNd::VectorNd(const VectorNd& copyMe) :
      v(copyMe.v) {
}

VectorNd::VectorNd(const Vector2d& copyMe) :
      v(2) {
   v[0] = copyMe.x;
   v[1] = copyMe.y;
}

VectorNd::VectorNd(const Vector3d& copyMe) :
      v(3) {
   v[0] = copyMe.x;
   v[1] = copyMe.y;
   v[2] = copyMe.z;
}

VectorNd::VectorNd(const Vector4d& copyMe) :
      v(4) {
   v[0] = copyMe.w;
   v[1] = copyMe.x;
   v[2] = copyMe.y;
   v[3] = copyMe.z;
}

VectorNd::VectorNd(VectorNd&& copyMe) :
      v(std::move(copyMe.v)) {
}

VectorNd& VectorNd::operator=(const VectorNd& assignMe) {
   v = assignMe.v;
   return *this;
}

VectorNd& VectorNd::operator=(const Vector2d& assignMe) {
   if (v.size() != 2) {
      v.resize(2);
   }
   v[0] = assignMe.x;
   v[1] = assignMe.y;
   return *this;
}

VectorNd& VectorNd::operator=(const Vector3d& assignMe) {
   if (v.size() != 3) {
      v.resize(3);
   }
   v[0] = assignMe.x;
   v[1] = assignMe.y;
   v[2] = assignMe.z;
   return *this;
}

VectorNd& VectorNd::operator=(const Vector4d& assignMe) {
   if (v.size() != 4) {
      v.resize(4);
   }
   v[0] = assignMe.w;
   v[1] = assignMe.x;
   v[2] = assignMe.y;
   v[3] = assignMe.z;
   return *this;
}

VectorNd& VectorNd::operator=(VectorNd&& moveMe) {
   v = std::move(moveMe.v);
   return *this;
}

void VectorNd::resize(size_t n) {
   v.resize(n, 0);
}

size_t VectorNd::size() const {
   return v.size();
}

double* VectorNd::data() {
   return v.data();
}

void VectorNd::set(const VectorNd& v) {
   this->v = v.v;
}

void VectorNd::setZero() {
   for (size_t i = 0; i < v.size(); i++) {
      v[i] = 0;
   }
}

const double& VectorNd::operator()(size_t idx) const {
   return v[idx];
}

double& VectorNd::operator()(size_t idx) {
   return v[idx];
}

const double& VectorNd::operator[](size_t idx) const {
   return v[idx];
}

double& VectorNd::operator[](size_t idx) {
   return v[idx];
}

void VectorNd::add(const VectorNd& v) {
   for (size_t i = 0; i < this->v.size(); i++) {
      this->v[i] += v(i);
   }
}

void VectorNd::add(const VectorNd& v1, const VectorNd& v2) {
   for (size_t i = 0; i < v.size(); i++) {
      v[i] = v1(i) + v2(i);
   }
}

void VectorNd::subtract(const VectorNd& v) {
   for (size_t i = 0; i < this->v.size(); i++) {
      this->v[i] -= v(i);
   }
}

/**
 * @brief v1-v2
 *
 * Sets the contents of this vector to v1-v2
 *
 * @param v1
 * @param v2
 */
void VectorNd::subtract(const VectorNd& v1, const VectorNd& v2) {
   for (size_t i = 0; i < v.size(); i++) {
      v[i] = v1(i) - v2(i);
   }
}

// v1 + s*v2
void VectorNd::scaledAdd(const VectorNd& v1, double s, const VectorNd& v2) {
   for (size_t i = 0; i < v.size(); i++) {
      v[i] = v1(i) + s * v2(i);
   }
}

void VectorNd::scaledAdd(double s, const VectorNd& v) {
   scaledAdd(*this, s, v);
}

void VectorNd::scale(double s) {
   for (size_t i = 0; i < v.size(); i++) {
      v[i] = s * v[i];
   }
}

void VectorNd::scale(double s, const VectorNd& v) {
   for (size_t i = 0; i < this->v.size(); i++) {
      this->v[i] = s * v(i);
   }
}

void VectorNd::negate() {
   for (size_t i = 0; i < v.size(); i++) {
      v[i] = -v[i];
   }
}

double VectorNd::dot(const VectorNd& v) const {
   double d = 0;
   for (size_t i = 0; i < this->v.size(); i++) {
      d += this->v[i] * v(i);
   }
   return d;
}

double VectorNd::norm() const {
   return sqrt(normSquared());
}

double VectorNd::normSquared() const {
   double d = 0;
   for (size_t i = 0; i < v.size(); i++) {
      d += v[i] * v[i];
   }
   return d;
}

// return false if norm is zero
bool VectorNd::normalize() {
   double myNorm = norm();
   if (myNorm != 0) {
      scale(1.0 / myNorm);
      return true;
   }
   return false;
}

void VectorNd::interpolate(const VectorNd& p1, double t, const VectorNd& p2) {
   for (size_t i = 0; i < v.size(); i++) {
      v[i] = (1 - t) * p1(i) + t * p2(i);
   }
}

std::string VectorNd::toString(const std::string fmt) const {
   std::vector<char> cbuff(80);
   size_t pos = 0;
   for (size_t i = 0; i < size(); i++) {
      pos = safeprintd(cbuff, pos, fmt.c_str(), v[i]);
   }
   return std::string(cbuff.data());
}

std::string VectorNd::toString(const std::string fmt,
      const std::string backfmt) const {
   std::vector<char> cbuff(80);
   size_t pos = 0;
   for (size_t i = 0; i < size() - 1; i++) {
      pos = safeprintd(cbuff, pos, fmt.c_str(), v[i]);
   }
   if (size() > 0) {
      safeprintd(cbuff, pos, backfmt.c_str(), v.back());
   }
   return std::string(cbuff.data());
}

// Point implementation
Point3d::Point3d() :
      Vector3d(0, 0, 0) {
}

Point3d::Point3d(const Point3d& copyMe) :
      Vector3d(copyMe) {
}

Point3d::Point3d(const Vector3d& pointMe) :
      Vector3d(pointMe) {
}

Point3d::Point3d(double x, double y, double z) :
      Vector3d(x, y, z) {
}

Point3d& Point3d::operator=(const Point3d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   z = assignMe.z;
   return *this;
}

double Point3d::distance(const Point3d& d) const {
   double dx = x - d.x;
   double dy = y - d.y;
   double dz = z - d.z;
   return sqrt(dx * dx + dy * dy + dz * dz);
}

double Point3d::distanceSquared(const Point3d& d) const {
   double dx = x - d.x;
   double dy = y - d.y;
   double dz = z - d.z;
   return dx * dx + dy * dy + dz * dz;
}

double Point3d::distance(double x, double y, double z) const {
   double dx = this->x - x;
   double dy = this->y - y;
   double dz = this->z - z;
   return sqrt(dx * dx + dy * dy + dz * dz);
}

double Point3d::distanceSquared(double x, double y, double z) const {
   double dx = this->x - x;
   double dy = this->y - y;
   double dz = this->z - z;
   return dx * dx + dy * dy + dz * dz;
}

// Indexed point
// Point implementation
IndexedPoint3d::IndexedPoint3d() :
      Point3d(0, 0, 0), idx(-1) {
}

IndexedPoint3d::IndexedPoint3d(const IndexedPoint3d& copyMe) :
      Point3d(copyMe), idx(copyMe.idx) {
}

IndexedPoint3d::IndexedPoint3d(const Vector3d& pointMe, size_t index) :
      Point3d(pointMe), idx(index) {
}

IndexedPoint3d::IndexedPoint3d(double x, double y, double z, size_t index) :
      Point3d(x, y, z), idx(index) {
}

IndexedPoint3d& IndexedPoint3d::operator=(const IndexedPoint3d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   z = assignMe.z;
   idx = assignMe.idx;
   return *this;
}

void IndexedPoint3d::setIndex(size_t index) {
   idx = index;
}

size_t IndexedPoint3d::getIndex() const {
   return idx;
}

void IndexedPoint3d::set(const Vector3d& vec) {
	this->x = vec.x;
	this->y = vec.y;
	this->z = vec.z;
}

void IndexedPoint3d::set(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

void IndexedPoint3d::set(double x, double y, double z, size_t index) {
	set(x,y,z);
	setIndex(index);
}

// Matrix implementation

const Matrix2d Matrix2d::IDENTITY = Matrix2d(1, 0, 0, 1);

Matrix2d::Matrix2d() :
      m() {
}

Matrix2d::Matrix2d(const Matrix2d& copyMe) :
      m(copyMe.m) {
}

Matrix2d::Matrix2d(Matrix2d&& moveMe) :
      m(std::move(moveMe.m)) {
}

Matrix2d::Matrix2d(const double m[]) :
      m() {
   set(m);
}

Matrix2d::Matrix2d(double m00, double m01, double m10, double m11) :
      m() {
   set(m00, m01, m10, m11);
}

Matrix2d& Matrix2d::operator=(const Matrix2d& assignMe) {
   m = assignMe.m;
   return *this;
}

Matrix2d& Matrix2d::operator=(Matrix2d&& assignMe) {
   m = std::move(assignMe.m);
   return *this;
}

size_t Matrix2d::rows() const {
   return 2;
}

size_t Matrix2d::cols() const {
   return 2;
}

size_t Matrix2d::size() const {
   return IDX2D_N;
}

double& Matrix2d::operator()(size_t i, size_t j) {
   return m[IDX2D(i, j)];
}

const double& Matrix2d::operator()(size_t i, size_t j) const {
   return m[IDX2D(i, j)];
}

double& Matrix2d::operator[](size_t i) {
   return m[i];
}

const double& Matrix2d::operator[](size_t i) const {
   return m[i];
}

void Matrix2d::set(const Matrix2d& mat) {
   m = mat.m;
}

void Matrix2d::set(const double mat[]) {
   std::copy(mat, mat + IDX2D_N, m.begin());
}

void Matrix2d::set(double m00, double m01, double m10, double m11) {
   m[IDX2D_00] = m00;
   m[IDX2D_01] = m01;
   m[IDX2D_10] = m10;
   m[IDX2D_11] = m11;
}

void Matrix2d::setColumn(int col, const Vector2d& v) {
   m[IDX2D(0, col)] = v.x;
   m[IDX2D(1, col)] = v.y;
}

void Matrix2d::getColumn(int col, Vector2d& v) const {
   v.x = m[IDX2D(0, col)];
   v.y = m[IDX2D(1, col)];
}

void Matrix2d::setRow(int row, const Vector2d& v) {
   m[IDX2D(row, 0)] = v.x;
   m[IDX2D(row, 1)] = v.y;
}

void Matrix2d::getRow(int row, Vector2d& v) const {
   v.x = m[IDX2D(row, 0)];
   v.y = m[IDX2D(row, 1)];
}

void Matrix2d::add(const Matrix2d& mat) {
   for (int i = 0; i < IDX2D_N; i++) {
      m[i] += mat.m[i];
   }
}

void Matrix2d::add(const Matrix2d& mat1, const Matrix2d& mat2) {
   for (int i = 0; i < IDX2D_N; i++) {
      m[i] = mat1.m[i]+mat2.m[i];
   }
}

void Matrix2d::subtract(const Matrix2d& mat) {
   for (int i = 0; i < IDX2D_N; i++) {
      m[i] -= mat.m[i];
   }
}

void Matrix2d::subtract(const Matrix2d& mat1, const Matrix2d& mat2) {
   for (int i = 0; i < IDX2D_N; i++) {
      m[i] = mat1.m[i]-mat2.m[i];
   }
}


void Matrix2d::scaledAdd(double s, const Matrix2d& mat) {
   for (int i = 0; i < IDX2D_N; i++) {
      m[i] += s * mat.m[i];
   }
}

void Matrix2d::scale(double s) {
   for (int i = 0; i < IDX2D_N; i++) {
      m[i] = s * m[i];
   }
}

void Matrix2d::scaleColumn(int col, double s) {
   m[IDX2D(0, col)] = s * m[IDX2D(0, col)];
   m[IDX2D(1, col)] = s * m[IDX2D(1, col)];
}

void Matrix2d::scaleRow(int row, double s) {
   m[IDX2D(row, 0)] = s * m[IDX2D(row, 0)];
   m[IDX2D(row, 1)] = s * m[IDX2D(row, 1)];
}

void Matrix2d::multiply(const Matrix2d& right) {
   multiply(*this, right);
}

void Matrix2d::multiply(const Matrix2d& left, const Matrix2d& right) {
   // store for safety, so can multiply by one's self
   double m00 = left.m[IDX2D_00] * right.m[IDX2D_00]
         + left.m[IDX2D_01] * right.m[IDX2D_10];
   double m10 = left.m[IDX2D_10] * right.m[IDX2D_00]
         + left.m[IDX2D_11] * right.m[IDX2D_10];

   double m01 = left.m[IDX2D_00] * right.m[IDX2D_01]
         + left.m[IDX2D_01] * right.m[IDX2D_11];
   double m11 = left.m[IDX2D_10] * right.m[IDX2D_01]
         + left.m[IDX2D_11] * right.m[IDX2D_11];

   m[IDX2D_00] = m00;
   m[IDX2D_10] = m10;

   m[IDX2D_01] = m01;
   m[IDX2D_11] = m11;
}

void Matrix2d::multiply(const Vector2d& pnt, Vector2d& out) const {

   double x = pnt.x;
   double y = pnt.y;

   out.x = m[IDX2D_00] * x + m[IDX2D_01] * y;
   out.y = m[IDX2D_10] * x + m[IDX2D_11] * y;

}

void Matrix2d::multiplyLeft(const Vector2d& pnt, Vector2d& out) const {
   double x = pnt.x;
   double y = pnt.y;

   out.x = m[IDX2D_00] * x + m[IDX2D_10] * y;
   out.y = m[IDX2D_01] * x + m[IDX2D_11] * y;
}

void Matrix2d::outerProduct(const Vector2d& v1, const Vector2d& v2) {
   m[IDX2D_00] = v1.x * v2.x;
   m[IDX2D_01] = v1.x * v2.y;
   m[IDX2D_10] = v1.y * v2.x;
   m[IDX2D_11] = v1.y * v2.y;
}

void Matrix2d::addOuterProduct(const Vector2d& v1, const Vector2d& v2) {
   m[IDX2D_00] += v1.x * v2.x;
   m[IDX2D_01] += v1.x * v2.y;
   m[IDX2D_10] += v1.y * v2.x;
   m[IDX2D_11] += v1.y * v2.y;
}

void Matrix2d::addScaledOuterProduct(double s, const Vector2d& v1, const Vector2d& v2) {
   m[IDX2D_00] += s*v1.x * v2.x;
   m[IDX2D_01] += s*v1.x * v2.y;
   m[IDX2D_10] += s*v1.y * v2.x;
   m[IDX2D_11] += s*v1.y * v2.y;
}

double Matrix2d::determinant() const {
   double det = m[IDX2D_00] * m[IDX2D_11] - m[IDX2D_01] * m[IDX2D_10];
   return det;
}

double Matrix2d::condition() const {

   double s = m[0] * m[0] + m[1] * m[1] + m[2] * m[2] + m[3] * m[3];
   double d = determinant();

   d = sqrt(s * s / 4 - d * d);
   double s1 = sqrt(s / 2 + d);
   double s2 = sqrt(s / 2 - d);

   if (s2 > s1) {
      return s2 / s1;
   }
   return s1 / s2;

}

void Matrix2d::transpose() {
   std::swap(m[IDX2D_01], m[IDX2D_10]);
}

// returns determinant
double Matrix2d::pseudoInvert() {

   double det = m[IDX2D_00] * m[IDX2D_11] - m[IDX2D_01] * m[IDX2D_10];
   double alpha = (m[IDX2D_00] * m[IDX2D_01] + m[IDX2D_10] * m[IDX2D_11])
         / (m[IDX2D_00] * m[IDX2D_00] + m[IDX2D_01] * m[IDX2D_01]);
   double c = (1 + alpha * alpha)
         * (m[IDX2D_00] * m[IDX2D_00] + m[IDX2D_10] * m[IDX2D_10]);

   transpose();
   scale(1.0 / c);

   return det;

}

double Matrix2d::invert() {

   double det = determinant();

   if (det == 0) {
      return pseudoInvert();
   }

   // use determinant method
   scale(1.0 / det);
   std::swap(m[IDX2D_00], m[IDX2D_00]);
   m[IDX2D_01] = -m[IDX2D_01];
   m[IDX2D_10] = -m[IDX2D_10];

   return det;
}

void Matrix2d::setIdentity() {
   m[IDX2D_00] = 1;
   m[IDX2D_10] = 0;

   m[IDX2D_01] = 0;
   m[IDX2D_11] = 1;
}

void Matrix2d::setZero() {
   m[IDX2D_00] = 0;
   m[IDX2D_10] = 0;

   m[IDX2D_01] = 0;
   m[IDX2D_11] = 0;
}

const Matrix3d Matrix3d::IDENTITY = Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1);

Matrix3d::Matrix3d() :
      m() {
}

Matrix3d::Matrix3d(const Matrix3d& copyMe) :
      m(copyMe.m) {
}

Matrix3d::Matrix3d(Matrix3d&& copyMe) :
      m(std::move(copyMe.m)) {
}

Matrix3d::Matrix3d(const double m[]) :
      m() {
   set(m);
}

Matrix3d::Matrix3d(double m00, double m01, double m02, double m10, double m11,
      double m12, double m20, double m21, double m22) :
      m() {
   set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
}

Matrix3d& Matrix3d::operator=(const Matrix3d& assignMe) {
   m = assignMe.m;
   return *this;
}

Matrix3d& Matrix3d::operator=(Matrix3d&& moveMe) {
   m = std::move(moveMe.m);
   return *this;
}

size_t Matrix3d::rows() const {
   return 3;
}

size_t Matrix3d::cols() const {
   return 3;
}

size_t Matrix3d::size() const {
   return IDX3D_N;
}

double& Matrix3d::operator()(size_t i, size_t j) {
   return m[IDX3D(i, j)];
}

const double& Matrix3d::operator()(size_t i, size_t j) const {
   return m[IDX3D(i, j)];
}

double& Matrix3d::operator[](size_t i) {
   return m[i];
}

const double& Matrix3d::operator[](size_t i) const {
   return m[i];
}

void Matrix3d::set(const Matrix3d& mat) {
   m = mat.m;
}

void Matrix3d::set(const double mat[]) {
   std::copy(mat, mat + IDX3D_N, m.begin());
}

void Matrix3d::set(double m00, double m01, double m02, double m10, double m11,
      double m12, double m20, double m21, double m22) {
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
   for (int i = 0; i < IDX3D_N; i++) {
      m[i] += mat.m[i];
   }
}

void Matrix3d::add(const Matrix3d& mat1, const Matrix3d& mat2) {
   for (int i = 0; i < IDX3D_N; i++) {
      m[i] = mat1.m[i]+mat2.m[i];
   }
}

void Matrix3d::subtract(const Matrix3d& mat) {
   for (int i = 0; i < IDX3D_N; i++) {
      m[i] -= mat.m[i];
   }
}

void Matrix3d::subtract(const Matrix3d& mat1, const Matrix3d& mat2) {
   for (int i = 0; i < IDX3D_N; i++) {
      m[i] = mat1.m[i]-mat2.m[i];
   }
}

void Matrix3d::scaledAdd(double s, const Matrix3d& mat) {
   for (int i = 0; i < IDX3D_N; i++) {
      m[i] += s * mat.m[i];
   }
}

void Matrix3d::scale(double s) {
   for (int i = 0; i < IDX3D_N; i++) {
      m[i] = s * m[i];
   }
}

void Matrix3d::scaleColumn(int col, double s) {
   m[IDX3D(0, col)] = s * m[IDX3D(0, col)];
   m[IDX3D(1, col)] = s * m[IDX3D(1, col)];
   m[IDX3D(2, col)] = s * m[IDX3D(2, col)];
}

void Matrix3d::scaleRow(int row, double s) {
   m[IDX3D(row, 0)] = s * m[IDX3D(row, 0)];
   m[IDX3D(row, 1)] = s * m[IDX3D(row, 1)];
   m[IDX3D(row, 2)] = s * m[IDX3D(row, 2)];
}

void Matrix3d::multiply(const Matrix3d& right) {
   multiply(*this, right);
}

void Matrix3d::multiply(const Matrix3d& left, const Matrix3d& right) {
   // store for safety, so can multiply by one's self
   double m00 = left.m[IDX3D_00] * right.m[IDX3D_00]
         + left.m[IDX3D_01] * right.m[IDX3D_10]
         + left.m[IDX3D_02] * right.m[IDX3D_20];
   double m10 = left.m[IDX3D_10] * right.m[IDX3D_00]
         + left.m[IDX3D_11] * right.m[IDX3D_10]
         + left.m[IDX3D_12] * right.m[IDX3D_20];
   double m20 = left.m[IDX3D_20] * right.m[IDX3D_00]
         + left.m[IDX3D_21] * right.m[IDX3D_10]
         + left.m[IDX3D_22] * right.m[IDX3D_20];

   double m01 = left.m[IDX3D_00] * right.m[IDX3D_01]
         + left.m[IDX3D_01] * right.m[IDX3D_11]
         + left.m[IDX3D_02] * right.m[IDX3D_21];
   double m11 = left.m[IDX3D_10] * right.m[IDX3D_01]
         + left.m[IDX3D_11] * right.m[IDX3D_11]
         + left.m[IDX3D_12] * right.m[IDX3D_21];
   double m21 = left.m[IDX3D_20] * right.m[IDX3D_01]
         + left.m[IDX3D_21] * right.m[IDX3D_11]
         + left.m[IDX3D_22] * right.m[IDX3D_21];

   double m02 = left.m[IDX3D_00] * right.m[IDX3D_02]
         + left.m[IDX3D_01] * right.m[IDX3D_12]
         + left.m[IDX3D_02] * right.m[IDX3D_22];
   double m12 = left.m[IDX3D_10] * right.m[IDX3D_02]
         + left.m[IDX3D_11] * right.m[IDX3D_12]
         + left.m[IDX3D_12] * right.m[IDX3D_22];
   double m22 = left.m[IDX3D_20] * right.m[IDX3D_02]
         + left.m[IDX3D_21] * right.m[IDX3D_12]
         + left.m[IDX3D_22] * right.m[IDX3D_22];

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

   out.x = m[IDX3D_00] * x + m[IDX3D_01] * y + m[IDX3D_02] * z;
   out.y = m[IDX3D_10] * x + m[IDX3D_11] * y + m[IDX3D_12] * z;
   out.z = m[IDX3D_20] * x + m[IDX3D_21] * y + m[IDX3D_22] * z;
}

void Matrix3d::multiplyLeft(const Vector3d& pnt, Vector3d& out) const {
   double x = pnt.x;
   double y = pnt.y;
   double z = pnt.z;

   out.x = m[IDX3D_00] * x + m[IDX3D_10] * y + m[IDX3D_20] * z;
   out.y = m[IDX3D_01] * x + m[IDX3D_11] * y + m[IDX3D_21] * z;
   out.z = m[IDX3D_02] * x + m[IDX3D_12] * y + m[IDX3D_22] * z;
}

void Matrix3d::subtractMultiplyLeft(const Vector3d& p0, const Vector3d& p1,
      Vector3d& out) const {
   double x = p0.x - p1.x;
   double y = p0.y - p1.y;
   double z = p0.z - p1.z;

   out.x = m[IDX3D_00] * x + m[IDX3D_10] * y + m[IDX3D_20] * z;
   out.y = m[IDX3D_01] * x + m[IDX3D_11] * y + m[IDX3D_21] * z;
   out.z = m[IDX3D_02] * x + m[IDX3D_12] * y + m[IDX3D_22] * z;
}

void Matrix3d::multiplyAdd(const Vector3d& p0, const Vector3d& p1,
      Vector3d& out) const {
   double x = p0.x;
   double y = p0.y;
   double z = p0.z;

   out.x = m[IDX3D_00] * x + m[IDX3D_01] * y + m[IDX3D_02] * z + p1.x;
   out.y = m[IDX3D_10] * x + m[IDX3D_11] * y + m[IDX3D_12] * z + p1.y;
   out.z = m[IDX3D_20] * x + m[IDX3D_21] * y + m[IDX3D_22] * z + p1.z;
}

void Matrix3d::outerProduct(const Vector3d& v1, const Vector3d& v2) {
   m[IDX3D_00] = v1.x * v2.x;
   m[IDX3D_01] = v1.x * v2.y;
   m[IDX3D_02] = v1.x * v2.z;
   m[IDX3D_10] = v1.y * v2.x;
   m[IDX3D_11] = v1.y * v2.y;
   m[IDX3D_12] = v1.y * v2.z;
   m[IDX3D_20] = v1.z * v2.x;
   m[IDX3D_21] = v1.z * v2.y;
   m[IDX3D_22] = v1.z * v2.z;
}

void Matrix3d::addOuterProduct(const Vector3d& v1, const Vector3d& v2) {
   m[IDX3D_00] += v1.x * v2.x;
   m[IDX3D_01] += v1.x * v2.y;
   m[IDX3D_02] += v1.x * v2.z;
   m[IDX3D_10] += v1.y * v2.x;
   m[IDX3D_11] += v1.y * v2.y;
   m[IDX3D_12] += v1.y * v2.z;
   m[IDX3D_20] += v1.z * v2.x;
   m[IDX3D_21] += v1.z * v2.y;
   m[IDX3D_22] += v1.z * v2.z;
}

void Matrix3d::addScaledOuterProduct(double s, const Vector3d& v1, const Vector3d& v2) {
   m[IDX3D_00] += s*v1.x * v2.x;
   m[IDX3D_01] += s*v1.x * v2.y;
   m[IDX3D_02] += s*v1.x * v2.z;
   m[IDX3D_10] += s*v1.y * v2.x;
   m[IDX3D_11] += s*v1.y * v2.y;
   m[IDX3D_12] += s*v1.y * v2.z;
   m[IDX3D_20] += s*v1.z * v2.x;
   m[IDX3D_21] += s*v1.z * v2.y;
   m[IDX3D_22] += s*v1.z * v2.z;
}

double Matrix3d::determinant() const {
   double det = m[IDX3D_00] * m[IDX3D_11] * m[IDX3D_22]
         + m[IDX3D_10] * m[IDX3D_21] * m[IDX3D_02]
         + m[IDX3D_01] * m[IDX3D_12] * m[IDX3D_20]
         - m[IDX3D_11] * m[IDX3D_20] * m[IDX3D_02]
         - m[IDX3D_00] * m[IDX3D_21] * m[IDX3D_12]
         - m[IDX3D_10] * m[IDX3D_01] * m[IDX3D_22];
   return det;
}

double Matrix3d::condition() const {
   Matrix3d U;
   Matrix3d V;
   Vector3d s;
   math::svd3(*this, U, s, V);
   return s.x / s.z;
}

void Matrix3d::transpose() {
   std::swap(m[IDX3D_01], m[IDX3D_10]);
   std::swap(m[IDX3D_02], m[IDX3D_20]);
   std::swap(m[IDX3D_12], m[IDX3D_21]);
}

// returns determinant
double Matrix3d::pseudoInvert() {
   // pseudo-inverse
   Matrix3d U;
   Matrix3d V;
   Vector3d s;
   math::svd3(*this, U, s, V);
   double det = s.x * s.y * s.z;

   // threshold taken from Matlab recommendation
   if (s.x > 3 * s.x * MAS_MATH_MACHINE_PRECISION) {
      s.x = 1.0 / s.x;
   } else {
      s.x = 0;
      det = 0;
   }
   if (s.y > 3 * s.x * MAS_MATH_MACHINE_PRECISION) {
      s.y = 1.0 / s.y;
   } else {
      s.y = 0;
      det = 0;
   }
   if (s.z > 3 * s.x * MAS_MATH_MACHINE_PRECISION) {
      s.z = 1.0 / s.z;
   } else {
      s.z = 0;
      det = 0;
   }

   m[IDX3D_00] = U.m[IDX3D_00] * s.x;
   m[IDX3D_10] = U.m[IDX3D_10] * s.x;
   m[IDX3D_20] = U.m[IDX3D_20] * s.x;

   m[IDX3D_01] = U.m[IDX3D_01] * s.y;
   m[IDX3D_11] = U.m[IDX3D_11] * s.y;
   m[IDX3D_21] = U.m[IDX3D_21] * s.y;

   m[IDX3D_02] = U.m[IDX3D_02] * s.z;
   m[IDX3D_12] = U.m[IDX3D_12] * s.z;
   m[IDX3D_22] = U.m[IDX3D_22] * s.z;

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

   double idet = 1.0 / det;

   m[IDX3D_00] = idet * (m11 * m22 - m12 * m21);
   m[IDX3D_10] = idet * (m12 * m20 - m10 * m22);
   m[IDX3D_20] = idet * (m10 * m21 - m11 * m20);

   m[IDX3D_01] = idet * (m02 * m21 - m01 * m22);
   m[IDX3D_11] = idet * (m00 * m22 - m02 * m20);
   m[IDX3D_21] = idet * (m01 * m20 - m00 * m21);

   m[IDX3D_02] = idet * (m01 * m12 - m11 * m02);
   m[IDX3D_12] = idet * (m02 * m10 - m00 * m12);
   m[IDX3D_22] = idet * (m00 * m11 - m01 * m10);

   return det;
}

void Matrix3d::setIdentity() {
   m[IDX3D_00] = 1;
   m[IDX3D_10] = 0;
   m[IDX3D_20] = 0;

   m[IDX3D_01] = 0;
   m[IDX3D_11] = 1;
   m[IDX3D_21] = 0;

   m[IDX3D_02] = 0;
   m[IDX3D_12] = 0;
   m[IDX3D_22] = 1;
}

void Matrix3d::setZero() {
   m[IDX3D_00] = 0;
   m[IDX3D_10] = 0;
   m[IDX3D_20] = 0;

   m[IDX3D_01] = 0;
   m[IDX3D_11] = 0;
   m[IDX3D_21] = 0;

   m[IDX3D_02] = 0;
   m[IDX3D_12] = 0;
   m[IDX3D_22] = 0;
}

// Matrix implementation

MatrixNd::MatrixNd() :
      nr(0), nc(0), m() {
}

MatrixNd::MatrixNd(size_t rows, size_t cols) :
      nr(rows), nc(cols), m(rows * cols) {
}

MatrixNd::MatrixNd(const MatrixNd& copyMe) :
      nr(copyMe.nr), nc(copyMe.nc), m(copyMe.m) {
}

MatrixNd::MatrixNd(const Matrix2d& copyMe) :
      nr(2), nc(2), m(4) {
   std::copy(copyMe.m.begin(), copyMe.m.end(), m.begin());
}

MatrixNd::MatrixNd(const Matrix3d& copyMe) :
      nr(3), nc(3), m(9) {
   std::copy(copyMe.m.begin(), copyMe.m.end(), m.begin());
}

MatrixNd::MatrixNd(const Matrix4d& copyMe) :
      nr(4), nc(4), m(16) {
   std::copy(copyMe.m.begin(), copyMe.m.end(), m.begin());
}

MatrixNd::MatrixNd(size_t rows, size_t cols, const double* vals) :
      nr(rows), nc(cols), m(rows * cols) {
   std::copy(vals, vals + rows * cols, m.begin());
}

MatrixNd::MatrixNd(MatrixNd&& moveMe) :
      nr(moveMe.nr), nc(moveMe.nc), m(std::move(moveMe.m)) {
   moveMe.nr = 0;
   moveMe.nc = 0;
}

MatrixNd& MatrixNd::operator=(const MatrixNd& assignMe) {
   nr = assignMe.nr;
   nc = assignMe.nc;
   m = assignMe.m;
   return *this;
}

MatrixNd& MatrixNd::operator=(const Matrix2d& assignMe) {
   nr = 2;
   nc = 2;
   m.resize(4);
   std::copy(assignMe.m.begin(), assignMe.m.end(), m.begin());
   return *this;
}

MatrixNd& MatrixNd::operator=(const Matrix3d& assignMe) {
   nr = 3;
   nc = 3;
   m.resize(9);
   std::copy(assignMe.m.begin(), assignMe.m.end(), m.begin());
   return *this;
}

MatrixNd& MatrixNd::operator=(const Matrix4d& assignMe) {
   nr = 4;
   nc = 4;
   m.resize(16);
   std::copy(assignMe.m.begin(), assignMe.m.end(), m.begin());
   return *this;
}

MatrixNd& MatrixNd::operator=(MatrixNd&& moveMe) {
   m = std::move(moveMe.m);
   nr = moveMe.nr;
   nc = moveMe.nc;
   moveMe.nr = 0;
   moveMe.nc = 0;
   return *this;
}

double& MatrixNd::operator()(size_t i, size_t j) {
   return m[IDXND(i, j, nr, nc)];
}

const double& MatrixNd::operator()(size_t i, size_t j) const {
   return m[IDXND(i, j, nr, nc)];
}

double& MatrixNd::operator[](size_t i) {
   return m[i];
}

const double& MatrixNd::operator[](size_t i) const {
   return m[i];
}

size_t MatrixNd::rows() const {
   return nr;
}

size_t MatrixNd::cols() const {
   return nc;
}

size_t MatrixNd::size() const {
   return nr * nc;
}

void MatrixNd::resize(size_t rows, size_t cols) {
   m.resize(rows * cols);
}

void MatrixNd::set(const MatrixNd& mat) {
   m = mat.m;
   nr = mat.nr;
   nc = mat.nc;
}

void MatrixNd::set(size_t rows, size_t cols, const double* vals) {
   nr = rows;
   nc = cols;
   m.resize(rows * cols);
   std::copy(vals, vals + rows * cols, m.begin());
}

void MatrixNd::setColumn(int col, const VectorNd& v) {
   for (size_t i = 0; i < nr; i++) {
      m[IDXND(i, col, nr, nc)] = v[i];
   }
}

void MatrixNd::getColumn(int col, VectorNd& v) const {
   for (size_t i = 0; i < nr; i++) {
      v[i] = m[IDXND(i, col, nr, nc)];
   }
}

void MatrixNd::setRow(int row, const VectorNd& v) {
   for (size_t j = 0; j < nc; j++) {
      m[IDXND(row, j, nr, nc)] = v[j];
   }
}

void MatrixNd::getRow(int row, VectorNd& v) const {
   for (size_t j = 0; j < nc; j++) {
      v[j] = m[IDXND(row, j, nr, nc)];
   }
}

void MatrixNd::add(const MatrixNd& mat) {
   for (size_t i = 0; i < nr * nc; i++) {
      m[i] += mat.m[i];
   }
}

void MatrixNd::add(const MatrixNd& mat1, const MatrixNd& mat2) {
   for (size_t i = 0; i < nr*nc; i++) {
      m[i] = mat1.m[i]+mat2.m[i];
   }
}

void MatrixNd::subtract(const MatrixNd& mat) {
   for (size_t i = 0; i < nr * nc; i++) {
      m[i] -= mat.m[i];
   }
}

void MatrixNd::subtract(const MatrixNd& mat1, const MatrixNd& mat2) {
   for (size_t i = 0; i < nr*nc; i++) {
      m[i] = mat1.m[i]-mat2.m[i];
   }
}

void MatrixNd::scaledAdd(double s, const MatrixNd& mat) {
   for (size_t i = 0; i < nr * nc; i++) {
      m[i] += s * mat.m[i];
   }
}

void MatrixNd::scale(double s) {
   for (size_t i = 0; i < nr * nc; i++) {
      m[i] = s * m[i];
   }
}

void MatrixNd::scaleColumn(int col, double s) {
   for (size_t i = 0; i < nr; i++) {
      m[IDXND(i, col, nr, nc)] = s * m[IDXND(i, col, nr, nc)];
   }
}

void MatrixNd::scaleRow(int row, double s) {
   for (size_t j = 0; j < nc; j++) {
      m[IDXND(row, j, nr, nc)] = s * m[IDXND(row, j, nr, nc)];
   }
}

void MatrixNd::multiply(const MatrixNd& right) {
   multiply(*this, right);
}

void MatrixNd::multiply(const MatrixNd& left, const MatrixNd& right) {

   // store for safety, so can multiply by one's self
   size_t nnr = left.nr;
   size_t nnc = right.nc;
   std::vector<double> nm(nnr * nnc);

   for (size_t i = 0; i < nnr; i++) {
      for (size_t j = 0; j < nnc; j++) {
         size_t idx = IDXND(i, j, nnr, nnc);
         nm[idx] = 0;
         for (size_t k = 0; j < left.nc; k++) {
            nm[idx] += left.m[IDXND(i, k, left.nr, left.nc)]
                  * right.m[IDXND(k, j, right.nr, right.nc)];
         }
      }
   }

   nr = nnr;
   nc = nnc;
   m = std::move(nm);

}

void MatrixNd::multiply(const VectorNd& pnt, VectorNd& out) const {

   const VectorNd tmp = pnt;
   for (size_t i = 0; i < nr; i++) {
      out[i] = 0;
      for (size_t j = 0; j < nc; j++) {
         out[i] += m[IDXND(i, j, nr, nc)] * tmp[j];
      }
   }
}

void MatrixNd::multiplyLeft(const VectorNd& pnt, VectorNd& out) const {

   const VectorNd tmp = pnt;
   for (size_t j = 0; j < nc; j++) {
      out[j] = 0;
      for (size_t i = 0; i < nr; i++) {
         out[j] += m[IDXND(i, j, nr, nc)] * tmp[i];
      }
   }
}

void MatrixNd::transpose() {
   if (nr == nc) {
      for (size_t i=0; i<nr; i++) {
         for (size_t j=i+1; j<nc; j++) {
            std::swap( m[IDXND(i,j,nr,nr)], m[IDXND(j,i,nr,nr)] );
         }
      }
   } else {
      size_t n = nr*nc;
      std::vector<double> nm(n);

      int idx1=0;
      int idx2 = 0;
      for (size_t i=0; i<nr; i++) {
         idx2 = i;
         for (size_t j=0; j<nc; j++) {
            nm[idx1++] = m[idx2];
            idx2+=nr;
         }
      }

      m = std::move(nm);
      std::swap(nr, nc);
   }

}
void MatrixNd::setZero() {
   std::fill(m.begin(), m.end(), 0);
}

const Matrix4d Matrix4d::IDENTITY = Matrix4d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 1);

Matrix4d::Matrix4d() :
      m() {
}

Matrix4d::Matrix4d(const Matrix4d& copyMe) :
      m(copyMe.m) {
}

Matrix4d::Matrix4d(Matrix4d&& moveMe) :
      m(std::move(moveMe.m)) {
}

Matrix4d::Matrix4d(const double m[]) :
      m() {
   set(m);
}

Matrix4d::Matrix4d(double m00, double m01, double m02, double m03, double m10,
      double m11, double m12, double m13, double m20, double m21, double m22,
      double m23, double m30, double m31, double m32, double m33) :
      m() {
   set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
         m32, m33);
}

Matrix4d& Matrix4d::operator=(const Matrix4d& assignMe) {
   m = assignMe.m;
   return *this;
}

Matrix4d& Matrix4d::operator=(Matrix4d&& moveMe) {
   m = std::move(moveMe.m);
   return *this;
}

double& Matrix4d::operator()(size_t i, size_t j) {
   return m[IDX4D(i, j)];
}

const double& Matrix4d::operator()(size_t i, size_t j) const {
   return m[IDX4D(i, j)];
}

double& Matrix4d::operator[](size_t i) {
   return m[i];
}

const double& Matrix4d::operator[](size_t i) const {
   return m[i];
}

size_t Matrix4d::rows() const {
   return 4;
}

size_t Matrix4d::cols() const {
   return 4;
}

size_t Matrix4d::size() const {
   return IDX4D_N;
}

void Matrix4d::set(const Matrix4d& mat) {
   m = mat.m;
}

void Matrix4d::set(const double mat[]) {
   std::copy(mat, mat + IDX4D_N, m.begin());
}

void Matrix4d::set(double m00, double m01, double m02, double m03, double m10,
      double m11, double m12, double m13, double m20, double m21, double m22,
      double m23, double m30, double m31, double m32, double m33) {
   m[IDX4D_00] = m00;
   m[IDX4D_01] = m01;
   m[IDX4D_02] = m02;
   m[IDX4D_03] = m03;
   m[IDX4D_10] = m10;
   m[IDX4D_11] = m11;
   m[IDX4D_12] = m12;
   m[IDX4D_13] = m13;
   m[IDX4D_20] = m20;
   m[IDX4D_21] = m21;
   m[IDX4D_22] = m22;
   m[IDX4D_23] = m23;
   m[IDX4D_30] = m30;
   m[IDX4D_31] = m31;
   m[IDX4D_32] = m32;
   m[IDX4D_33] = m33;
}

void Matrix4d::setColumn(int col, const Vector4d& v) {
   m[IDX4D(0, col)] = v.w;
   m[IDX4D(1, col)] = v.x;
   m[IDX4D(2, col)] = v.y;
   m[IDX4D(3, col)] = v.z;
}

void Matrix4d::getColumn(int col, Vector4d& v) const {
   v.w = m[IDX4D(0, col)];
   v.x = m[IDX4D(1, col)];
   v.y = m[IDX4D(2, col)];
   v.z = m[IDX4D(3, col)];
}

void Matrix4d::setRow(int row, const Vector4d& v) {
   m[IDX4D(row, 0)] = v.w;
   m[IDX4D(row, 1)] = v.x;
   m[IDX4D(row, 2)] = v.y;
   m[IDX4D(row, 3)] = v.z;
}

void Matrix4d::getRow(int row, Vector4d& v) const {
   v.w = m[IDX4D(row, 0)];
   v.x = m[IDX4D(row, 1)];
   v.y = m[IDX4D(row, 2)];
   v.z = m[IDX4D(row, 3)];
}

void Matrix4d::add(const Matrix4d& mat) {
   for (int i = 0; i < IDX4D_N; i++) {
      m[i] += mat.m[i];
   }
}

void Matrix4d::add(const Matrix4d& mat1, const Matrix4d& mat2) {
   for (int i = 0; i < IDX4D_N; i++) {
      m[i] = mat1.m[i]+mat2.m[i];
   }
}

void Matrix4d::subtract(const Matrix4d& mat) {
   for (int i = 0; i < IDX4D_N; i++) {
      m[i] -= mat.m[i];
   }
}

void Matrix4d::subtract(const Matrix4d& mat1, const Matrix4d& mat2) {
   for (int i = 0; i < IDX4D_N; i++) {
      m[i] = mat1.m[i]-mat2.m[i];
   }
}

void Matrix4d::scaledAdd(double s, const Matrix4d& mat) {
   for (int i = 0; i < IDX4D_N; i++) {
      m[i] += s * mat.m[i];
   }
}

void Matrix4d::scale(double s) {
   for (int i = 0; i < IDX4D_N; i++) {
      m[i] = s * m[i];
   }
}

void Matrix4d::scaleColumn(int col, double s) {
   m[IDX4D(0, col)] = s * m[IDX4D(0, col)];
   m[IDX4D(1, col)] = s * m[IDX4D(1, col)];
   m[IDX4D(2, col)] = s * m[IDX4D(2, col)];
   m[IDX4D(3, col)] = s * m[IDX4D(3, col)];
}

void Matrix4d::scaleRow(int row, double s) {
   m[IDX4D(row, 0)] = s * m[IDX4D(row, 0)];
   m[IDX4D(row, 1)] = s * m[IDX4D(row, 1)];
   m[IDX4D(row, 2)] = s * m[IDX4D(row, 2)];
   m[IDX4D(row, 3)] = s * m[IDX4D(row, 3)];
}

void Matrix4d::multiply(const Matrix4d& right) {
   multiply(*this, right);
}

void Matrix4d::multiply(const Matrix4d& left, const Matrix4d& right) {
   // store for safety, so can multiply by one's self
   double m00 = left.m[IDX4D_00] * right.m[IDX4D_00]
         + left.m[IDX4D_01] * right.m[IDX4D_10]
         + left.m[IDX4D_02] * right.m[IDX4D_20]
         + left.m[IDX4D_03] * right.m[IDX4D_30];
   double m10 = left.m[IDX4D_10] * right.m[IDX4D_00]
         + left.m[IDX4D_11] * right.m[IDX4D_10]
         + left.m[IDX4D_12] * right.m[IDX4D_20]
         + left.m[IDX4D_13] * right.m[IDX4D_30];
   double m20 = left.m[IDX4D_20] * right.m[IDX4D_00]
         + left.m[IDX4D_21] * right.m[IDX4D_10]
         + left.m[IDX4D_22] * right.m[IDX4D_20]
         + left.m[IDX4D_23] * right.m[IDX4D_30];
   double m30 = left.m[IDX4D_30] * right.m[IDX4D_00]
         + left.m[IDX4D_31] * right.m[IDX4D_10]
         + left.m[IDX4D_32] * right.m[IDX4D_20]
         + left.m[IDX4D_33] * right.m[IDX4D_30];

   double m01 = left.m[IDX4D_00] * right.m[IDX4D_01]
         + left.m[IDX4D_01] * right.m[IDX4D_11]
         + left.m[IDX4D_02] * right.m[IDX4D_21]
         + left.m[IDX4D_03] * right.m[IDX4D_31];
   double m11 = left.m[IDX4D_10] * right.m[IDX4D_01]
         + left.m[IDX4D_11] * right.m[IDX4D_11]
         + left.m[IDX4D_12] * right.m[IDX4D_21]
         + left.m[IDX4D_13] * right.m[IDX4D_31];
   double m21 = left.m[IDX4D_20] * right.m[IDX4D_01]
         + left.m[IDX4D_21] * right.m[IDX4D_11]
         + left.m[IDX4D_22] * right.m[IDX4D_21]
         + left.m[IDX4D_23] * right.m[IDX4D_31];
   double m31 = left.m[IDX4D_30] * right.m[IDX4D_01]
         + left.m[IDX4D_31] * right.m[IDX4D_11]
         + left.m[IDX4D_32] * right.m[IDX4D_21]
         + left.m[IDX4D_33] * right.m[IDX4D_31];

   double m02 = left.m[IDX4D_00] * right.m[IDX4D_02]
         + left.m[IDX4D_01] * right.m[IDX4D_12]
         + left.m[IDX4D_02] * right.m[IDX4D_22]
         + left.m[IDX4D_03] * right.m[IDX4D_32];
   double m12 = left.m[IDX4D_10] * right.m[IDX4D_02]
         + left.m[IDX4D_11] * right.m[IDX4D_12]
         + left.m[IDX4D_12] * right.m[IDX4D_22]
         + left.m[IDX4D_13] * right.m[IDX4D_32];
   double m22 = left.m[IDX4D_20] * right.m[IDX4D_02]
         + left.m[IDX4D_21] * right.m[IDX4D_12]
         + left.m[IDX4D_22] * right.m[IDX4D_22]
         + left.m[IDX4D_23] * right.m[IDX4D_32];
   double m32 = left.m[IDX4D_30] * right.m[IDX4D_02]
         + left.m[IDX4D_31] * right.m[IDX4D_12]
         + left.m[IDX4D_32] * right.m[IDX4D_22]
         + left.m[IDX4D_33] * right.m[IDX4D_32];

   double m03 = left.m[IDX4D_00] * right.m[IDX4D_03]
         + left.m[IDX4D_01] * right.m[IDX4D_13]
         + left.m[IDX4D_02] * right.m[IDX4D_23]
         + left.m[IDX4D_03] * right.m[IDX4D_33];
   double m13 = left.m[IDX4D_10] * right.m[IDX4D_03]
         + left.m[IDX4D_11] * right.m[IDX4D_13]
         + left.m[IDX4D_12] * right.m[IDX4D_23]
         + left.m[IDX4D_13] * right.m[IDX4D_33];
   double m23 = left.m[IDX4D_20] * right.m[IDX4D_03]
         + left.m[IDX4D_21] * right.m[IDX4D_13]
         + left.m[IDX4D_22] * right.m[IDX4D_23]
         + left.m[IDX4D_23] * right.m[IDX4D_33];
   double m33 = left.m[IDX4D_30] * right.m[IDX4D_03]
         + left.m[IDX4D_31] * right.m[IDX4D_13]
         + left.m[IDX4D_32] * right.m[IDX4D_23]
         + left.m[IDX4D_33] * right.m[IDX4D_33];

   m[IDX4D_00] = m00;
   m[IDX4D_10] = m10;
   m[IDX4D_20] = m20;
   m[IDX4D_30] = m30;

   m[IDX4D_01] = m01;
   m[IDX4D_11] = m11;
   m[IDX4D_21] = m21;
   m[IDX4D_31] = m31;

   m[IDX4D_02] = m02;
   m[IDX4D_12] = m12;
   m[IDX4D_22] = m22;
   m[IDX4D_32] = m32;

   m[IDX4D_03] = m03;
   m[IDX4D_13] = m13;
   m[IDX4D_23] = m23;
   m[IDX4D_33] = m33;
}

void Matrix4d::multiply(const Vector4d& pnt, Vector4d& out) const {

   double w = pnt.w;
   double x = pnt.x;
   double y = pnt.y;
   double z = pnt.z;

   out.w = m[IDX4D_00] * w + m[IDX4D_01] * x + m[IDX4D_02] * y
         + m[IDX4D_03] * z;
   out.x = m[IDX4D_10] * w + m[IDX4D_11] * x + m[IDX4D_12] * y
         + m[IDX4D_13] * z;
   out.y = m[IDX4D_20] * w + m[IDX4D_21] * x + m[IDX4D_22] * y
         + m[IDX4D_23] * z;
   out.z = m[IDX4D_30] * w + m[IDX4D_31] * x + m[IDX4D_32] * y
         + m[IDX4D_33] * z;
}

void Matrix4d::multiplyLeft(const Vector4d& pnt, Vector4d& out) const {

   double w = pnt.w;
   double x = pnt.x;
   double y = pnt.y;
   double z = pnt.z;

   out.w = m[IDX4D_00] * w + m[IDX4D_10] * x + m[IDX4D_20] * y
         + m[IDX4D_30] * z;
   out.x = m[IDX4D_01] * w + m[IDX4D_11] * x + m[IDX4D_21] * y
         + m[IDX4D_31] * z;
   out.y = m[IDX4D_02] * w + m[IDX4D_12] * x + m[IDX4D_22] * y
         + m[IDX4D_32] * z;
   out.z = m[IDX4D_03] * w + m[IDX4D_13] * x + m[IDX4D_23] * y
         + m[IDX4D_33] * z;
}

void Matrix4d::outerProduct(const Vector4d& v1, const Vector4d& v2) {
   m[IDX4D_00] = v1.w * v2.w;
   m[IDX4D_01] = v1.w * v2.x;
   m[IDX4D_02] = v1.w * v2.y;
   m[IDX4D_03] = v1.w * v2.z;

   m[IDX4D_10] = v1.x * v2.w;
   m[IDX4D_11] = v1.x * v2.x;
   m[IDX4D_12] = v1.x * v2.y;
   m[IDX4D_13] = v1.x * v2.z;

   m[IDX4D_20] = v1.y * v2.w;
   m[IDX4D_21] = v1.y * v2.x;
   m[IDX4D_22] = v1.y * v2.y;
   m[IDX4D_23] = v1.y * v2.z;

   m[IDX4D_30] = v1.z * v2.w;
   m[IDX4D_31] = v1.z * v2.x;
   m[IDX4D_32] = v1.z * v2.y;
   m[IDX4D_33] = v1.z * v2.z;
}

void Matrix4d::addOuterProduct(const Vector4d& v1, const Vector4d& v2) {
   m[IDX4D_00] += v1.w * v2.w;
   m[IDX4D_01] += v1.w * v2.x;
   m[IDX4D_02] += v1.w * v2.y;
   m[IDX4D_03] += v1.w * v2.z;

   m[IDX4D_10] += v1.x * v2.w;
   m[IDX4D_11] += v1.x * v2.x;
   m[IDX4D_12] += v1.x * v2.y;
   m[IDX4D_13] += v1.x * v2.z;

   m[IDX4D_20] += v1.y * v2.w;
   m[IDX4D_21] += v1.y * v2.x;
   m[IDX4D_22] += v1.y * v2.y;
   m[IDX4D_23] += v1.y * v2.z;

   m[IDX4D_30] += v1.z * v2.w;
   m[IDX4D_31] += v1.z * v2.x;
   m[IDX4D_32] += v1.z * v2.y;
   m[IDX4D_33] += v1.z * v2.z;
}

void Matrix4d::addScaledOuterProduct(double s, const Vector4d& v1, const Vector4d& v2) {
   m[IDX4D_00] += s*v1.w * v2.w;
   m[IDX4D_01] += s*v1.w * v2.x;
   m[IDX4D_02] += s*v1.w * v2.y;
   m[IDX4D_03] += s*v1.w * v2.z;

   m[IDX4D_10] += s*v1.x * v2.w;
   m[IDX4D_11] += s*v1.x * v2.x;
   m[IDX4D_12] += s*v1.x * v2.y;
   m[IDX4D_13] += s*v1.x * v2.z;

   m[IDX4D_20] += s*v1.y * v2.w;
   m[IDX4D_21] += s*v1.y * v2.x;
   m[IDX4D_22] += s*v1.y * v2.y;
   m[IDX4D_23] += s*v1.y * v2.z;

   m[IDX4D_30] += s*v1.z * v2.w;
   m[IDX4D_31] += s*v1.z * v2.x;
   m[IDX4D_32] += s*v1.z * v2.y;
   m[IDX4D_33] += s*v1.z * v2.z;
}

double Matrix4d::determinant() const {

   double a = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15]
         + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];

   double b = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15]
         - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];

   double c = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15]
         + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];

   double d = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14]
         - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
   double det = m[0] * a + m[1] * b + m[2] * c + m[3] * d;
   return det;
}

void Matrix4d::transpose() {
   std::swap(m[IDX4D_01], m[IDX4D_10]);
   std::swap(m[IDX4D_02], m[IDX4D_20]);
   std::swap(m[IDX4D_03], m[IDX4D_30]);
   std::swap(m[IDX4D_12], m[IDX4D_21]);
   std::swap(m[IDX4D_13], m[IDX4D_31]);
   std::swap(m[IDX4D_23], m[IDX4D_32]);
}

double Matrix4d::invert() {

   double inv[IDX4D_N];

   // determinant method
   inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15]
         + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];

   inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15]
         - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];

   inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15]
         + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];

   inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14]
         - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

   inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15]
         - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];

   inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15]
         + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];

   inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15]
         - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];

   inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14]
         + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

   inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15]
         + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];

   inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15]
         - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];

   inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15]
         + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];

   inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14]
         - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

   inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11]
         - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];

   inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11]
         + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];

   inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11]
         - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];

   inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10]
         + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

   double det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

   if (det == 0) {
      return 0;
   }

   det = 1.0 / det;

   for (int i = 0; i < IDX4D_N; i++) {
      m[i] = inv[i] * det;
   }

   return det;
}

void Matrix4d::setIdentity() {

   m[IDX4D_00] = 1;
   m[IDX4D_10] = 0;
   m[IDX4D_20] = 0;
   m[IDX4D_30] = 0;

   m[IDX4D_01] = 0;
   m[IDX4D_11] = 1;
   m[IDX4D_21] = 0;
   m[IDX4D_31] = 0;

   m[IDX4D_02] = 0;
   m[IDX4D_12] = 0;
   m[IDX4D_22] = 1;
   m[IDX4D_32] = 0;

   m[IDX4D_03] = 0;
   m[IDX4D_13] = 0;
   m[IDX4D_23] = 0;
   m[IDX4D_33] = 1;
}

void Matrix4d::setZero() {

   m[IDX4D_00] = 0;
   m[IDX4D_10] = 0;
   m[IDX4D_20] = 0;
   m[IDX4D_30] = 0;

   m[IDX4D_01] = 0;
   m[IDX4D_11] = 0;
   m[IDX4D_21] = 0;
   m[IDX4D_31] = 0;

   m[IDX4D_02] = 0;
   m[IDX4D_12] = 0;
   m[IDX4D_22] = 0;
   m[IDX4D_32] = 0;

   m[IDX4D_03] = 0;
   m[IDX4D_13] = 0;
   m[IDX4D_23] = 0;
   m[IDX4D_33] = 0;
}

const RotationMatrix3d RotationMatrix3d::IDENTITY = RotationMatrix3d(1, 0, 0, 0,
      1, 0, 0, 0, 1);

RotationMatrix3d::RotationMatrix3d() {
   setIdentity();
}

RotationMatrix3d::RotationMatrix3d(const RotationMatrix3d& copyMe) :
      Matrix3d(copyMe) {
}

RotationMatrix3d::RotationMatrix3d(const double m[]) :
      Matrix3d(m) {
}

RotationMatrix3d::RotationMatrix3d(double m00, double m01, double m02,
      double m10, double m11, double m12, double m20, double m21, double m22) :
      Matrix3d(m00, m01, m02, m10, m11, m12, m20, m21, m22) {
}

RotationMatrix3d& RotationMatrix3d::operator=(const Matrix3d& assignMe) {
   m = assignMe.m;
   return *this;
}

RotationMatrix3d& RotationMatrix3d::operator=(Matrix3d&& moveMe) {
   m = std::move(moveMe.m);
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
   return (fabs(determinant() - 1) < MAS_MATH_MACHINE_PRECISION);
}

void RotationMatrix3d::setZero() {
   setIdentity();
}

const RigidTransform3d RigidTransform3d::IDENTITY = RigidTransform3d(
      RotationMatrix3d::IDENTITY, Vector3d::ZERO);

RigidTransform3d::RigidTransform3d() :
      R(RotationMatrix3d::IDENTITY), t(Vector3d::ZERO) {
}

RigidTransform3d::RigidTransform3d(const RigidTransform3d& copyMe) :
      R(copyMe.R), t(copyMe.t) {
}

RigidTransform3d::RigidTransform3d(const RotationMatrix3d& R, const Vector3d& t) :
      R(R), t(t) {
}

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
:
      dir(1, 0, 0), origin(0, 0, 0) {
}

Line::Line(const Line& other) :
      dir(other.dir), origin(other.origin) {
}

Line::Line(const Point3d& origin, const Vector3d& dir) :
      dir(dir), origin(origin) {
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

void Line::set(const Point3d& origin, const Vector3d& dir) {
   this->origin.set(origin);
   this->dir.set(dir);
   this->dir.normalize();
}

bool Line::set(const Point3d& p0, const Point3d& p1) {
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
   p1.subtract(origin);
   p1.scaledAdd(origin, p1.dot(dir), dir);
   return p1.distance(p);
}

double Line::distance(double x, double y, double z) const {
   Point3d p1(x, y, z);
   p1.subtract(origin);
   p1.scaledAdd(origin, p1.dot(dir), dir);
   return p1.distance(x, y, z);
}

// Plane implementation
Plane::Plane() // default to x-y plane
:
      normal(Vector3d(0, 0, 1)), d(0) {
}

Plane::Plane(const Plane& other) :
      normal(other.normal), d(other.d) {
}

Plane::Plane(double a, double b, double c, double d) :
      normal(a,b,c), d(d) {
   this->normal.normalize();
}

Plane::Plane(const Vector3d& normal, double d) :
      normal(normal), d(d) {
   this->normal.normalize();
}

Plane::Plane(const Vector3d& normal, const Point3d& pnt) :
      normal(normal), d(-normal.dot(pnt.x, pnt.y, pnt.z)) {
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

void Plane::set(const Vector3d& normal, double d) {
   this->normal.set(normal);
   this->d = d;
   this->normal.normalize();
}

void Plane::set(double a, double b, double c, double d) {
   this->normal.set(a,b,c);
   this->d = d;
   this->normal.normalize();
}

bool Plane::set(const Point3d& p0, const Point3d& p1, const Point3d& p2) {
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
   return normal.dot(p) + d;
}

double Plane::distanceSigned(double x, double y, double z) const {
   return normal.x * x + normal.y * y + normal.z * z + d;
}

double Plane::distance(const Point3d& p) const {
   return fabs(distanceSigned(p));
}

double Plane::distance(double x, double y, double z) const {
   return fabs(distanceSigned(x, y, z));
}

}
