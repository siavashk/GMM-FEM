#ifndef MAS_BASE_H
#define MAS_BASE_H

#include <stdio.h>
#include <string>
#include <vector>
#include <array>

#ifdef MAS_DEBUG
#define LOG(fmt,...) \
      do { \
         fprintf(stderr, "%s:%d:%s(): " fmt "\n", __FILE__, \
               __LINE__, __func__, ## __VA_ARGS__); \
               fflush(stderr); \
      }  \
      while (false)
#else
#define LOG(fmt, ...) 0
#endif

// column-major format
#ifdef MATRIX_ROW_MAJOR
#define IDX2D(i,j)   2*i+j
#define IDX2D_00  0
#define IDX2D_10  2
#define IDX2D_01  1
#define IDX2D_11  3
#define IDX2D_N   4

#define IDX3D(i,j)	3*i+j
#define IDX3D_00	0
#define IDX3D_10	3
#define IDX3D_20	6
#define IDX3D_01	1
#define IDX3D_11	4
#define IDX3D_21	7
#define IDX3D_02	2
#define IDX3D_12	5
#define IDX3D_22	8
#define IDX3D_N		9

#define IDX4D(i,j)	4*i+j
#define IDX4D_00	0
#define IDX4D_10	4
#define IDX4D_20	8
#define IDX4D_30	12
#define IDX4D_01	1
#define IDX4D_11	5
#define IDX4D_21	9
#define IDX4D_31	13
#define IDX4D_02	2
#define IDX4D_12	6
#define IDX4D_22	10
#define IDX4D_32	14
#define IDX4D_03	3
#define IDX4D_13	7
#define IDX4D_23	11
#define IDX4D_33	15
#define IDX4D_N		16

#define IDXND(i,j,r,c) i*c+j
#else
#define MATRIX_COL_MAJOR
#define IDX2D(i,j)   2*j+i
#define IDX2D_00  0
#define IDX2D_10  1
#define IDX2D_01  2
#define IDX2D_11  3
#define IDX2D_N   4

#define IDX3D(i,j)	3*j+i
#define IDX3D_00	0
#define IDX3D_10	1
#define IDX3D_20	2
#define IDX3D_01	3
#define IDX3D_11	4
#define IDX3D_21	5
#define IDX3D_02	6
#define IDX3D_12	7
#define IDX3D_22	8
#define IDX3D_N	9

#define IDX4D(i,j)	4*j+i
#define IDX4D_00	0
#define IDX4D_10	1
#define IDX4D_20	2
#define IDX4D_30	3
#define IDX4D_01	4
#define IDX4D_11	5
#define IDX4D_21	6
#define IDX4D_31	7
#define IDX4D_02	8
#define IDX4D_12	9
#define IDX4D_22	10
#define IDX4D_32	11
#define IDX4D_03	12
#define IDX4D_13	13
#define IDX4D_23	14
#define IDX4D_33	15
#define IDX4D_N	16

#define IDXND(i,j,r,c)  i+j*r
#endif

namespace mas {

/**
 * 2D Vector implementation
 */
class Vector2d {
public:
   double x, y;
   static const Vector2d ZERO;
   static const Vector2d X_AXIS;
   static const Vector2d Y_AXIS;

public:
   Vector2d();
   Vector2d(const Vector2d& copyMe);
   Vector2d(double x, double y);
   Vector2d& operator=(const Vector2d& assignMe);

   size_t size() const;

   void set(const Vector2d& v);
   void set(double x, double y);
   void setZero();

   // access
   const double& operator()(size_t idx) const;
   double& operator()(size_t idx);
   const double& operator[](size_t idx) const;
   double& operator[](size_t idx);

   void add(const Vector2d& v);
   void add(double x, double y);
   void add(const Vector2d& v1, const Vector2d& v2);
   void subtract(const Vector2d& v);
   void subtract(const Vector2d& v1, const Vector2d& v2);
   void scaledAdd(double s, const Vector2d& v2);
   void scaledAdd(const Vector2d& v1, double s, const Vector2d& v2);
   void scale(double s);
   void scale(double s, const Vector2d& v);
   void negate();

   double dot(const Vector2d& v) const;
   double dot(double x, double y) const;
   double norm() const;
   double normSquared() const;
   bool normalize();

   void interpolate(const Vector2d& v1, double t, const Vector2d& v2);

   std::string toString(const std::string fmt) const;
   std::string toString(const std::string fmt, const std::string backfmt) const;

};

/**
 * 3D vector implementation
 */
class Vector3d {
public:
   double x, y, z;
   static const Vector3d ZERO;    //< (0,0,0)
   static const Vector3d X_AXIS;  //< (1,0,0)
   static const Vector3d Y_AXIS;  //< (0,1,0)
   static const Vector3d Z_AXIS;  //< (0,0,1)

public:
   Vector3d();
   Vector3d(const Vector3d& copyMe);
   Vector3d(double x, double y, double z);
   virtual Vector3d& operator=(const Vector3d& assignMe);

   size_t size() const;

   void set(const Vector3d& pnt);
   void set(double x, double y, double z);
   void setZero();

   const double& operator()(size_t idx) const;
   double& operator()(size_t idx);
   const double& operator[](size_t idx) const;
   double& operator[](size_t idx);

   void add(const Vector3d& v);
   void add(double x, double y, double z);
   void add(const Vector3d& v1, const Vector3d& v2);
   void subtract(const Vector3d& v);
   void subtract(const Vector3d& v1, const Vector3d& v2);
   void scaledAdd(double s, const Vector3d& v2);
   void scaledAdd(const Vector3d& v1, double s, const Vector3d& v2);
   void scale(double s);
   void scale(double s, const Vector3d& v);
   void negate();

   double dot(const Vector3d& v) const;
   double dot(double x, double y, double z) const;
   void cross(const Vector3d& v);
   void cross(const Vector3d& v1, const Vector3d& v2);
   double norm() const;
   double normSquared() const;
   bool normalize();

   void interpolate(const Vector3d& v1, double t, const Vector3d& v2);

   std::string toString(const std::string fmt) const;
   std::string toString(const std::string fmt, const std::string backfmt) const;
};

class Vector4d {
public:
   double w, x, y, z;
   static const Vector4d ZERO;
   static const Vector4d W_AXIS;
   static const Vector4d X_AXIS;
   static const Vector4d Y_AXIS;
   static const Vector4d Z_AXIS;

public:
   Vector4d();
   Vector4d(const Vector4d& copyMe);
   Vector4d(double w, double x, double y, double z);
   Vector4d& operator=(const Vector4d& assignMe);

   size_t size() const;

   void set(const Vector4d& pnt);
   void set(double w, double x, double y, double z);
   void setZero();

   const double& operator()(size_t idx) const;
   double& operator()(size_t idx);
   const double& operator[](size_t idx) const;
   double& operator[](size_t idx);

   void add(const Vector4d& v);
   void add(double w, double x, double y, double z);
   void add(const Vector4d& v1, const Vector4d& v2);
   void subtract(const Vector4d& v);
   void subtract(const Vector4d& v1, const Vector4d& v2);
   void scaledAdd(double s, const Vector4d& v2);
   void scaledAdd(const Vector4d& v1, double s, const Vector4d& v2);
   void scale(double s);
   void scale(double s, const Vector4d& v);
   void negate();

   double dot(const Vector4d& v) const;
   double dot(double w, double x, double y, double z) const;
   double norm() const;
   double normSquared() const;
   bool normalize();

   void interpolate(const Vector4d& v1, double t, const Vector4d& v2);

   std::string toString(const std::string fmt) const;
   std::string toString(const std::string fmt, const std::string backfmt) const;
};

class VectorNd {
private:
   std::vector<double> v;

public:
   VectorNd();
   VectorNd(size_t size);
   VectorNd(const VectorNd& copyMe);
   VectorNd(const Vector2d& copyMe);
   VectorNd(const Vector3d& copyMe);
   VectorNd(const Vector4d& copyMe);
   VectorNd(VectorNd&& moveMe);
   VectorNd& operator=(const VectorNd& assignMe);
   VectorNd& operator=(const Vector2d& assignMe);
   VectorNd& operator=(const Vector3d& assignMe);
   VectorNd& operator=(const Vector4d& assignMe);
   VectorNd& operator=(VectorNd&& moveMe);

   void resize(size_t n);
   size_t size() const;

   double* data();

   void set(const VectorNd& v);
   void setZero();

   const double& operator()(size_t idx) const;
   double& operator()(size_t idx);
   const double& operator[](size_t idx) const;
   double& operator[](size_t idx);

   void add(const VectorNd& v);
   void add(const VectorNd& v1, const VectorNd& v2);
   void subtract(const VectorNd& v);
   void subtract(const VectorNd& v1, const VectorNd& v2);
   void scaledAdd(double s, const VectorNd& v2);
   void scaledAdd(const VectorNd& v1, double s, const VectorNd& v2);
   void scale(double s);
   void scale(double s, const VectorNd& v);
   void negate();

   double dot(const VectorNd& v) const;
   double norm() const;
   double normSquared() const;
   bool normalize();

   void interpolate(const VectorNd& v1, double t, const VectorNd& v2);

   std::string toString(const std::string fmt) const;
   std::string toString(const std::string fmt, const std::string backfmt) const;

};

class Point3d: public Vector3d {
public:
   Point3d();
   Point3d(const Point3d& copyMe);
   Point3d(const Vector3d& pointMe);
   Point3d(double x, double y, double z);
   virtual Point3d& operator=(const Point3d& assignMe);

   double distance(const Point3d& d) const;
   double distance(double x, double y, double z) const;
   double distanceSquared(const Point3d& d) const;
   double distanceSquared(double x, double y, double z) const;
};

class IndexedPoint3d: public Point3d {
public:
   size_t idx;
public:
   IndexedPoint3d();
   IndexedPoint3d(const IndexedPoint3d& copyMe);
   IndexedPoint3d(const Vector3d& pointMe, size_t index);
   IndexedPoint3d(double x, double y, double z, size_t index);
   virtual IndexedPoint3d& operator=(const IndexedPoint3d& assignMe);

   size_t getIndex() const;
   void setIndex(size_t index);

   void set(const Vector3d& pnt);
   void set(double x, double y, double z);
   void set(double x, double y, double z, size_t index);
};

// Generic 2D matrix
class Matrix2d {
public:
   std::array<double, IDX2D_N> m;   // column-major format
public:
   static const Matrix2d IDENTITY;
public:
   Matrix2d();
   Matrix2d(const Matrix2d& copyMe);
   Matrix2d(Matrix2d&& moveMe);
   Matrix2d(const double m[]);   // column-major
   Matrix2d(double m00, double m01, double m10, double m11);
   Matrix2d& operator=(const Matrix2d& assignMe);
   Matrix2d& operator=(Matrix2d&& moveMe);

   size_t rows() const;
   size_t cols() const;
   size_t size() const;

   double& operator()(size_t i, size_t j);
   const double& operator()(size_t i, size_t j) const;
   double& operator[](size_t i);
   const double& operator[](size_t i) const;

   void set(const Matrix2d& mat);
   void set(const double m[]);
   void set(double m00, double m01, double m10, double m11);

   void setColumn(int col, const Vector2d& v);
   void getColumn(int col, Vector2d& v) const;
   void setRow(int row, const Vector2d& v);
   void getRow(int row, Vector2d& v) const;

   void add(const Matrix2d& mat);
   void add(const Matrix2d& mat1, const Matrix2d& mat2);
   void subtract(const Matrix2d& mat);
   void subtract(const Matrix2d& mat1, const Matrix2d& mat2);
   void scaledAdd(double s, const Matrix2d& mat);

   void scale(double s);
   void scaleColumn(int col, double s);
   void scaleRow(int row, double s);

   void multiply(const Matrix2d& right);
   void multiply(const Matrix2d& left, const Matrix2d& right);
   void multiply(const Vector2d& pnt, Vector2d& out) const;
   void multiplyLeft(const Vector2d& pnt, Vector2d& out) const;

   void outerProduct(const Vector2d& v1, const Vector2d& v2);
   void addOuterProduct(const Vector2d& v1, const Vector2d& v2);
   void addScaledOuterProduct(double s, const Vector2d& v1, const Vector2d& v2);

   double determinant() const;
   double condition() const;

   virtual void transpose();
   virtual double invert();
   virtual double pseudoInvert();   // using SVD

   void setIdentity();
   virtual void setZero();
};

// Generic 3D matrix
class Matrix3d {
public:
   std::array<double, IDX3D_N> m;	// column-major format
public:
   static const Matrix3d IDENTITY;
public:
   Matrix3d();
   Matrix3d(const Matrix3d& copyMe);
   Matrix3d(Matrix3d&& moveMe);
   Matrix3d(const double m[]);	// column-major
   Matrix3d(double m00, double m01, double m02, double m10, double m11,
         double m12, double m20, double m21, double m22);
   virtual Matrix3d& operator=(const Matrix3d& assignMe);
   virtual Matrix3d& operator=(Matrix3d&& moveMe);

   size_t rows() const;
   size_t cols() const;
   size_t size() const;

   double& operator()(size_t i, size_t j);
   const double& operator()(size_t i, size_t j) const;
   double& operator[](size_t i);
   const double& operator[](size_t i) const;

   void set(const Matrix3d& mat);
   void set(const double m[]);
   void set(double m00, double m01, double m02, double m10, double m11,
         double m12, double m20, double m21, double m22);

   void setColumn(int col, const Vector3d& v);
   void getColumn(int col, Vector3d& v) const;
   void setRow(int row, const Vector3d& v);
   void getRow(int row, Vector3d& v) const;

   void add(const Matrix3d& mat);
   void add(const Matrix3d& mat1, const Matrix3d& mat2);
   void subtract(const Matrix3d& mat);
   void subtract(const Matrix3d& mat1, const Matrix3d& mat2);
   void scaledAdd(double s, const Matrix3d& mat);

   void scale(double s);
   void scaleColumn(int col, double s);
   void scaleRow(int row, double s);

   void multiply(const Matrix3d& right);
   void multiply(const Matrix3d& left, const Matrix3d& right);
   void multiply(const Vector3d& pnt, Vector3d& out) const;
   void multiplyLeft(const Vector3d& pnt, Vector3d& out) const;

   /**
    * @brief M^T (p0 - p1)
    *
    * @param p0
    * @param p1
    * @param out
    */
   void subtractMultiplyLeft(const Vector3d& p0, const Vector3d& p1,
         Vector3d& out) const;

   /**
    * @brief M p0 + p1
    *
    * @param p0
    * @param p1
    * @param out
    */
   void multiplyAdd(const Vector3d& p0, const Vector3d& p1,
         Vector3d& out) const;

   void outerProduct(const Vector3d& v1, const Vector3d& v2);
   void addOuterProduct(const Vector3d& v1, const Vector3d& v2);
   void addScaledOuterProduct(double s, const Vector3d& v1, const Vector3d& v2);

   double determinant() const;
   double condition() const;

   void transpose();

   virtual double invert();
   virtual double pseudoInvert();	// using SVD

   void setIdentity();
   virtual void setZero();
};

// Generic 4D matrix
class Matrix4d {
public:
   std::array<double, IDX4D_N> m;   // column-major format
public:
   static const Matrix4d IDENTITY;
public:
   Matrix4d();
   Matrix4d(const Matrix4d& copyMe);
   Matrix4d(Matrix4d&& moveMe);
   Matrix4d(const double m[]);   // column-major
   Matrix4d(double m00, double m01, double m02, double m03, double m10,
         double m11, double m12, double m13, double m20, double m21, double m22,
         double m23, double m30, double m31, double m32, double m33);
   Matrix4d& operator=(const Matrix4d& assignMe);
   Matrix4d& operator=(Matrix4d&& moveMe);

   size_t rows() const;
   size_t cols() const;
   size_t size() const;

   double& operator()(size_t i, size_t j);
   const double& operator()(size_t i, size_t j) const;
   double& operator[](size_t i);
   const double& operator[](size_t i) const;

   void set(const Matrix4d& mat);
   void set(const double m[]); // column-major
   void set(double m00, double m01, double m02, double m03, double m10,
         double m11, double m12, double m13, double m20, double m21, double m22,
         double m23, double m30, double m31, double m32, double m33);

   void setColumn(int col, const Vector4d& v);
   void getColumn(int col, Vector4d& v) const;
   void setRow(int row, const Vector4d& v);
   void getRow(int row, Vector4d& v) const;

   void add(const Matrix4d& mat);
   void add(const Matrix4d& mat1, const Matrix4d& mat2);
   void subtract(const Matrix4d& mat);
   void subtract(const Matrix4d& mat1, const Matrix4d& mat2);
   void scaledAdd(double s, const Matrix4d& mat);

   void scale(double s);
   void scaleColumn(int col, double s);
   void scaleRow(int row, double s);

   void multiply(const Matrix4d& right);
   void multiply(const Matrix4d& left, const Matrix4d& right);
   void multiply(const Vector4d& pnt, Vector4d& out) const;
   void multiplyLeft(const Vector4d& pnt, Vector4d& out) const;

   void outerProduct(const Vector4d& v1, const Vector4d& v2);
   void addOuterProduct(const Vector4d& v1, const Vector4d& v2);
   void addScaledOuterProduct(double s, const Vector4d& v1, const Vector4d& v2);

   double determinant() const;

   void transpose();
   double invert();

   void setIdentity();
   virtual void setZero();
};

// Generic ND matrix
class MatrixNd {
private:
   size_t nr;
   size_t nc;
public:
   std::vector<double> m;   // column-major format

public:
   MatrixNd();
   MatrixNd(size_t rows, size_t cols);

   MatrixNd(const MatrixNd& copyMe);
   MatrixNd(const Matrix2d& copyMe);
   MatrixNd(const Matrix3d& copyMe);
   MatrixNd(const Matrix4d& copyMe);
   MatrixNd(size_t rows, size_t cols, const double* vals);  // column-major
   MatrixNd(MatrixNd&& moveMe);

   MatrixNd& operator=(const MatrixNd& assignMe);
   MatrixNd& operator=(const Matrix2d& assignMe);
   MatrixNd& operator=(const Matrix3d& assignMe);
   MatrixNd& operator=(const Matrix4d& assignMe);
   MatrixNd& operator=(MatrixNd&& moveMe);

   size_t rows() const;
   size_t cols() const;
   size_t size() const;

   void resize(size_t rows, size_t cols);

   double& operator()(size_t i, size_t j);
   const double& operator()(size_t i, size_t j) const;
   double& operator[](size_t i);
   const double& operator[](size_t i) const;

   void set(const MatrixNd& m);
   void set(size_t rows, size_t cols, const double* vals);

   void setColumn(int col, const VectorNd& v);
   void getColumn(int col, VectorNd& v) const;
   void setRow(int row, const VectorNd& v);
   void getRow(int row, VectorNd& v) const;

   void add(const MatrixNd& mat);
   void add(const MatrixNd& mat1, const MatrixNd& mat2);
   void subtract(const MatrixNd& mat);
   void subtract(const MatrixNd& mat1, const MatrixNd& mat2);
   void scaledAdd(double s, const MatrixNd& mat);

   void scale(double s);
   void scaleColumn(int col, double s);
   void scaleRow(int row, double s);

   void multiply(const MatrixNd& right);
   void multiply(const MatrixNd& left, const MatrixNd& right);
   void multiply(const VectorNd& pnt, VectorNd& out) const;
   void multiplyLeft(const VectorNd& pnt, VectorNd& out) const;

   // double determinant() const;

   void transpose();
   // double invert();
   //void setIdentity();

   void setZero();
};

// Specific 3D matrix restricted to rotations
class RotationMatrix3d: public Matrix3d {
public:
   static const RotationMatrix3d IDENTITY;
public:
   RotationMatrix3d();
   RotationMatrix3d(const RotationMatrix3d& copyMe);
   RotationMatrix3d(const double m[]);	// column-major
   RotationMatrix3d(double m00, double m01, double m02, double m10, double m11,
         double m12, double m20, double m21, double m22);

   virtual RotationMatrix3d& operator=(const Matrix3d& assignMe);
   virtual RotationMatrix3d& operator=(Matrix3d&& moveMe);

   virtual double invert();
   virtual double pseudoInvert();

   bool isValid();	// check still represents rotation
   virtual void setZero();	// zero rotation = Identity
};

class RigidTransform3d {
public:
   RotationMatrix3d R;
   Vector3d t;
   static const RigidTransform3d IDENTITY;
public:
   RigidTransform3d();
   RigidTransform3d(const RigidTransform3d& copyMe);
   RigidTransform3d(const RotationMatrix3d& R, const Vector3d& t);
   virtual ~RigidTransform3d() {
   }
   ;
   virtual RigidTransform3d& operator=(const RigidTransform3d& assignMe);

   void set(const RotationMatrix3d& R, const Vector3d& t);
   void setRotation(const RotationMatrix3d& R);
   void setTranslation(const Vector3d& t);
   void transform(Point3d& point) const;
   void transform(Vector3d& vector) const;
   void inverseTransform(Point3d& point) const;
   void inverseTransform(Vector3d& vector) const;
   void invert();
   void multiply(const RigidTransform3d& right); //right-multiply
   void multiplyLeft(const RigidTransform3d& left);
   void multiply(const RigidTransform3d& left, const RigidTransform3d& right);

   void setIdentity();
};

class Line {
public:
   Vector3d dir;
   Point3d origin;
public:
   Line();
   Line(const Line& other);
   Line(const Point3d& origin, const Vector3d& dir);
   Line(const Point3d& p0, const Point3d& p1);
   virtual Line& operator=(const Line& assignMe);

   void set(const Point3d& origin, const Vector3d& dir);
   bool set(const Point3d& p0, const Point3d& p1);
   void flip();

   Point3d getOrigin() const;
   Vector3d getDirection() const;

   double distance(const Point3d& pnt) const;
   double distance(double x, double y, double z) const;
};

class Plane {
public:
   Vector3d normal;
   double d;
public:
   Plane();
   Plane(const Plane& other);
   Plane(const Vector3d& normal, double d);
   Plane(const Vector3d& normal, const Point3d& pnt);
   Plane(const Point3d& p0, const Point3d& p1, const Point3d& p2);
   virtual Plane& operator=(const Plane& assignMe);

   void set(const Vector3d& normal, double d);
   bool set(const Point3d& p0, const Point3d& p1, const Point3d& p2);
   void flip();

   // signed distance
   double distanceSigned(const Point3d& pnt) const;
   double distanceSigned(double x, double y, double z) const;
   double distance(const Point3d& pnt) const;
   double distance(double x, double y, double z) const;
};

}
#endif
