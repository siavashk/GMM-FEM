#ifndef MAS_BASE_H
#define MAS_BASE_H

#include <stdio.h>
#include <string>

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

// row-major format
#ifndef MATRIX_COL_MAJOR
#define MATRIX_COL_MAJOR
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
#else
#define MATRIX_ROW_MAJOR
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
#define IDX3D_N		9
#endif

namespace mas {

class Vector3d {
public:
	double x,y,z;
	static const Vector3d ZERO;
	static const Vector3d X_AXIS;
	static const Vector3d Y_AXIS;
	static const Vector3d Z_AXIS;

public:
	Vector3d();
	Vector3d(const Vector3d& copyMe);
	Vector3d(double x, double y, double z);
	virtual ~Vector3d();
	virtual Vector3d& operator=(const Vector3d& assignMe);

	void set(const Vector3d& pnt);
	void set(double x, double y, double z);
	void setZero();
	double get(int idx) const;
	void set(int idx, double val);

	void add(const Vector3d& v);
	void add(double x, double y, double z);
	void add(const Vector3d &v1, const Vector3d &v2);
	void subtract(const Vector3d& v);
	void subtract(const Vector3d &v1, const Vector3d &v2);
	void scaledAdd(double s, const Vector3d &v2);
	void scaledAdd(const Vector3d &v1, double s, const Vector3d &v2);
	void scale(double s);
	void negate();

	double dot(const Vector3d& v) const;
	double dot(double x, double y, double z) const;
	void cross(const Vector3d& v);
	void cross(const Vector3d& v1, const Vector3d& v2);
	double norm() const;
	double normSquared() const;
	bool normalize();

	void interpolate(Vector3d v1, double t, Vector3d v2);

	std::string toString(std::string fmt) const;
};

class Point3d : public Vector3d {
public:
	Point3d();
	Point3d(const Point3d& copyMe);
	Point3d(const Vector3d& pointMe);
	Point3d(double x, double y, double z);
	virtual ~Point3d();
	virtual Point3d& operator=(const Point3d& assignMe);

	double distance(const Point3d &d) const;
	double distanceSquared(const Point3d &d) const;
};

// Generic 3D matrix
class Matrix3d {
public:
	double m[9];	// row-major format
public:
	static const Matrix3d IDENTITY;
public:
	Matrix3d();
	Matrix3d(const Matrix3d &copyMe);
	Matrix3d(const double m[]);	// row-major
	Matrix3d(double m00, double m01, double m02,
			double m10, double m11, double m12,
			double m20, double m21, double m22);
	virtual ~Matrix3d() {};
	virtual Matrix3d& operator=(const Matrix3d &assignMe);

	double get(int i, int j) const;
	void set(int i, int j, double val);
	void set(const Matrix3d &mat);
	void set(const double m[]);
	void set(double m00, double m01, double m02,
			double m10, double m11, double m12,
			double m20, double m21, double m22);

	void setColumn(int col, const Vector3d &v);
	void getColumn(int col, Vector3d &v) const;
	void setRow(int row, const Vector3d &v);
	void getRow(int row, Vector3d &v) const;

	void add(const Matrix3d &mat);
	void subtract(const Matrix3d &mat);
	void scaledAdd(double s, const Matrix3d &mat);

	void scale(double s);
	void scaleColumn(int col, double s);
	void scaleRow(int row, double s);

	void multiply(const Matrix3d &right);
	void multiply(const Matrix3d &left, const Matrix3d &right);
	void multiply(const Vector3d &pnt, Vector3d &out) const;
	void multiplyLeft(const Vector3d &pnt, Vector3d &out) const;

	void outerProduct(const Vector3d &v1, const Vector3d &v2);
	void addOuterProduct(const Vector3d &v1, const Vector3d &v2);

	double determinant() const;
	double condition() const;
	virtual void transpose();
	virtual double invert();
	virtual double pseudoInvert();	// using SVD

			void setIdentity();
			virtual void setZero();
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
	virtual ~Plane();
	virtual Plane& operator=(const Plane& assignMe);

	void set (const Vector3d& normal, double d);
	bool set (const Point3d& p0, const Point3d& p1, const Point3d& p2);
	void flip();

	// signed distance
	double distanceSigned(const Point3d &pnt) const;
	double distanceSigned(double x, double y, double z) const;
	double distance(const Point3d &pnt) const;
	double distance(double x, double y, double z) const;
};

// Specific 3D matrix restricted to rotations
class RotationMatrix3d : public Matrix3d {
public:
	static const RotationMatrix3d IDENTITY;
public:
	RotationMatrix3d();
	RotationMatrix3d(const RotationMatrix3d &copyMe);
	RotationMatrix3d(const double m[]);	// row-major
	RotationMatrix3d(double m00, double m01, double m02,
			double m10, double m11, double m12,
			double m20, double m21, double m22);
	virtual ~RotationMatrix3d() {};
	virtual RotationMatrix3d& operator=(const RotationMatrix3d &assignMe);
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
	RigidTransform3d(const RigidTransform3d &copyMe);
	RigidTransform3d(const RotationMatrix3d &R, const Vector3d &t);
	virtual ~RigidTransform3d() {};
	virtual RigidTransform3d& operator=(
			const RigidTransform3d &assignMe);

	void set(const RotationMatrix3d &R, const Vector3d &t);
	void setRotation(const RotationMatrix3d &R);
	void setTranslation(const Vector3d &t);
	void transform(Point3d &point) const;
	void transform(Vector3d &vector) const;
	void inverseTransform(Point3d &point) const;
	void inverseTransform(Vector3d &vector) const;
	void invert();
	void multiply(const RigidTransform3d &right); //right-multiply
	void multiplyLeft(const RigidTransform3d &left);
	void multiply(const RigidTransform3d &left,
			const RigidTransform3d &right);

	void setIdentity();
};

}
#endif
