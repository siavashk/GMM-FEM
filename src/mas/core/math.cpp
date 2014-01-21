#include "mas/core/math.h"
#include <algorithm>
#include <math.h>

namespace mas {
namespace math {

// SVD code
// Givens rotation
void svd_givens(double a, double b, double out[]) {
	if (b==0) {
		out[0] = 1;
		out[1] = 0;
	} else {
		if (fabs(b) > fabs(a)) {
			double tau = -a/b;
			out[1] = 1.0/sqrt(1+tau*tau);
			out[0] = out[1]*tau;
		} else {
			double tau = -b/a;
			out[0] = 1.0/sqrt(1+tau*tau);
			out[1] = out[0]*tau;
		}
	}
}

void svd_rotate_cols(Matrix3d &Q, int col1, int col2, double gr[]) {

	double C = gr[0];
	double S = gr[1];
	if (col1 > col2) {
		S = -S;
		// switch columns
		int tmp = col1;
		col1 = col2;
		col2 = tmp;
	}

	for (int i=0; i<3; i++) {
		double mc1 = Q.m[IDX3D(i,col1)];
		double mc2 = Q.m[IDX3D(i,col2)];
		Q.m[IDX3D(i,col1)] = C*mc1 - S*mc2;
		Q.m[IDX3D(i,col2)] = S*mc1 + C*mc2;
	}

}

void svd_rotate_rows(Matrix3d &Q, int row1, int row2, double gr[]) {
	double C = gr[0];
	double S = gr[1];

	if (row1 > row2) {
		S = -S;
		// switch rows
		int tmp = row1;
		row1 = row2;
		row2 = tmp;
	}

	for (int i=0; i<3; i++) {
		double mr1 = Q.m[IDX3D(row1,i)];
		double mr2 = Q.m[IDX3D(row2,i)];
		Q.m[IDX3D(row1,i)] = C*mr1 - S*mr2;
		Q.m[IDX3D(row2,i)] = S*mr1 + C*mr2;
	}
}

void svd_bidiagonalize(Matrix3d &Q, Matrix3d &U, Matrix3d &V,
		double gr[]) {

	svd_givens(Q.m[IDX3D_00], Q.m[IDX3D_20], gr);
	svd_rotate_rows(Q, 0, 2, gr);

	double C1 = gr[0];
	double S1 = gr[1];

	svd_givens(Q.m[IDX3D_00], Q.m[IDX3D_10], gr);
	svd_rotate_rows(Q, 0, 1, gr);

	double C2 = gr[0];
	double S2 = gr[1];

	U.m[IDX3D_00] = C1*C2;
	U.m[IDX3D_10] = -S2;
	U.m[IDX3D_20] = -S1*C2;
	U.m[IDX3D_01] = C1*S2;
	U.m[IDX3D_11] = C2;
	U.m[IDX3D_21] = -S1*S2;
	U.m[IDX3D_02] = S1;
	U.m[IDX3D_12] = 0;
	U.m[IDX3D_22] = C1;

	svd_givens(Q.m[IDX3D_01], Q.m[IDX3D_02], gr);
	svd_rotate_cols(Q, 1, 2, gr);
	C2 = gr[0];
	S2 = gr[1];

	V.m[IDX3D_00] = 1;
	V.m[IDX3D_10] = 0;
	V.m[IDX3D_20] = 0;
	V.m[IDX3D_01] = 0;
	V.m[IDX3D_11] = C2;
	V.m[IDX3D_21] = -S2;
	V.m[IDX3D_02] = 0;
	V.m[IDX3D_12] = S2;
	V.m[IDX3D_22] = C2;

	svd_givens(Q.m[IDX3D_11], Q.m[IDX3D_21], gr);
	svd_rotate_rows(Q, 1, 2, gr);
	svd_rotate_cols(U, 1, 2, gr);

	// should be zero, set to be safe
	Q.m[IDX3D_10] = 0;
	Q.m[IDX3D_20] = 0;
	Q.m[IDX3D_21] = 0;
	Q.m[IDX3D_02] = 0;
}

// zero top row
void svd_zero_3x3(Matrix3d &Q, Matrix3d &U, double gr[]) {

	double al1 = Q.m[IDX3D_01];
	double ak1 = Q.m[IDX3D_11];
	svd_givens(ak1, al1, gr);
	svd_rotate_cols(U, 1, 0, gr);

	double C = gr[0];
	double S = gr[1];

	Q.m[IDX3D_11] = ak1*C-al1*S;
	double ak2 = Q.m[IDX3D_12];
	Q.m[IDX3D_12] = ak2*C;

	al1 = ak2*S;
	ak1 = Q.m[IDX3D_22];
	svd_givens(ak1, al1, gr);
	svd_rotate_cols(U, 2, 0, gr);

	Q.m[IDX3D_22] = ak1*gr[0] - al1*gr[1];
	Q.m[IDX3D_01] = 0;

}

// zero top row of upper 2x2 diagonal matrix
void svd_zero_upper_2x2(Matrix3d &Q, Matrix3d &U, double gr[]) {

	double al1 = Q.m[IDX3D_01];
	double ak1 = Q.m[IDX3D_11];

	svd_givens(ak1, al1, gr);
	svd_rotate_cols(U, 1, 0, gr);
	Q.m[IDX3D_11] = ak1*gr[0]-al1*gr[1];
	Q.m[IDX3D_01] = 0;

}

// zero top row of lower 2x2 diagonal matrix
void svd_zero_lower_2x2(Matrix3d &Q, Matrix3d &U, double gr[]) {

	double al1 = Q.m[IDX3D_12];
	double ak1 = Q.m[IDX3D_22];

	svd_givens(ak1, al1, gr);
	svd_rotate_cols(U, 2, 1, gr);
	Q.m[IDX3D_22] = ak1*gr[0]-al1*gr[1];
	Q.m[IDX3D_12] = 0;

}

// Golub Kahan SVD step
void svd_step_3x3(Matrix3d &Q, Matrix3d &U, Matrix3d &V, double gr[]) {

	double A00 = Q.m[IDX3D_00];
	double A01 = Q.m[IDX3D_01];
	double A11 = Q.m[IDX3D_11];
	double A12 = Q.m[IDX3D_12];
	double A22 = Q.m[IDX3D_22];
	double A02 = 0;

	double tmm = A11*A11 + A01*A01;
	double tmn = A11*A12;
	double tnn = A12*A12 + A22*A22;

	double d = (tmm-tnn)/2;
	int signd = (d >= 0 ? 1 : -1);

	double mu = tnn - tmn*tmn/(d + signd*sqrt(d*d+tmn*tmn));

	// Givens rotations
	svd_givens(A00*A00-mu, A00*A01, gr);
	double C = gr[0];
	double S = gr[1];

	double B00 = C*A00-S*A01;
	double B10 = -S*A11;
	double B01 = S*A00+C*A01;
	double B11 = C*A11;
	double B21 = 0;
	double B12 = 0;
	double B22 = 0;

	svd_rotate_cols(V, 0, 1, gr);

	svd_givens(B00, B10, gr);
	C = gr[0];
	S = gr[1];

	Q.m[IDX3D_00] = C*B00 - S*B10;
	A01 = C*B01 - S*B11;
	A02 = -S*A12;
	A11 = S*B01 + C*B11;
	A12 = C*A12;

	svd_rotate_cols(U, 0, 1, gr);

	svd_givens(A01, A02, gr);
	C = gr[0];
	S = gr[1];

	Q.m[IDX3D_01] = C*A01 - S*A02;
	B11 = C*A11 - S*A12;
	B21 = -S*A22;
	B12 = S*A11 + C*A12;
	B22 = C*A22;
	svd_rotate_cols(V, 1, 2, gr);

	svd_givens(B11, B21, gr);
	C = gr[0];
	S = gr[1];
	Q.m[IDX3D_11] = C*B11 - S*B21;
	Q.m[IDX3D_12] = C*B12 - S*B22;
	Q.m[IDX3D_22] = S*B12 + C*B22;

	svd_rotate_cols(U, 1, 2, gr);
}

void svd_step_upper_2x2(Matrix3d &Q, Matrix3d &U, Matrix3d &V, double gr[]) {

	double A00 = Q.m[IDX3D_00];
	double A01 = Q.m[IDX3D_01];
	double A11 = Q.m[IDX3D_11];

	double tmm = A00*A00;
	double tmn = A00*A11;
	double tnn = A01*A01 + A11*A11;

	double d = (tmm-tnn)/2;
	int signd = (d >= 0 ? 1 : -1);
	double mu = tnn - tmn*tmn/(d + signd*sqrt(d*d+tmn*tmn));

	// Givens rotations
	svd_givens(A00*A00-mu, A00*A01, gr);
	double C = gr[0];
	double S = gr[1];

	double B00 = C*A00-S*A01;
	double B10 = -S*A11;
	double B01 = S*A00+C*A01;
	double B11 = C*A11;

	svd_rotate_cols(V, 0, 1, gr);

	svd_givens(B00, B10, gr);
	C = gr[0];
	S = gr[1];

	Q.m[IDX3D_00] = C*B00 - S*B10;
	Q.m[IDX3D_01] = C*B01 - S*B11;
	Q.m[IDX3D_11] = S*B01 + C*B11;
	svd_rotate_cols(U, 0, 1, gr);

}

void svd_step_lower_2x2(Matrix3d &Q, Matrix3d &U, Matrix3d &V, double gr[]) {

	double A11 = Q.m[IDX3D_11];
	double A12 = Q.m[IDX3D_12];
	double A22 = Q.m[IDX3D_22];

	double tmm = A11*A11;
	double tmn = A11*A12;
	double tnn = A12*A12 + A22*A22;

	double d = (tmm-tnn)/2;
	int signd = (d >= 0 ? 1 : -1);
	double mu = tnn - tmn*tmn/(d + signd*sqrt(d*d+tmn*tmn));

	// Givens rotations
	svd_givens(A11*A11-mu, A11*A12, gr);
	double C = gr[0];
	double S = gr[1];

	double B11 = C*A11-S*A12;
	double B21 = -S*A22;
	double B12 = S*A11+C*A12;
	double B22 = C*A22;

	svd_rotate_cols(V, 1, 2, gr);

	svd_givens(B11, B21, gr);
	C = gr[0];
	S = gr[1];

	Q.m[IDX3D_11] = C*B11 - S*B21;
	Q.m[IDX3D_12] = C*B12 - S*B22;
	Q.m[IDX3D_22] = S*B12 + C*B22;
	svd_rotate_cols(U, 1, 2, gr);
}

void svd_rotate_UV(Matrix3d &U, Matrix3d &V, int c0, int c1, int c2) {

	double a0, a1, a2;
	if (c0 == 0) {
		if (c1 == 2) {
			// switch 1/2
			a0 = U.m[IDX3D_01];
			a1 = U.m[IDX3D_11];
			a2 = U.m[IDX3D_21];
			U.m[IDX3D_01] = U.m[IDX3D_02];
			U.m[IDX3D_11] = U.m[IDX3D_12];
			U.m[IDX3D_21] = U.m[IDX3D_22];
			U.m[IDX3D_02] = a0;
			U.m[IDX3D_12] = a1;
			U.m[IDX3D_22] = a2;

			a0 = V.m[IDX3D_01];
			a1 = V.m[IDX3D_11];
			a2 = V.m[IDX3D_21];
			V.m[IDX3D_01] = V.m[IDX3D_02];
			V.m[IDX3D_11] = V.m[IDX3D_12];
			V.m[IDX3D_21] = V.m[IDX3D_22];
			V.m[IDX3D_02] = a0;
			V.m[IDX3D_12] = a1;
			V.m[IDX3D_22] = a2;
		}
	} else if (c0 == 1) {
		if (c1 == 0) {
			// switch 0/1
			a0 = U.m[IDX3D_00];
			a1 = U.m[IDX3D_10];
			a2 = U.m[IDX3D_20];
			U.m[IDX3D_00] = U.m[IDX3D_01];
			U.m[IDX3D_10] = U.m[IDX3D_11];
			U.m[IDX3D_20] = U.m[IDX3D_21];
			U.m[IDX3D_01] = a0;
			U.m[IDX3D_11] = a1;
			U.m[IDX3D_21] = a2;

			a0 = V.m[IDX3D_00];
			a1 = V.m[IDX3D_10];
			a2 = V.m[IDX3D_20];
			V.m[IDX3D_00] = V.m[IDX3D_01];
			V.m[IDX3D_10] = V.m[IDX3D_11];
			V.m[IDX3D_20] = V.m[IDX3D_21];
			V.m[IDX3D_01] = a0;
			V.m[IDX3D_11] = a1;
			V.m[IDX3D_21] = a2;
		} else {
			// 1, 2, 0
			a0 = U.m[IDX3D_00];
			a1 = U.m[IDX3D_10];
			a2 = U.m[IDX3D_20];
			U.m[IDX3D_00] = U.m[IDX3D_01];
			U.m[IDX3D_10] = U.m[IDX3D_11];
			U.m[IDX3D_20] = U.m[IDX3D_21];
			U.m[IDX3D_01] = U.m[IDX3D_02];
			U.m[IDX3D_11] = U.m[IDX3D_12];
			U.m[IDX3D_21] = U.m[IDX3D_22];
			U.m[IDX3D_02] = a0;
			U.m[IDX3D_12] = a1;
			U.m[IDX3D_22] = a2;

			a0 = V.m[IDX3D_00];
			a1 = V.m[IDX3D_10];
			a2 = V.m[IDX3D_20];
			V.m[IDX3D_00] = V.m[IDX3D_01];
			V.m[IDX3D_10] = V.m[IDX3D_11];
			V.m[IDX3D_20] = V.m[IDX3D_21];
			V.m[IDX3D_01] = V.m[IDX3D_02];
			V.m[IDX3D_11] = V.m[IDX3D_12];
			V.m[IDX3D_21] = V.m[IDX3D_22];
			V.m[IDX3D_02] = a0;
			V.m[IDX3D_12] = a1;
			V.m[IDX3D_22] = a2;
		}
	} else {
		if (c1 == 0) {
			// 2, 0, 1
			a0 = U.m[IDX3D_00];
			a1 = U.m[IDX3D_10];
			a2 = U.m[IDX3D_20];
			U.m[IDX3D_00] = U.m[IDX3D_02];
			U.m[IDX3D_10] = U.m[IDX3D_12];
			U.m[IDX3D_20] = U.m[IDX3D_22];
			U.m[IDX3D_02] = U.m[IDX3D_01];
			U.m[IDX3D_12] = U.m[IDX3D_11];
			U.m[IDX3D_22] = U.m[IDX3D_21];
			U.m[IDX3D_01] = a0;
			U.m[IDX3D_11] = a1;
			U.m[IDX3D_21] = a2;

			a0 = V.m[IDX3D_00];
			a1 = V.m[IDX3D_10];
			a2 = V.m[IDX3D_20];
			V.m[IDX3D_00] = V.m[IDX3D_02];
			V.m[IDX3D_10] = V.m[IDX3D_12];
			V.m[IDX3D_20] = V.m[IDX3D_22];
			V.m[IDX3D_02] = V.m[IDX3D_01];
			V.m[IDX3D_12] = V.m[IDX3D_11];
			V.m[IDX3D_22] = V.m[IDX3D_21];
			V.m[IDX3D_01] = a0;
			V.m[IDX3D_11] = a1;
			V.m[IDX3D_21] = a2;
		} else {
			// switch 0/2
			a0 = U.m[IDX3D_00];
			a1 = U.m[IDX3D_10];
			a2 = U.m[IDX3D_20];
			U.m[IDX3D_00] = U.m[IDX3D_02];
			U.m[IDX3D_10] = U.m[IDX3D_12];
			U.m[IDX3D_20] = U.m[IDX3D_22];
			U.m[IDX3D_02] = a0;
			U.m[IDX3D_12] = a1;
			U.m[IDX3D_22] = a2;

			a0 = V.m[IDX3D_00];
			a1 = V.m[IDX3D_10];
			a2 = V.m[IDX3D_20];
			V.m[IDX3D_00] = V.m[IDX3D_02];
			V.m[IDX3D_10] = V.m[IDX3D_12];
			V.m[IDX3D_20] = V.m[IDX3D_22];
			V.m[IDX3D_02] = a0;
			V.m[IDX3D_12] = a1;
			V.m[IDX3D_22] = a2;
		}
	}

}

// arrange singular values from largest to smallest
void svd_sort(Matrix3d &Q, Matrix3d &U, Matrix3d &V) {

	int signdet = 1;
	double s0 = Q.m[IDX3D_00];
	double s1 = Q.m[IDX3D_11];
	double s2 = Q.m[IDX3D_22];

	// correct signs
	if (Q.m[IDX3D_00] < 0) {
		signdet = -signdet;
		s0 = -s0;
		V.m[IDX3D_00] = -V.m[IDX3D_00];
		V.m[IDX3D_10] = -V.m[IDX3D_10];
		V.m[IDX3D_20] = -V.m[IDX3D_20];
	}
	if (Q.m[IDX3D_11] < 0) {
		signdet = -signdet;
		s1 = -s1;
		V.m[IDX3D_01] = -V.m[IDX3D_01];
		V.m[IDX3D_11] = -V.m[IDX3D_11];
		V.m[IDX3D_21] = -V.m[IDX3D_21];
	}
	if (Q.m[IDX3D_22] < 0) {
		signdet = -signdet;
		s2 = -s2;
		V.m[IDX3D_02] = -V.m[IDX3D_02];
		V.m[IDX3D_12] = -V.m[IDX3D_12];
		V.m[IDX3D_22] = -V.m[IDX3D_22];
	}

	// Sort singular values
	if (s0 > s1) {
		if (s1 > s2) {
			// nothing
		} else if (s0 >= s2) {
			// switch s1/s2
			svd_rotate_UV(U, V, 0, 2, 1);
			double tmp = s1;
			s1 = s2;
			s2 = tmp;
		} else {
			// rotate 2, 0, 1
			svd_rotate_UV(U, V, 2, 0, 1);
			double tmp = s0;
			s0 = s2;
			s2 = s1;
			s1 = tmp;
		}
	} else {
		if (s0 >= s2) {
			// switch 0/1
			svd_rotate_UV(U, V, 1, 0, 2);
			double tmp = s1;
			s1 = s0;
			s0 = tmp;
		} else if (s1 >= s2) {
			svd_rotate_UV(U, V, 1, 2, 0);
			double tmp = s0;
			s0 = s1;
			s1 = s2;
			s2 = tmp;
		} else {
			svd_rotate_UV(U, V, 2, 1, 0);
			double tmp = s0;
			s0 = s2;
			s2 = tmp;
		}
	}

	Q.m[IDX3D_00] = s0;
	Q.m[IDX3D_11] = s1;
	Q.m[IDX3D_22] = s2;

}


bool svd3(const Matrix3d &A, Matrix3d &U, Vector3d &s, Matrix3d &V,
		size_t maxIters) {
	// copy A for operations
	Matrix3d B = A;
	double gr[2];	// to pass along rotations

	svd_bidiagonalize(B, U, V, gr);

	// inf norm of B, max row sum
	double bnorm = std::max(std::max(fabs(B.m[IDX3D_00])+fabs(B.m[IDX3D_01]),
			fabs(B.m[IDX3D_11])+fabs(B.m[IDX3D_12])), fabs(B.m[IDX3D_22]));


	if (bnorm == 0) {
		s.set(0,0,0);
		U.setIdentity();
		V.setIdentity();
		return true;
	}

	int iters = 0;
	maxIters = maxIters*3;	// account for dimension

	while (fabs(B.m[IDX3D_12]) > MAS_MATH_MACHINE_PRECISION *
			(fabs(B.m[IDX3D_11]) + fabs(B.m[IDX3D_22]))) {

		if (fabs(B.m[IDX3D_11]/bnorm) < MAS_MATH_MACHINE_PRECISION) {
			B.m[IDX3D_11] = 0;
		}

		if (B.m[IDX3D_11] != 0 && B.m[IDX3D_01] !=0 ) {
			if (fabs(B.m[IDX3D_00]/bnorm) < MAS_MATH_MACHINE_PRECISION) {
				B.m[IDX3D_00] = 0;
				svd_zero_3x3(B, U, gr);
			} else {
				svd_step_3x3(B, U, V, gr);
			}
		} else {
			if (B.m[IDX3D_11] == 0) {
				svd_zero_lower_2x2(B, U, gr);
			} else {
				svd_step_lower_2x2(B, U, V, gr);
			}
		}

		if (++iters > maxIters) {
			return false;
		}
	}
	B.m[IDX3D_12] = 0;

	while ( fabs(B.m[IDX3D_01]) > MAS_MATH_MACHINE_PRECISION *
			( fabs(B.m[IDX3D_00]) + fabs(B.m[IDX3D_11]))) {

		if (fabs(B.m[IDX3D_00]/bnorm) < MAS_MATH_MACHINE_PRECISION) {
			B.m[IDX3D_00] = 0;
			svd_zero_upper_2x2(B, U, gr);
		} else {
			svd_step_upper_2x2(B, U, V, gr);
		}

		if (++iters >= maxIters) {
			return false;
		}
	}
	svd_sort(B, U, V);

	s.x = B.m[IDX3D_00];
	s.y = B.m[IDX3D_11];
	s.z = B.m[IDX3D_22];
	return true;
}

}
}
