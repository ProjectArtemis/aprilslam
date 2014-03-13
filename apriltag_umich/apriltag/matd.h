/*
 * This file is part of the AprilTag library.
 *
 * AprilTag is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * AprilTag is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef _MATD_H
#define _MATD_H

#define MATD_EPS 1e-8

typedef struct
{
    int nrows, ncols;

    // data in row-major order (index = rows*ncols + col)
    double *data;
} matd_t;

#define MAT_EL(m, row,col) (m)->data[((row)*(m)->ncols + (col))]

typedef struct
{
    // was the input matrix singular? When a near-zero pivot is found,
    // it is replaced with a value of MATD_EPS so that an approximate inverse
    // can be found. This flag is set to indicate that this has happened.
    int singular;

    int *piv; // permutation indices
    int pivsign;
    matd_t *lu; // combined L and U matrices, permuted.
} matd_lu_t;

//////////////////////////////////
// Creating and Destroying
matd_t *matd_create(int rows, int cols);
matd_t *matd_create_data(int rows, int cols, const double *data);

// NB: Scalars are different than 1x1 matrices (implementation note:
// they are encoded as 0x0 matrices). For example: for matrices A*B, A
// and B must both have specific dimensions. However, if A is a
// scalar, there are no restrictions on the size of B.
matd_t *matd_create_scalar(double v);

matd_t *matd_copy(const matd_t *m);
matd_t *matd_identity(int dim);
void matd_destroy(matd_t *m);


//////////////////////////////////
// Basic Operations

// fmt is printf specifier for each individual element. Each row will
// be printed on a separate newline.
void matd_print(const matd_t *m, const char *fmt);

matd_t *matd_add(const matd_t *a, const matd_t *b);
matd_t *matd_subtract(const matd_t *a, const matd_t *b);
matd_t *matd_scale(const matd_t *a, double s);

matd_t *matd_multiply(const matd_t *a, const matd_t *b);
matd_t *matd_transpose(const matd_t *a);

// matd_inverse attempts to compute an inverse. Fast (and less
// numerically stable) methods are used for 2x2, 3x3, and 4x4
// matrices. Nearly singular matrices are "patched up" by replacing
// near-zero pivots or determinants with MATD_EPS.
matd_t *matd_inverse(const matd_t *a);

matd_t *matd_solve(matd_t *A, matd_t *b);

double matd_det(const matd_t *a);

matd_t *matd_op(const char *expr, ...);

////////////////////////////////
// LU Decomposition

/** The LU decomposition exists for square, full-rank matrices. This
 * function computes the factorization; actually doing something with
 * the factorization requires another call. The caller should call
 * matd_lu_destroy on the return value. Note that for rank-deficient
 * matrices, MATD_EPS will be substituted for the zero pivot, which
 * allows the factorization to continue (albeit with poor numerical
 * stability).
 **/
matd_lu_t *matd_lu(const matd_t *a);
void matd_lu_destroy(matd_lu_t *mlu);

/** Compute the determinant of the matrix. **/
double matd_lu_det(const matd_lu_t *lu);

/** Retrieve the L factor. **/
matd_t *matd_lu_l(const matd_lu_t *lu);

/** Retrieve the U factor. **/
matd_t *matd_lu_u(const matd_lu_t *lu);

/** Compute an answer to Ax = b. **/
matd_t *matd_lu_solve(const matd_lu_t *mlu, const matd_t *b);

#endif
