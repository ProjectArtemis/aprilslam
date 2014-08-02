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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "assert.h"
#include "apriltag/matd.h"

// a matd_t with rows=0 cols=0 is a SCALAR.

// to ease creating mati, matf, etc. in the future.
#define TYPE double

matd_t *matd_create(int rows, int cols) {
  matd_t *m = calloc(1, sizeof(matd_t));
  m->nrows = rows;
  m->ncols = cols;
  m->data = calloc(m->nrows * m->ncols, sizeof(TYPE));

  return m;
}

matd_t *matd_identity(int dim) {
  matd_t *m = matd_create(dim, dim);
  for (int i = 0; i < dim; i++) MAT_EL(m, i, i) = 1;
  return m;
}

matd_t *matd_create_scalar(double v) {
  matd_t *m = calloc(1, sizeof(matd_t));
  m->nrows = 0;
  m->ncols = 0;
  m->data = calloc(1, sizeof(TYPE));
  m->data[0] = v;

  return m;
}

matd_t *matd_create_data(int rows, int cols, const TYPE *data) {
  matd_t *m = matd_create(rows, cols);
  for (int i = 0; i < rows * cols; i++) m->data[i] = data[i];

  return m;
}

matd_t *matd_copy(const matd_t *m) {
  matd_t *x = matd_create(m->nrows, m->ncols);
  memcpy(x->data, m->data, sizeof(TYPE) * m->ncols * m->nrows);
  return x;
}

void matd_print(const matd_t *m, const char *fmt) {
  if (m->nrows == 0) {
    printf(fmt, MAT_EL(m, 0, 0));
  } else {
    for (int i = 0; i < m->nrows; i++) {
      for (int j = 0; j < m->ncols; j++) {
        printf(fmt, MAT_EL(m, i, j));
      }
      printf("\n");
    }
  }
}

void matd_destroy(matd_t *m) {
  free(m->data);
  free(m);
}

matd_t *matd_multiply(const matd_t *a, const matd_t *b) {
  if (a->nrows == 0) return matd_scale(b, a->data[0]);
  if (b->nrows == 0) return matd_scale(a, b->data[0]);

  assert(a->ncols == b->nrows);
  matd_t *m = matd_create(a->nrows, b->ncols);

  for (int i = 0; i < m->nrows; i++) {
    for (int j = 0; j < m->ncols; j++) {
      TYPE acc = 0;
      for (int k = 0; k < a->ncols; k++) {
        acc += MAT_EL(a, i, k) * MAT_EL(b, k, j);
      }
      MAT_EL(m, i, j) = acc;
    }
  }

  return m;
}

matd_t *matd_scale(const matd_t *a, TYPE s) {
  if (a->nrows == 0) return matd_create_scalar(a->data[0] * s);

  matd_t *m = matd_create(a->nrows, a->ncols);

  for (int i = 0; i < m->nrows; i++) {
    for (int j = 0; j < m->ncols; j++) {
      MAT_EL(m, i, j) = s * MAT_EL(a, i, j);
    }
  }

  return m;
}

matd_t *matd_add(const matd_t *a, const matd_t *b) {
  assert(a->nrows == b->nrows);
  assert(a->ncols == b->ncols);

  if (a->nrows == 0) return matd_create_scalar(a->data[0] + b->data[0]);

  matd_t *m = matd_create(a->nrows, a->ncols);

  for (int i = 0; i < m->nrows; i++) {
    for (int j = 0; j < m->ncols; j++) {
      MAT_EL(m, i, j) = MAT_EL(a, i, j) + MAT_EL(b, i, j);
    }
  }

  return m;
}

void matd_add_inplace(matd_t *a, const matd_t *b) {
  assert(a->nrows == b->nrows);
  assert(a->ncols == b->ncols);

  if (a->nrows == 0) {
    a->data[0] += b->data[0];
    return;
  }

  for (int i = 0; i < a->nrows; i++) {
    for (int j = 0; j < a->ncols; j++) {
      MAT_EL(a, i, j) += MAT_EL(b, i, j);
    }
  }
}

matd_t *matd_subtract(const matd_t *a, const matd_t *b) {
  assert(a->nrows == b->nrows);
  assert(a->ncols == b->ncols);

  if (a->nrows == 0) return matd_create_scalar(a->data[0] - b->data[0]);

  matd_t *m = matd_create(a->nrows, a->ncols);

  for (int i = 0; i < m->nrows; i++) {
    for (int j = 0; j < m->ncols; j++) {
      MAT_EL(m, i, j) = MAT_EL(a, i, j) - MAT_EL(b, i, j);
    }
  }

  return m;
}

matd_t *matd_transpose(const matd_t *a) {
  if (a->nrows == 0) return matd_create_scalar(a->data[0]);

  matd_t *m = matd_create(a->ncols, a->nrows);

  for (int i = 0; i < a->nrows; i++) {
    for (int j = 0; j < a->ncols; j++) {
      MAT_EL(m, j, i) = MAT_EL(a, i, j);
    }
  }
  return m;
}

static double make_non_zero(double v) {
  if (fabs(v) < MATD_EPS) {
    if (v < 0) return -MATD_EPS;
    return MATD_EPS;
  }

  return v;
}

// XXX How to handle singular matrices?
matd_t *matd_inverse(const matd_t *x) {
  assert(x->nrows == x->ncols);

  switch (x->nrows) {
    case 0:
      // scalar
      return matd_create_scalar(1.0 / make_non_zero(x->data[0]));

    case 1: {
      // a 1x1 matrix
      matd_t *m = matd_create(x->nrows, x->nrows);
      MAT_EL(m, 0, 0) = 1.0 / make_non_zero(x->data[0]);
      return m;
    }

    case 2: {
      matd_t *m = matd_create(x->nrows, x->nrows);

      double invdet = 1.0 / make_non_zero(MAT_EL(x, 0, 0) * MAT_EL(x, 1, 1) -
                                          MAT_EL(x, 0, 1) * MAT_EL(x, 1, 0));
      MAT_EL(m, 0, 0) = MAT_EL(x, 1, 1) * invdet;
      MAT_EL(m, 0, 1) = -MAT_EL(x, 0, 1) * invdet;
      MAT_EL(m, 1, 0) = -MAT_EL(x, 1, 0) * invdet;
      MAT_EL(m, 1, 1) = MAT_EL(x, 0, 0) * invdet;

      return m;
    }

    case 3: {
      matd_t *m = matd_create(x->nrows, x->nrows);

      double a = MAT_EL(x, 0, 0), b = MAT_EL(x, 0, 1), c = MAT_EL(x, 0, 2);
      double d = MAT_EL(x, 1, 0), e = MAT_EL(x, 1, 1), f = MAT_EL(x, 1, 2);
      double g = MAT_EL(x, 2, 0), h = MAT_EL(x, 2, 1), i = MAT_EL(x, 2, 2);

      double det = make_non_zero(a * e * i - a * f * h - d * b * i + d * c * h +
                                 g * b * f - g * c * e);
      det = 1.0 / det;

      MAT_EL(m, 0, 0) = det * (e * i - f * h);
      MAT_EL(m, 0, 1) = det * (-b * i + c * h);
      MAT_EL(m, 0, 2) = det * (b * f - c * e);
      MAT_EL(m, 1, 0) = det * (-d * i + f * g);
      MAT_EL(m, 1, 1) = det * (a * i - c * g);
      MAT_EL(m, 1, 2) = det * (-a * f + c * d);
      MAT_EL(m, 2, 0) = det * (d * h - e * g);
      MAT_EL(m, 2, 1) = det * (-a * h + b * g);
      MAT_EL(m, 2, 2) = det * (a * e - b * d);
      return m;
    }
  }

  matd_lu_t *mlu = matd_lu(x);
  matd_t *ident = matd_identity(x->nrows);
  matd_t *inv = matd_lu_solve(mlu, ident);

  matd_destroy(ident);
  matd_lu_destroy(mlu);

  return inv;
}

matd_t *matd_solve(matd_t *A, matd_t *b) {
  matd_lu_t *mlu = matd_lu(A);
  matd_t *x = matd_lu_solve(mlu, b);

  matd_lu_destroy(mlu);
  return x;
}

double matd_det(const matd_t *x) {
  // if rank deficient, this is easy.
  if (x->nrows != x->ncols) return 0;

  switch (x->nrows) {
    case 0:
      // scalar
      assert(0);  // not defined.
      return x->data[0];

    case 1: {
      // a 1x1 matrix
      return MAT_EL(x, 0, 0);
    }

    case 2: {
      double det = (MAT_EL(x, 0, 0) * MAT_EL(x, 1, 1) -
                    MAT_EL(x, 0, 1) * MAT_EL(x, 1, 0));
      return det;
    }

    case 3: {
      double a = MAT_EL(x, 0, 0), b = MAT_EL(x, 0, 1), c = MAT_EL(x, 0, 2);
      double d = MAT_EL(x, 1, 0), e = MAT_EL(x, 1, 1), f = MAT_EL(x, 1, 2);
      double g = MAT_EL(x, 2, 0), h = MAT_EL(x, 2, 1), i = MAT_EL(x, 2, 2);

      double det = (a * e * i - a * f * h - d * b * i + d * c * h + g * b * f -
                    g * c * e);
      return det;
    }

    default: {
      matd_lu_t *mlu = matd_lu(x);
      double det = matd_lu_det(mlu);
      matd_lu_destroy(mlu);

      return det;
    }
  }
}

// TODO Optimization: Some operations we could perform in-place,
// saving some memory allocation work. E.g., ADD, SUBTRACT. Just need to make
// sure
// that we don't do an in-place modification on a matrix that was an input
// argument.

// handle right-associative operators, greedily consuming them. These
// include transpose and inverse. This is called by the main recursion
// method.
static inline matd_t *matd_op_gobble_right(const char *expr, int *pos,
                                           matd_t *acc, matd_t **garb,
                                           int *garbpos, int *isarg) {
  while (expr[*pos] != 0) {

    switch (expr[*pos]) {

      case '\'': {
        assert(acc != NULL);
        matd_t *res = matd_transpose(acc);
        garb[*garbpos] = res;
        (*garbpos)++;
        acc = res;
        *isarg = 0;

        (*pos)++;
        break;
      }

      // handle inverse ^-1. No other exponents are allowed.
      case '^': {
        assert(acc != NULL);
        assert(expr[*pos + 1] == '-');
        assert(expr[*pos + 2] == '1');

        matd_t *res = matd_inverse(acc);
        garb[*garbpos] = res;
        (*garbpos)++;
        acc = res;
        *isarg = 0;

        (*pos) += 3;
        break;
      }

      default:
        return acc;
    }
  }

  return acc;
}

matd_lu_t *matd_lu(const matd_t *a) {
  int *piv = calloc(a->nrows, sizeof(int));
  int pivsign = 1;
  matd_t *lu = matd_copy(a);

  matd_lu_t *mlu = calloc(1, sizeof(matd_lu_t));

  for (int i = 0; i < a->nrows; i++) piv[i] = i;

  for (int j = 0; j < a->ncols; j++) {
    for (int i = 0; i < a->nrows; i++) {
      int kmax = i < j ? i : j;  // min(i,j)

      // compute dot product of row i with column j (up through element kmax)
      double acc = 0;
      for (int k = 0; k < kmax; k++) acc += MAT_EL(lu, i, k) * MAT_EL(lu, k, j);

      MAT_EL(lu, i, j) -= acc;
    }

    // find pivot and exchange if necessary.
    int p = j;
    if (1) {
      for (int i = j + 1; i < lu->nrows; i++) {
        if (fabs(MAT_EL(lu, i, j)) > fabs(MAT_EL(lu, p, j))) {
          p = i;
        }
      }
    }

    // swap rows p and j?
    if (p != j) {
      TYPE tmp[lu->ncols];
      memcpy(tmp, &MAT_EL(lu, p, 0), sizeof(TYPE) * lu->ncols);
      memcpy(&MAT_EL(lu, p, 0), &MAT_EL(lu, j, 0), sizeof(TYPE) * lu->ncols);
      memcpy(&MAT_EL(lu, j, 0), tmp, sizeof(TYPE) * lu->ncols);
      int k = piv[p];
      piv[p] = piv[j];
      piv[j] = k;
      pivsign = -pivsign;
    }

    double LUjj = MAT_EL(lu, j, j);

    // If our pivot is very small (which means the matrix is
    // singular or nearly singular), replace with a new pivot of the
    // right sign.
    if (fabs(LUjj) < MATD_EPS) {
      if (LUjj < 0)
        LUjj = -MATD_EPS;
      else
        LUjj = MATD_EPS;

      MAT_EL(lu, j, j) = LUjj;

      mlu->singular = 1;
    }

    if (j < lu->ncols && j < lu->nrows && LUjj != 0) {
      LUjj = 1.0 / LUjj;
      for (int i = j + 1; i < lu->nrows; i++) MAT_EL(lu, i, j) *= LUjj;
    }
  }

  mlu->lu = lu;
  mlu->piv = piv;
  mlu->pivsign = pivsign;

  return mlu;
}

void matd_lu_destroy(matd_lu_t *mlu) {
  matd_destroy(mlu->lu);
  free(mlu->piv);
  free(mlu);
}

double matd_lu_det(const matd_lu_t *mlu) {
  matd_t *lu = mlu->lu;
  double det = mlu->pivsign;

  if (lu->nrows == lu->ncols) {
    for (int i = 0; i < lu->ncols; i++) det *= MAT_EL(lu, i, i);
  }

  return det;
}

matd_t *matd_lu_l(const matd_lu_t *mlu) {
  matd_t *lu = mlu->lu;

  matd_t *L = matd_create(lu->nrows, lu->ncols);
  for (int i = 0; i < lu->nrows; i++) {
    for (int j = 0; j < lu->ncols; j++) {
      if (i > j)
        MAT_EL(L, i, j) = MAT_EL(lu, i, j);
      else if (i == j)
        MAT_EL(L, i, j) = 1;
    }
  }

  return L;
}

matd_t *matd_lu_u(const matd_lu_t *mlu) {
  matd_t *lu = mlu->lu;

  matd_t *U = matd_create(lu->ncols, lu->ncols);
  for (int i = 0; i < lu->ncols; i++) {
    for (int j = 0; j < lu->ncols; j++) {
      if (i <= j) MAT_EL(U, i, j) = MAT_EL(lu, i, j);
    }
  }

  return U;
}

matd_t *matd_lu_solve(const matd_lu_t *mlu, const matd_t *b) {
  matd_t *x = matd_copy(b);

  // permute right hand side
  for (int i = 0; i < mlu->lu->nrows; i++)
    memcpy(&MAT_EL(x, mlu->piv[i], 0), &MAT_EL(b, i, 0),
           sizeof(TYPE) * b->ncols);

  // solve Ly = b
  for (int k = 0; k < mlu->lu->nrows; k++) {
    for (int i = k + 1; i < mlu->lu->nrows; i++) {
      double LUik = -MAT_EL(mlu->lu, i, k);
      for (int t = 0; t < b->ncols; t++)
        MAT_EL(x, i, t) += MAT_EL(x, k, t) * LUik;
    }
  }

  // solve Ux = y
  for (int k = mlu->lu->ncols - 1; k >= 0; k--) {
    double LUkk = 1.0 / MAT_EL(mlu->lu, k, k);
    for (int t = 0; t < b->ncols; t++) MAT_EL(x, k, t) *= LUkk;

    for (int i = 0; i < k; i++) {
      double LUik = -MAT_EL(mlu->lu, i, k);
      for (int t = 0; t < b->ncols; t++)
        MAT_EL(x, i, t) += MAT_EL(x, k, t) * LUik;
    }
  }

  return x;
}

// @garb, garbpos  A list of every matrix allocated during evaluation... used to
// assist cleanup.
// @isarg: The returned value is one of the original arguments. Set to 0 if
// we're returning a newly-allocated.
// @oneterm: we should return at the end of this term (i.e., stop at a PLUS,
// MINUS, LPAREN).
static matd_t *matd_op_recurse(const char *expr, int *pos, matd_t *acc,
                               matd_t **args, int *argpos, matd_t **garb,
                               int *garbpos, int *isarg, int oneterm) {
  while (expr[*pos] != 0) {

    switch (expr[*pos]) {

      case '(': {
        if (oneterm && acc != NULL) return acc;
        (*pos)++;
        matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb,
                                      garbpos, isarg, 0);
        rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos, isarg);

        if (acc == NULL) {
          acc = rhs;
          // isarg is unchanged--- whatever the recursive call did above.
        } else {
          matd_t *res = matd_multiply(acc, rhs);
          garb[*garbpos] = res;
          (*garbpos)++;
          acc = res;
          *isarg = 0;
        }

        break;
      }

      case ')': {
        if (oneterm) return acc;

        (*pos)++;
        return acc;
      }

      case '*':
        (*pos)++;

        matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb,
                                      garbpos, isarg, 1);
        rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos, isarg);

        if (acc == NULL) {
          acc = rhs;
        } else {
          matd_t *res = matd_multiply(acc, rhs);
          garb[*garbpos] = res;
          (*garbpos)++;
          acc = res;
          *isarg = 0;
        }

        break;

      case 'M': {
        matd_t *rhs = args[*argpos];
        (*pos)++;
        (*argpos)++;

        rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos, isarg);

        if (acc == NULL) {
          acc = rhs;
          *isarg = 1;
        } else {
          matd_t *res = matd_multiply(acc, rhs);
          garb[*garbpos] = res;
          (*garbpos)++;
          acc = res;
          *isarg = 0;
        }

        break;
      }

      /*
         case 'D': {
         int rows = expr[*pos+1]-'0';
         int cols = expr[*pos+2]-'0';

         matd_t *rhs = matd_create(rows, cols);

         break;
         }
         */
      // a constant (SCALAR) defined inline. Treat just like M, creating a
      // matd_t on the fly.
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
      case '.': {
        const char *start = &expr[*pos];
        char *end;
        double s = strtod(start, &end);
        (*pos) += (end - start);
        matd_t *rhs = matd_create_scalar(s);
        garb[*garbpos] = rhs;
        (*garbpos)++;

        rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos, isarg);

        if (acc == NULL) {
          acc = rhs;
        } else {
          matd_t *res = matd_multiply(acc, rhs);
          garb[*garbpos] = res;
          (*garbpos)++;
          acc = res;
        }

        *isarg = 0;
        break;
      }

      case '+': {
        if (oneterm && acc != NULL) return acc;

        // don't support unary plus
        assert(acc != NULL);
        (*pos)++;
        matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb,
                                      garbpos, isarg, 1);
        rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos, isarg);

        // This is an example of an in-place optimization.
        // original code:
        // matd_t *res = matd_add(acc, rhs);
        matd_t *res;
        if (!isarg) {
          matd_add_inplace(rhs, acc);
          res = rhs;
        } else {
          res = matd_add(acc, rhs);
        }

        garb[*garbpos] = res;
        (*garbpos)++;
        acc = res;
        *isarg = 0;
        break;
      }

      case '-': {
        if (oneterm && acc != NULL) return acc;

        if (acc == NULL) {
          // unary minus
          (*pos)++;
          matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb,
                                        garbpos, isarg, 1);
          rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos, isarg);

          matd_t *res = matd_scale(rhs, -1);
          garb[*garbpos] = res;
          (*garbpos)++;
          acc = res;
          *isarg = 0;
        } else {
          // subtract
          (*pos)++;
          matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb,
                                        garbpos, isarg, 1);
          rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos, isarg);

          matd_t *res = matd_subtract(acc, rhs);
          garb[*garbpos] = res;
          (*garbpos)++;
          acc = res;
          *isarg = 0;
        }
        break;
      }

      case ' ':
        // nothing to do. spaces are meaningless.
        (*pos)++;
        break;

      default:
        (*pos)++;
        printf("Unknown character '%c'\n", expr[*pos]);
        break;
    }
  }
  return acc;
}

// always returns a new matrix.
matd_t *matd_op(const char *expr, ...) {
  int nargs = 0;
  int exprlen = 0;
  for (const char *p = expr; *p != 0; p++) {
    if (*p == 'M') nargs++;
    exprlen++;
  }

  va_list ap;
  va_start(ap, expr);

  matd_t *args[nargs];
  for (int i = 0; i < nargs; i++) {
    args[i] = va_arg(ap, matd_t *);
    // XXX: sanity check argument; emit warning/error if args[i]
    // doesn't look like a matd_t*.
  }

  va_end(ap);

  int pos = 0;
  int argpos = 0;
  int garbpos = 0;
  int isarg = 0;

  matd_t *garb[exprlen];  // can't create more than 1 new result per character

  matd_t *res = matd_op_recurse(expr, &pos, NULL, args, &argpos, garb, &garbpos,
                                &isarg, 0);

  for (int i = 0; i < garbpos; i++) {
    if (garb[i] != res) matd_destroy(garb[i]);
  }

  if (isarg) res = matd_copy(res);

  return res;
}
