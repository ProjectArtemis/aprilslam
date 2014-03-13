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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

#include "graymodel.h"
#include "matd.h"

graymodel_t *graymodel_create()
{
  graymodel_t *gm = calloc(1, sizeof(graymodel_t));
  gm->A = matd_create(4, 4);
  gm->b = matd_create(4, 1);
  return gm;
}

void graymodel_add_observation(graymodel_t *gm, double x, double y, double v)
{
  double xy = x * y;

  // update only upper-right elements. A'A is symmetric,
  // we'll fill the other elements in later.
  MAT_EL(gm->A, 0, 0) += x * x;
  MAT_EL(gm->A, 0, 1) += x * y;
  MAT_EL(gm->A, 0, 2) += x * xy;
  MAT_EL(gm->A, 0, 3) += x;
  MAT_EL(gm->A, 1, 1) += y * y;
  MAT_EL(gm->A, 1, 2) += y * xy;
  MAT_EL(gm->A, 1, 3) += y;
  MAT_EL(gm->A, 2, 2) += xy * xy;
  MAT_EL(gm->A, 2, 3) += xy;
  MAT_EL(gm->A, 3, 3) += 1;

  MAT_EL(gm->b, 0, 0) += x * v;
  MAT_EL(gm->b, 1, 0) += y * v;
  MAT_EL(gm->b, 2, 0) += xy * v;
  MAT_EL(gm->b, 3, 0) += v;

  gm->n++;
}

void graymodel_solve(graymodel_t *gm)
{
  if (gm->n > 6)
  {
    // make symmetric
    for (int i = 0; i < 4; i++)
      for (int j = i + 1; j < 4; j++)
        MAT_EL(gm->A, j, i) = MAT_EL(gm->A, i, j);

    gm->x = matd_solve(gm->A, gm->b);
  }
  else
  {
    // use flat model
    gm->x = matd_create(4, 1);
    MAT_EL(gm->x, 3, 0) = MAT_EL(gm->b, 3, 0) / gm->n;
  }
}

double graymodel_interpolate(graymodel_t *gm, double x, double y)
{
  assert(gm->x != NULL);
  return MAT_EL(gm->x, 0, 0) * x + MAT_EL(gm->x, 1, 0) * y +
         MAT_EL(gm->x, 2, 0) * x * y + MAT_EL(gm->x, 3, 0);
}

void graymodel_destroy(graymodel_t *gm)
{
  matd_destroy(gm->A);
  matd_destroy(gm->b);
  if (gm->x != NULL)
    matd_destroy(gm->x);
  free(gm);
}

