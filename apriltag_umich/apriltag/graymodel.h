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

#ifndef _GRAYMODEL_H
#define _GRAYMODEL_H

#include "matd.h"

typedef struct graymodel graymodel_t;
struct graymodel {
  matd_t *A;
  matd_t *b;
  matd_t *x;

  int n;
};

graymodel_t *graymodel_create();

void graymodel_add_observation(graymodel_t *gm, double x, double y, double v);

void graymodel_solve(graymodel_t *gm);

double graymodel_interpolate(graymodel_t *gm, double x, double y);

void graymodel_destroy(graymodel_t *gm);

#endif
