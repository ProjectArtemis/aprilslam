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

#ifndef _LINE_FIT_H
#define _LINE_FIT_H

#include "common.h"

#include <string.h>

#define LINE_FIT_TYPE float

typedef struct line_fit line_fit_t;
struct line_fit
{
    LINE_FIT_TYPE mXX, mYY, mXY, mX, mY; // 1st and 2nd moments
    LINE_FIT_TYPE n; // total weight of points
    uint16_t x0, x1, y0, y1; // bounding box
};

static inline void line_fit_init(line_fit_t *lf)
{
    memset(lf, 0, sizeof(line_fit_t));
    lf->x0 =  65535;
    lf->y0 =  65535;

    // since we only use pixel values, 0 is small enough for x1, y1.
}

static inline void line_fit_init_nozero_update(line_fit_t *lf, LINE_FIT_TYPE x, LINE_FIT_TYPE y, LINE_FIT_TYPE w)
{
    lf->mX = w*x;
    lf->mY = w*y;
    lf->mXX = w*x*x;
    lf->mXY = w*x*y;
    lf->mYY = w*y*y;

    lf->n = w;

    lf->x0 = x;
    lf->x1 = x;
    lf->y0 = y;
    lf->y1 = y;
}

static inline void line_fit_init_nozero_update_w1(line_fit_t *lf, uint32_t x, uint32_t y)
{
    lf->mX += x;
    lf->mY += y;
    lf->mXX += x*x;
    lf->mXY += x*y;
    lf->mYY += y*y;

    lf->n++;

    lf->x0 = x;
    lf->x1 = x;
    lf->y0 = y;
    lf->y1 = y;
}

// make this line fit object include all the data that was included in the other one.
static inline void line_fit_combine(line_fit_t *lf, line_fit_t *other)
{
    lf->mX += other->mX;
    lf->mY += other->mY;
    lf->mXX += other->mXX;
    lf->mXY += other->mXY;
    lf->mYY += other->mYY;

    lf->n += other->n;

    lf->x0 = imin(other->x0, lf->x0);
    lf->x1 = imax(other->x1, lf->x1);
    lf->y0 = imin(other->y0, lf->y0);
    lf->y1 = imax(other->y1, lf->y1);
}

static inline void line_fit_update(line_fit_t *lf, uint32_t x, uint32_t y, uint32_t w)
{
    lf->mX += w*x;
    lf->mY += w*y;
    lf->mXX += w*x*x;
    lf->mXY += w*x*y;
    lf->mYY += w*y*y;

    lf->n += w;

    lf->x0 = imin(x, lf->x0);
    lf->x1 = imax(x, lf->x1);
    lf->y0 = imin(y, lf->y0);
    lf->y1 = imax(y, lf->y1);
}

static inline void line_fit_update_w1(line_fit_t *lf, uint32_t x, uint32_t y)
{
    lf->mX += x;
    lf->mY += y;
    lf->mXX += x*x;
    lf->mXY += x*y;
    lf->mYY += y*y;

    lf->n += 1;

    lf->x0 = imin(x, lf->x0);
    lf->x1 = imax(x, lf->x1);
    lf->y0 = imin(y, lf->y0);
    lf->y1 = imax(y, lf->y1);
}

// writes a point that lies on the line in p0, the unit vector in u.
static inline void line_fit_compute(line_fit_t *lf, float *p0, float *u)
{
    float Ex = (float) (lf->mX / lf->n);
    float Ey = (float) (lf->mY / lf->n);
    float Cxx = (float) (lf->mXX / lf->n - Ex*Ex);
    float Cxy = (float) (lf->mXY / lf->n - Ex*Ey);
    float Cyy = (float) (lf->mYY / lf->n - Ey*Ey);

    float phi = 0.5f*atan2(-2*Cxy, (Cyy - Cxx));

    // the direction of the line is -sin(phi), cos(phi), and it goes
    // through (Ex, Ey)
    p0[0] = Ex;
    p0[1] = Ey;
    u[0] = -sin(phi);
    u[1] = cos(phi);
}

#endif
