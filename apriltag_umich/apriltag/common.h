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

#ifndef _COMMON_H
#define _COMMON_H

#include <math.h>

static inline int iclamp(int a, int min, int max) {
  if (a < min)
    return min;
  if (a > max)
    return max;
  return a;
}

static inline double dclamp(double a, double min, double max) {
  if (a < min)
    return min;
  if (a > max)
    return max;
  return a;
}

static inline int isq(int v) {
  return v*v;
}

static inline float sq(float v) {
  return v*v;
}

static inline int imin(int a, int b) {
  return a < b ? a : b;
}

static inline int imax(int a, int b) {
  return a < b ? b : a;
}

static inline float mod2pi(float v) {
  // XXX Dumb version
  while (v < -M_PI)
    v += 2*M_PI;

  while (v >= M_PI)
    v -= 2*M_PI;

  return v;
}

#endif


