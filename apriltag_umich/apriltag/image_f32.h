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

#ifndef _IMAGE_F32_H
#define _IMAGE_F32_H

#include <stdint.h>

typedef struct image_f32 image_f32_t;

struct image_f32 {
  int width, height;
  int stride;  // in units of floats

  float *buf;  // indexed as buf[y*stride + x]
};

#include "image_u8.h"

image_f32_t *image_f32_create(int width, int height);

image_f32_t *image_f32_create_from_u8(image_u8_t *im);

void image_f32_destroy(image_f32_t *im);

#endif
