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

#ifndef _IMAGE_U32_H
#define _IMAGE_U32_H

#include <stdint.h>
#include "image_u8.h"
#include "image_f32.h"

typedef struct image_u32 image_u32_t;
struct image_u32 {
  int width, height;
  int stride;

  uint32_t *buf;
};

image_u32_t *image_u32_create(int width, int height);
image_u32_t *image_u32_create_from_pnm(const char *path);
image_u32_t *image_u32_create_from_u8(image_u8_t *in);
image_u32_t *image_u32_copy(image_u32_t *src);

// returns an array of 3 channels.
image_u8_t **image_u32_split_channels(image_u32_t *src);

void image_u32_clear(image_u32_t *im);
void image_u32_destroy(image_u32_t *im);
void image_u32_draw_line(image_u32_t *im, float x0, float y0, float x1,
                         float y1, uint32_t v, int width);
void image_u32_draw_circle(image_u32_t *im, float x0, float y0, float r,
                           uint32_t v);

int image_u32_write_pnm(image_u32_t *im, const char *path);

#endif
