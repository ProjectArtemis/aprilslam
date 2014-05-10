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

#ifndef _IMAGE_U8_H
#define _IMAGE_U8_H

#include <stdint.h>

typedef struct image_u8 image_u8_t;
struct image_u8 {
  int width, height;
  int stride;

  uint8_t *buf;
};

#include "image_f32.h"

image_u8_t *image_u8_create(int width, int height);
image_u8_t *image_u8_create_from_gray(int width, int height, uint8_t *gray);
image_u8_t *image_u8_create_from_rgb3(int width, int height, uint8_t *rgb, int stride);
image_u8_t *image_u8_create_from_f32(image_f32_t *fim);
image_u8_t *image_u8_create_from_pnm(const char *path);

image_u8_t *image_u8_copy(const image_u8_t *src);
void image_u8_draw_line(image_u8_t *im, float x0, float y0, float x1, float y1, int v, int width);
void image_u8_draw_circle(image_u8_t *im, float x0, float y0, float r, int v);

void image_u8_clear(image_u8_t *im);
void image_u8_destroy(image_u8_t *im);

int image_u8_write_pgm(const image_u8_t *im, const char *path);

void image_u8_darken(image_u8_t *im);
void image_u8_gaussian_blur(image_u8_t *im, double sigma, int k);

// NB: only 2 and 3 supported.
image_u8_t *image_u8_decimate(image_u8_t *im, int factor);

// rotate the image by 'rad' radians. (Rotated in the "intuitive
// sense", i.e., if Y were up. When input values are unavailable, the
// value 'pad' is inserted instead. The geometric center of the output
// image corresponds to the geometric center of the input image.
image_u8_t *image_u8_rotate(const image_u8_t *in, double rad, uint8_t pad);

#endif
