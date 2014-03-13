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
#include "image_f32.h"

image_f32_t *image_f32_create(int width, int height)
{
    image_f32_t *fim = (image_f32_t*) calloc(1, sizeof(image_f32_t));

    fim->width = width;
    fim->height = height;
    fim->stride = width; // XXX do better alignment

    fim->buf = calloc(fim->height * fim->stride, sizeof(float));

    return fim;
}

// scales by 1/255u
image_f32_t *image_f32_create_from_u8(image_u8_t *im)
{
    image_f32_t *fim = image_f32_create(im->width, im->height);

    for (int y = 0; y < fim->height; y++)
        for (int x = 0; x < fim->width; x++)
            fim->buf[y*fim->stride + x] = im->buf[y*im->stride + x] / 255.0f;

    return fim;
}

void image_f32_destroy(image_f32_t *im)
{
    free(im->buf);
    free(im);
}
