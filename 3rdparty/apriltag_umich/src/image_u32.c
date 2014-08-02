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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "apriltag/image_u32.h"

// note: this alignment is in units of uint32s
#define ALIGNMENT 4

image_u32_t *image_u32_create(int width, int height) {
  image_u32_t *im = (image_u32_t *)calloc(1, sizeof(image_u32_t));

  im->width = width;
  im->height = height;
  im->stride = width;

  if ((im->stride % ALIGNMENT) != 0)
    im->stride += ALIGNMENT - (im->stride % ALIGNMENT);

  im->buf = (uint32_t *)calloc(1, sizeof(uint32_t) * im->height * im->stride);

  return im;
}

image_u32_t *image_u32_create_from_u8(image_u8_t *in) {
  image_u32_t *out = image_u32_create(in->width, in->height);

  for (int y = 0; y < in->height; y++) {
    for (int x = 0; x < in->width; x++) {
      uint32_t v = in->buf[in->stride * y + x];
      v = (v << 16) | (v << 8) | v;
      out->buf[out->stride * y + x] = v;
    }
  }

  return out;
}

image_u32_t *image_u32_copy(image_u32_t *src) {
  image_u32_t *im = (image_u32_t *)calloc(1, sizeof(image_u32_t));

  im->width = src->width;
  im->height = src->height;
  im->stride = src->stride;

  im->buf = (uint32_t *)calloc(1, im->height * im->stride);

  memcpy(im->buf, src->buf, sizeof(uint32_t) * im->height * im->stride);
  return im;
}

void image_u32_destroy(image_u32_t *im) {
  if (im == NULL) return;

  free(im->buf);
  free(im);
}

void image_u32_clear(image_u32_t *im) {
  memset(im->buf, 0, sizeof(uint32_t) * im->height * im->stride);
}

// will pack pixels as (r << 16) | (g << 8) | b
image_u32_t *image_u32_create_from_pnm(const char *path) {
  int width, height, format = -1;

  FILE *f = fopen(path, "rb");
  if (f == NULL) return NULL;

  image_u32_t *im = NULL;

  char tmp[1024];
  int nparams = 0;  // will be 3 when we're all done.
  int params[3];

  while (nparams < 3) {
    if (fgets(tmp, sizeof(tmp), f) == NULL) goto error;

    // skip comments
    if (tmp[0] == '#') continue;

    char *p = tmp;

    if (format == -1 && tmp[0] == 'P') {
      format = tmp[1] - '0';
      assert(format == 6 || format == 5);
      p = &tmp[2];
    }

    // pull integers out of this line until there are no more.
    while (nparams < 3 && *p != 0) {
      while (*p == ' ') p++;

      // encounter rubbish? (End of line?)
      if (*p < '0' || *p > '9') break;

      int acc = 0;
      while (*p >= '0' && *p <= '9') {
        acc = acc * 10 + *p - '0';
        p++;
      }

      params[nparams++] = acc;
      p++;
    }
  }

  width = params[0];
  height = params[1];
  assert(params[2] == 255);

  im = image_u32_create(width, height);

  if (format == 5) {
    int sz = width * height;
    uint8_t *gray = malloc(sz);

    if (sz != fread(gray, 1, sz, f)) goto error;
    fclose(f);

    for (int y = 0; y < im->height; y++) {
      for (int x = 0; x < im->width; x++) {
        int v = gray[y * width + x];

        im->buf[y * im->stride + x] = (v << 16) | (v << 8) | v;
      }
    }

    free(gray);
    return im;
  }

  if (format == 6) {
    int stride = width * 3;
    int sz = stride * height;

    uint8_t *rgb = malloc(sz);

    if (sz != fread(rgb, 1, sz, f)) goto error;

    fclose(f);

    for (int y = 0; y < im->height; y++) {
      for (int x = 0; x < im->width; x++) {
        int r = rgb[y * stride + 3 * x + 0];
        int g = rgb[y * stride + 3 * x + 1];
        int b = rgb[y * stride + 3 * x + 2];

        int v = (r << 16) | (b << 8) | g;

        im->buf[y * im->stride + x] = v;
      }
    }

    free(rgb);

    return im;
  }

error:
  fclose(f);

  if (im != NULL) image_u32_destroy(im);

  return NULL;
}

// only widths 1 and 3 supported
void image_u32_draw_line(image_u32_t *im, float x0, float y0, float x1,
                         float y1, uint32_t v, int width) {
  double dist = sqrtf((y1 - y0) * (y1 - y0) + (x1 - x0) * (x1 - x0));
  double delta = 0.5 / dist;

  // terrible line drawing code
  for (float f = 0; f <= 1; f += delta) {
    int x = ((int)(x0 * f + x1 * (1 - f)));
    int y = ((int)(y0 * f + y1 * (1 - f)));

    if (x < 0 || y < 0 || x + 1 >= im->width || y + 1 >= im->height) continue;

    int idx = y * im->stride + x;
    im->buf[idx] = v;
    if (width > 1) {
      im->buf[idx + 1] = v;
      im->buf[idx + im->stride] = v;
      im->buf[idx + 1 + im->stride] = v;
    }
  }
}

void image_u32_draw_circle(image_u32_t *im, float x0, float y0, float r,
                           uint32_t v) {
  r = r * r;

  for (int y = y0 - r; y <= y0 + r; y++) {
    for (int x = x0 - r; x <= x0 + r; x++) {
      float d = (x - x0) * (x - x0) + (y - y0) * (y - y0);
      if (d > r) continue;

      int idx = y * im->stride + x;
      im->buf[idx] = v;
    }
  }
}

int image_u32_write_pnm(image_u32_t *im, const char *path) {
  FILE *f = fopen(path, "wb");
  int res = 0;

  if (f == NULL) {
    res = -1;
    goto finish;
  }

  fprintf(f, "P6\n%d %d\n255\n", im->width, im->height);

  for (int y = 0; y < im->height; y++) {
    uint8_t buf[im->width * 3];

    for (int x = 0; x < im->width; x++) {
      int rgb = im->buf[y * im->stride + x];

      buf[3 * x + 0] = (rgb >> 16);
      buf[3 * x + 1] = (rgb >> 8) & 0xff;
      buf[3 * x + 2] = rgb & 0xff;
    }

    if (sizeof(buf) != fwrite(buf, 1, sizeof(buf), f)) {
      res = -2;
      goto finish;
    }
  }

finish:
  if (f != NULL) fclose(f);

  return res;
}

image_u8_t **image_u32_split_channels(image_u32_t *in) {
  int nchannels = 3;
  image_u8_t **outs = calloc(nchannels, sizeof(image_u8_t *));

  for (int i = 0; i < nchannels; i++) {
    outs[i] = image_u8_create(in->width, in->height);
  }

  for (int y = 0; y < in->height; y++) {
    for (int x = 0; x < in->width; x++) {
      int rgb = in->buf[y * in->stride + x];
      int r = (rgb >> 16) & 0xff;
      int g = (rgb >> 8) & 0xff;
      int b = rgb & 0xff;

      outs[0]->buf[y * outs[0]->stride + x] = r;
      outs[1]->buf[y * outs[1]->stride + x] = g;
      outs[2]->buf[y * outs[2]->stride + x] = b;
    }
  }

  return outs;
}
