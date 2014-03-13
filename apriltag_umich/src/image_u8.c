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
#include "image_u8.h"
#include "common.h"

#define ALIGNMENT 64

image_u8_t *image_u8_create(int width, int height)
{
  image_u8_t *im = (image_u8_t*) calloc(1, sizeof(image_u8_t));

  im->width = width;
  im->height = height;
  im->stride = width;

  if ((im->stride % ALIGNMENT) != 0)
    im->stride += ALIGNMENT - (im->stride % ALIGNMENT);

  im->buf = (uint8_t*) calloc(1, im->height * im->stride);

  return im;
}

image_u8_t *image_u8_copy(const image_u8_t *src)
{
  image_u8_t *im = (image_u8_t*) calloc(1, sizeof(image_u8_t));

  im->width = src->width;
  im->height = src->height;
  im->stride = src->stride;

  im->buf = (uint8_t*) calloc(1, im->height * im->stride);

  memcpy(im->buf, src->buf, im->height * im->stride);
  return im;
}

void image_u8_destroy(image_u8_t *im)
{
  if (im == NULL)
    return;

  free(im->buf);
  free(im);
}

void image_u8_clear(image_u8_t *im)
{
  memset(im->buf, 0, im->height * im->stride);
}

image_u8_t *image_u8_create_from_pnm(const char *path)
{
  int width, height, format = -1;

  FILE *f = fopen(path, "rb");
  if (f == NULL)
    return NULL;

  image_u8_t *im = NULL;

  char tmp[1024];
  int nparams = 0; // will be 3 when we're all done.
  int params[3];

  while (nparams < 3)
  {
    if (fgets(tmp, sizeof(tmp), f) == NULL)
      goto error;

    // skip comments
    if (tmp[0] == '#')
      continue;

    char *p = tmp;

    if (format == -1 && tmp[0] == 'P')
    {
      format = tmp[1] - '0';
      assert(format == 5 || format == 6);
      p = &tmp[2];
    }

    // pull integers out of this line until there are no more.
    while (nparams < 3 && *p != 0)
    {
      while (*p == ' ')
        p++;

      // encounter rubbish? (End of line?)
      if (*p < '0' || *p > '9')
        break;

      int acc = 0;
      while (*p >= '0' && *p <= '9')
      {
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

  //    if (3 != fscanf(f, "P%d\n%d %d\n255\n", &format, &width, &height))
  //        goto error;

  im = image_u8_create(width, height);

  switch (format)
  {
  case 5:
  {
    for (int y = 0; y < im->height; y++)
    {
      size_t len = fread(&im->buf[y * im->stride], 1, im->width, f);
      if (len != im->width)
        goto error;
    }

    fclose (f);
    return im;
  }

  case 6:
  {
    int stride = width * 3;
    int sz = stride * height;

    uint8_t *rgb = malloc(sz);

    if (sz != fread(rgb, 1, sz, f))
      goto error;

    fclose(f);

    for (int y = 0; y < im->height; y++)
    {
      for (int x = 0; x < im->width; x++)
      {
        int r = rgb[y * stride + 3 * x + 0];
        int g = rgb[y * stride + 3 * x + 1];
        int b = rgb[y * stride + 3 * x + 2];

        int gray = (int) (0.3 * r + 0.59 * g + 0.11 * b); // XXX not gamma correct
        if (gray > 255)
          gray = 255;

        im->buf[y * im->stride + x] = gray;
      }
    }

    free(rgb);

    return im;
  }
  }

error:
  fclose(f);

  if (im != NULL)
    image_u8_destroy(im);

  return NULL;
}

image_u8_t *image_u8_create_from_rgb3(int width, int height, uint8_t *rgb, int stride)
{
  image_u8_t *im = image_u8_create(width, height);

  for (int y = 0; y < im->height; y++)
  {
    for (int x = 0; x < im->width; x++)
    {
      int r = rgb[y * stride + 3 * x + 0];
      int g = rgb[y * stride + 3 * x + 0];
      int b = rgb[y * stride + 3 * x + 0];

      int gray = (int) (0.6 * g + 0.3 * r + 0.1 * b);
      if (gray > 255)
        gray = 255;

      im->buf[y * im->stride + x] = gray;
    }
  }

  return im;
}

image_u8_t *image_u8_create_from_f32(image_f32_t *fim)
{
  image_u8_t *im = image_u8_create(fim->width, fim->height);

  for (int y = 0; y < fim->height; y++)
  {
    for (int x = 0; x < fim->width; x++)
    {
      float v = fim->buf[y * fim->stride + x];
      im->buf[y * im->stride + x] = (int) (255 * v);
    }
  }

  return im;
}


int image_u8_write_pgm(const image_u8_t *im, const char *path)
{
  FILE *f = fopen(path, "wb");
  int res = 0;

  if (f == NULL)
  {
    res = -1;
    goto finish;
  }

  fprintf(f, "P5\n%d %d\n255\n", im->width, im->height);

  for (int y = 0; y < im->height; y++)
  {
    if (im->width != fwrite(&im->buf[y * im->stride], 1, im->width, f))
    {
      res = -2;
      goto finish;
    }
  }

finish:
  if (f != NULL)
    fclose(f);

  return res;
}

void image_u8_draw_circle(image_u8_t *im, float x0, float y0, float r, int v)
{
  r = r * r;

  for (int y = y0 - r; y <= y0 + r; y++)
  {
    for (int x = x0 - r; x <= x0 + r; x++)
    {
      float d = (x - x0) * (x - x0) + (y - y0) * (y - y0);
      if (d > r)
        continue;

      int idx = y * im->stride + x;
      im->buf[idx] = v;
    }
  }
}

void image_u8_draw_annulus(image_u8_t *im, float x0, float y0, float r0, float r1, int v)
{
  r0 = r0 * r0;
  r1 = r1 * r1;

  assert(r0 < r1);

  for (int y = y0 - r1; y <= y0 + r1; y++)
  {
    for (int x = x0 - r1; x <= x0 + r1; x++)
    {
      float d = (x - x0) * (x - x0) + (y - y0) * (y - y0);
      if (d < r0 || d > r1)
        continue;

      int idx = y * im->stride + x;
      im->buf[idx] = v;
    }
  }
}

// only widths 1 and 3 supported
void image_u8_draw_line(image_u8_t *im, float x0, float y0, float x1, float y1, int v, int width)
{
  double dist = sqrtf((y1 - y0) * (y1 - y0) + (x1 - x0) * (x1 - x0));
  double delta = 0.5 / dist;

  // terrible line drawing code
  for (float f = 0; f <= 1; f += delta)
  {
    int x = ((int) (x0 * f + x1 * (1 - f)));
    int y = ((int) (y0 * f + y1 * (1 - f)));

    if (x < 0 || y < 0 || x >= im->width || y >= im->height)
      continue;

    int idx = y * im->stride + x;
    im->buf[idx] = v;
    if (width > 1)
    {
      im->buf[idx + 1] = v;
      im->buf[idx + im->stride] = v;
      im->buf[idx + 1 + im->stride] = v;
    }
  }
}

void image_u8_darken(image_u8_t *im)
{
  for (int y = 0; y < im->height; y++)
  {
    for (int x = 0; x < im->width; x++)
    {
      im->buf[im->stride * y + x] /= 2;
    }
  }
}

static void convolve(const uint8_t *x, uint8_t *y, int sz, const uint8_t *k, int ksz)
{
  assert((ksz & 1) == 1);

  for (int i = 0; i < ksz / 2; i++)
    y[i] = x[i];

  for (int i = 0; i < sz - ksz; i++)
  {
    uint32_t acc = 0;

    for (int j = 0; j < ksz; j++)
      acc += k[j] * x[i + j];

    y[ksz / 2 + i] = acc >> 8;
  }

  for (int i = sz - ksz + ksz / 2; i < sz; i++)
    y[i] = x[i];
}

void image_u8_gaussian_blur(image_u8_t *im, double sigma, int ksz)
{
  assert((ksz & 1) == 1); // ksz must be odd.

  // build the kernel.
  double dk[ksz];
  for (int i = 0; i <= ksz / 2; i++)
  {
    double v = exp(-.5 * sq(i / sigma));
    dk[ksz / 2 - i] = v;
    dk[ksz / 2 + i] = v;
  }

  // normalize
  double acc = 0;
  for (int i = 0; i < ksz; i++)
    acc += dk[i];

  for (int i = 0; i < ksz; i++)
    dk[i] /= acc;

  uint8_t k[ksz];
  for (int i = 0; i < ksz; i++)
    k[i] = dk[i] * 255;

  if (0)
  {
    for (int i = 0; i < ksz; i++)
      printf("%d %15f %5d\n", i, dk[i], k[i]);
  }

  for (int y = 0; y < im->height; y++)
  {

    uint8_t x[im->stride];
    memcpy(x, &im->buf[y * im->stride], im->stride);

    convolve(x, &im->buf[y * im->stride], im->width, k, ksz);
  }

  for (int x = 0; x < im->width; x++)
  {
    uint8_t xb[im->height];
    uint8_t yb[im->height];

    for (int y = 0; y < im->height; y++)
      xb[y] = im->buf[y * im->stride + x];

    convolve(xb, yb, im->height, k, ksz);

    for (int y = 0; y < im->height; y++)
      im->buf[y * im->stride + x] = yb[y];
  }
}

image_u8_t *image_u8_rotate(const image_u8_t *in, double rad, uint8_t pad)
{
  int iwidth = in->width, iheight = in->height;
  rad = -rad; // interpret y as being "down"

  float c = cos(rad), s = sin(rad);

  float p[][2] = { { 0, 0}, { iwidth, 0 }, { iwidth, iheight }, { 0, iheight} };

  float xmin = HUGE, xmax = -HUGE, ymin = HUGE, ymax = -HUGE;
  float icx = iwidth / 2.0, icy = iheight / 2.0;

  for (int i = 0; i < 4; i++)
  {
    float px = p[i][0] - icx;
    float py = p[i][1] - icy;

    float nx = px * c - py * s;
    float ny = px * s + py * c;

    xmin = fmin(xmin, nx);
    xmax = fmax(xmax, nx);
    ymin = fmin(ymin, ny);
    ymax = fmax(ymax, ny);
  }

  int owidth = ceil(xmax - xmin), oheight = ceil(ymax - ymin);
  image_u8_t *out = image_u8_create(owidth, oheight);

  // iterate over output pixels.
  for (int oy = 0; oy < oheight; oy++)
  {
    for (int ox = 0; ox < owidth; ox++)
    {
      // work backwards from destination coordinates...
      // sample pixel centers.
      float sx = ox - owidth / 2.0 + .5;
      float sy = oy - oheight / 2.0 + .5;

      // project into input-image space
      int ix = floor(sx * c + sy * s + icx);
      int iy = floor(-sx * s + sy * c + icy);

      if (ix >= 0 && iy >= 0 && ix < iwidth && iy < iheight)
        out->buf[oy * out->stride + ox] = in->buf[iy * in->stride + ix];
      else
        out->buf[oy * out->stride + ox] = pad;
    }
  }

  return out;
}

#ifdef __ARM_NEON__
#include <arm_neon.h>

void neon_decimate2(uint8_t * __restrict dest, int destwidth, int destheight, int deststride,
                    uint8_t * __restrict src, int srcwidth, int srcheight, int srcstride)
{
  for (int y = 0; y < destheight; y++)
  {
    for (int x = 0; x < destwidth; x += 8)
    {
      uint8x16x2_t row0 = vld2q_u8(src + 2 * x);
      uint8x16x2_t row1 = vld2q_u8(src + 2 * x + srcstride);
      uint8x16_t sum0 = vhaddq_u8(row0.val[0], row1.val[1]);
      uint8x16_t sum1 = vhaddq_u8(row1.val[0], row0.val[1]);
      uint8x16_t sum = vhaddq_u8(sum0, sum1);
      vst1q_u8(dest + x, sum);
    }
    src += 2 * srcstride;
    dest += deststride;
  }
}

void neon_decimate3(uint8_t * __restrict dest, int destwidth, int destheight, int deststride,
                    uint8_t * __restrict src, int srcwidth, int srcheight, int srcstride)
{
  for (int y = 0; y < destheight; y++)
  {
    for (int x = 0; x < destwidth; x += 8)
    {
      uint8x16x3_t row0 = vld3q_u8(src + 3 * x);
      uint8x16x3_t row1 = vld3q_u8(src + 3 * x + srcstride);
      uint8x16x3_t row2 = vld3q_u8(src + 3 * x + 2 * srcstride);

      uint8x16_t sum0 = vhaddq_u8(row0.val[0], row0.val[1]);
      uint8x16_t sum1 = vhaddq_u8(row0.val[2], row1.val[0]);
      uint8x16_t sum2 = vhaddq_u8(row1.val[1], row1.val[2]);
      uint8x16_t sum3 = vhaddq_u8(row2.val[0], row2.val[1]);

      uint8x16_t suma = vhaddq_u8(sum0, sum1);
      uint8x16_t sumb = vhaddq_u8(sum2, sum3);
      uint8x16_t sum = vhaddq_u8(suma, sumb);

      vst1q_u8(dest + x, sum);
    }
    src += 3 * srcstride;
    dest += deststride;
  }
}

void neon_decimate4(uint8_t * __restrict dest, int destwidth, int destheight, int deststride,
                    uint8_t * __restrict src, int srcwidth, int srcheight, int srcstride)
{
  for (int y = 0; y < destheight; y++)
  {
    for (int x = 0; x < destwidth; x += 8)
    {
      uint8x16x4_t row0 = vld4q_u8(src + 4 * x);
      uint8x16x4_t row1 = vld4q_u8(src + 4 * x + srcstride);
      uint8x16x4_t row2 = vld4q_u8(src + 4 * x + 2 * srcstride);
      uint8x16x4_t row3 = vld4q_u8(src + 4 * x + 3 * srcstride);

      uint8x16_t sum0, sum1;

      sum0 = vhaddq_u8(row0.val[0], row0.val[3]);
      sum1 = vhaddq_u8(row0.val[2], row0.val[1]);
      uint8x16_t suma = vhaddq_u8(sum0, sum1);

      sum0 = vhaddq_u8(row1.val[0], row1.val[3]);
      sum1 = vhaddq_u8(row1.val[2], row1.val[1]);
      uint8x16_t sumb = vhaddq_u8(sum0, sum1);

      sum0 = vhaddq_u8(row2.val[0], row2.val[3]);
      sum1 = vhaddq_u8(row2.val[2], row2.val[1]);
      uint8x16_t sumc = vhaddq_u8(sum0, sum1);

      sum0 = vhaddq_u8(row3.val[0], row3.val[3]);
      sum1 = vhaddq_u8(row3.val[2], row3.val[1]);
      uint8x16_t sumd = vhaddq_u8(sum0, sum1);

      uint8x16_t sumx = vhaddq_u8(suma, sumd);
      uint8x16_t sumy = vhaddq_u8(sumc, sumb);

      uint8x16_t sum = vhaddq_u8(sumx, sumy);

      vst1q_u8(dest + x, sum);
    }
    src += 4 * srcstride;
    dest += deststride;
  }
}

#endif

image_u8_t *image_u8_decimate(image_u8_t *im, int factor)
{
  int width = im->width, height = im->height;

  int swidth = width / factor, sheight = height / factor;

  image_u8_t *decim = image_u8_create(swidth, sheight);

#ifdef __ARM_NEON__
  if (factor == 2)
  {
    neon_decimate2(decim->buf, decim->width, decim->height, decim->stride,
                   im->buf, im->width, im->height, im->stride);
    return decim;
  }
  else if (factor == 3)
  {
    neon_decimate3(decim->buf, decim->width, decim->height, decim->stride,
                   im->buf, im->width, im->height, im->stride);
    return decim;
  }
  else if (factor == 4)
  {
    neon_decimate4(decim->buf, decim->width, decim->height, decim->stride,
                   im->buf, im->width, im->height, im->stride);
    return decim;
  }
#endif

  if (factor == 2)
  {
    for (int sy = 0; sy < sheight; sy++)
    {
      int sidx = sy * decim->stride;
      int idx = (sy * 2) * im->stride;

      for (int sx = 0; sx < swidth; sx++)
      {
        uint32_t v = im->buf[idx] + im->buf[idx + 1] +
                     im->buf[idx + im->stride] + im->buf[idx + im->stride + 1];
        decim->buf[sidx] = (v >> 2);
        idx += 2;
        sidx++;
      }
    }
  }
  else if (factor == 3)
  {
    for (int sy = 0; sy < sheight; sy++)
    {
      int sidx = sy * decim->stride;
      int idx = (sy * 3) * im->stride;

      for (int sx = 0; sx < swidth; sx++)
      {
        uint32_t v = im->buf[idx] + im->buf[idx + 1] + im->buf[idx + 2] +
                     im->buf[idx + im->stride] + im->buf[idx + im->stride + 1] + im->buf[idx + im->stride + 2] +
                     im->buf[idx + 2 * im->stride] + im->buf[idx + 2 * im->stride + 1];
        // + im->buf[idx+2*im->stride + 1];
        // deliberately omit lower right corner so there are exactly 8 samples...
        decim->buf[sidx] = (v >> 3);
        idx += 3;
        sidx++;
      }
    }
  }
  else if (factor == 4)
  {
    for (int sy = 0; sy < sheight; sy++)
    {
      int sidx = sy * decim->stride;
      int idx = (sy * 4) * im->stride;

      for (int sx = 0; sx < swidth; sx++)
      {
        uint32_t v = im->buf[idx] + im->buf[idx + 1] + im->buf[idx + 2] + im->buf[idx + 3] +
                     im->buf[idx + im->stride] + im->buf[idx + im->stride + 1] + im->buf[idx + im->stride + 1] + im->buf[idx + im->stride + 2] +
                     im->buf[idx + 2 * im->stride] + im->buf[idx + 2 * im->stride + 1] + im->buf[idx + 2 * im->stride + 2] + im->buf[idx + 2 * im->stride + 3];

        decim->buf[sidx] = (v >> 4);
        idx += 4;
        sidx++;
      }
    }
  }
  else
  {
    // XXX this isn't a very good decimation code.
    uint32_t row[swidth];

    for (int y = 0; y < height; y += factor)
    {
      memset(row, 0, sizeof(row));

      for (int dy = 0; dy < factor; dy++)
      {
        for (int x = 0; x < width; x++)
        {
          row[x / factor] += im->buf[(y + dy) * im->stride + x];
        }
      }

      for (int x = 0; x < swidth; x++)
        decim->buf[(y / factor)*decim->stride + x] = row[x] / sq(factor);

    }
  }

  return decim;
}
