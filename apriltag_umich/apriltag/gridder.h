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

#ifndef _GRIDDER_H
#define _GRIDDER_H

#include <stdlib.h>

#define GRIDDER_MIN_ALLOC 8

struct gridder_cell {
  int alloc;
  int size;
  void **p;
};

typedef struct gridder gridder_t;
struct gridder {
  int x0, y0;
  int cell_size;
  int width, height;

  struct gridder_cell *cells;
};

typedef struct gridder_iterator gridder_iterator_t;
struct gridder_iterator {
  gridder_t *g;
  int ix0, iy0, ix1, iy1;
  int ix, iy;
  int pos;
};

static inline gridder_t *gridder_create(int x0, int y0, int x1, int y1, int cell_size) {
  gridder_t *g = calloc(1, sizeof(gridder_t));
  g->x0 = x0;
  g->y0 = y0;
  g->cell_size = cell_size;

  g->width = (int) (x1 - x0 + cell_size - 1) / cell_size;
  g->height = (int) (y1 - y0 + cell_size - 1) / cell_size;

  g->cells = calloc(g->width * g->height, sizeof(struct gridder_cell));
  return g;
}

static inline void gridder_destroy(gridder_t *g) {
  for (int idx = 0; idx < g->width * g->height; idx++)
    free(g->cells[idx].p);

  free(g->cells);
  free(g);
}

static inline void gridder_add(gridder_t *g, int x, int y, void *p) {
  int ix = (x - g->x0) / g->cell_size;
  if (ix < 0 || ix >= g->width)
    return;

  int iy = (y - g->y0) / g->cell_size;
  if (iy < 0 || iy >= g->height)
    return;

  int idx = iy*g->width + ix;

  struct gridder_cell *cell = &g->cells[idx];

  if (cell->size == cell->alloc) {
    int newalloc = cell->alloc * 2;
    if (newalloc < GRIDDER_MIN_ALLOC)
      newalloc = GRIDDER_MIN_ALLOC;

    cell->p = realloc(cell->p, sizeof(void*) * newalloc);
    cell->alloc = newalloc;
  }

  cell->p[cell->size++] = p;
}

static inline int clamp(int x, int x0, int x1) {
  x = x < x0 ? x0 : x;
  x = x > x1 ? x1 : x;
  return x;
}

static inline void gridder_iterator_init(gridder_t *g, gridder_iterator_t *gi, int cx, int cy, int r) {
  gi->g = g;
  gi->ix0 = clamp((cx - r - g->x0) / g->cell_size,  0, g->width - 1);
  gi->ix1 = clamp((cx + r - g->x0) / g->cell_size,  0, g->width - 1);
  gi->iy0 = clamp((cy - r - g->y0) / g->cell_size,  0, g->height - 1);
  gi->iy1 = clamp((cy + r - g->y0) / g->cell_size,  0, g->height - 1);

  gi->ix = gi->ix0;
  gi->iy = gi->iy0;
  gi->pos = 0;
}

// returns null if no more.
static inline void *gridder_iterator_next(gridder_iterator_t *gi) {
  gridder_t *g = gi->g;

  while (1) {
    int idx = gi->iy*g->width + gi->ix;

    if (gi->pos < g->cells[idx].size)
      return g->cells[idx].p[gi->pos++];

    if (gi->ix < gi->ix1) {
      gi->ix++;
      gi->pos = 0;
      continue;
    }

    if (gi->iy < gi->iy1) {
      gi->ix = gi->ix0;
      gi->iy++;
      gi->pos = 0;
      continue;
    }

    return NULL;
  }
}

#endif
