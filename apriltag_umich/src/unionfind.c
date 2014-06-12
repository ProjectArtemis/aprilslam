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
#include "unionfind.h"
#include <stdlib.h>
#include <assert.h>

unionfind_t *unionfind_create(uint32_t maxid) {
  unionfind_t *uf = (unionfind_t *)calloc(1, sizeof(unionfind_t));
  uf->maxid = maxid;
  uf->data = (struct ufrec *)calloc(maxid + 1, sizeof(struct ufrec));

  return uf;
}

void unionfind_destroy(unionfind_t *uf) {
  free(uf->data);
  free(uf);
}
