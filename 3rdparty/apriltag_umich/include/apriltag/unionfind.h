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
#ifndef _UNIONFIND_H
#define _UNIONFIND_H

#include <stdint.h>

typedef struct unionfind unionfind_t;

struct unionfind {
  uint32_t maxid;
  struct ufrec *data;
};

struct ufrec {
  // a parent of zero means that the parent is its own parent. (this
  // speeds initialization).  This is slightly interesting for node
  // 0, but it is initialized to its own parent, so it works out
  // okay.
  uint32_t parent;

  // actual size - 1 (so that on initialization it can be zero)
  uint32_t size;
};

unionfind_t *unionfind_create(uint32_t maxid);
void unionfind_destroy(unionfind_t *uf);

static inline uint32_t unionfind_get_representative(unionfind_t *uf,
                                                    uint32_t id) {
  // base case: a node is its own parent
  if (uf->data[id].parent == 0) return id;

  // otherwise, recurse
  uint32_t root = unionfind_get_representative(uf, uf->data[id].parent);

  // short circuit the path
  uf->data[id].parent = root;

  return root;
}

static inline uint32_t unionfind_get_set_size(unionfind_t *uf, uint32_t id) {
  uint32_t repid = unionfind_get_representative(uf, id);
  return uf->data[repid].size + 1;
}

static inline uint32_t unionfind_connect(unionfind_t *uf, uint32_t aid,
                                         uint32_t bid) {
  uint32_t aroot = unionfind_get_representative(uf, aid);
  uint32_t broot = unionfind_get_representative(uf, bid);

  if (aroot == broot) return aroot;

  uint32_t asize = uf->data[aroot].size;
  uint32_t bsize = uf->data[broot].size;

  if (asize > bsize) {
    uf->data[broot].parent = aroot;
    uf->data[aroot].size += bsize + 1;
    return aroot;
  } else {
    uf->data[aroot].parent = broot;
    uf->data[broot].size += asize + 1;
    return broot;
  }
}
#endif
