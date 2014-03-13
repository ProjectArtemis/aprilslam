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
#ifndef _WORKERPOOL_H
#define _WORKERPOOL_H

#include "zarray.h"

typedef struct workerpool workerpool_t;

// as a special case, if nthreads==1, no additional threads are
// created, and workerpool_run will run synchronously.
workerpool_t *workerpool_create(int nthreads);
void workerpool_destroy(workerpool_t *wp);

void workerpool_add_task(workerpool_t *wp, void (*f)(void *p), void *p);

// runs all added tasks, waits for them to complete.
void workerpool_run(workerpool_t *wp);

// same as workerpool_run, except always single threaded. (mostly for debugging).
void workerpool_run_single(workerpool_t *wp);

int workerpool_get_nthreads(workerpool_t *wp);

#endif
