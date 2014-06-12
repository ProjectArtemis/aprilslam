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
#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "apriltag.h"
#include "zarray.h"
#include "zhash.h"
#include "unionfind.h"
#include "line_fit.h"
#include "timeprofile.h"
#include "common.h"

struct edge {
  uint32_t a, b;
  int32_t score;
};

struct cluster_stat {
  int32_t gx, gy;
  line_fit_t lf;
};

struct merge_edge_task {
  april_tag_detector_t *td;
  unionfind_t *uf;
  struct cluster_stat *stats;

  struct edge *edges;
  int nedges;
};

struct fit_lines_task {
  april_tag_detector_t *td;
  unionfind_t *uf;
  zarray_t *segments;
  struct cluster_stat *stats;
  image_u8_t *im;
  image_u8_t *im_lines;

  int y0, y1;
};

struct make_edge_init_task {
  april_tag_detector_t *td;
  struct cluster_stat *stats;
  image_u8_t *im;
  int i0, i1;
};

static void make_edge_init_task(void *p) {
  struct make_edge_init_task *task = (struct make_edge_init_task *)p;

  image_u8_t *im = task->im;
  int width = im->width;
  april_tag_detector_t *td = task->td;
  struct cluster_stat *stats = task->stats;

  for (int y = task->i0; y < task->i1; y++) {
    for (int x = 0; x < width; x++) {
      int im_idx = y * im->stride + x;
      int uf_idx = y * width + x;

      int gx = im->buf[im_idx + 1] - im->buf[im_idx];
      int gy = im->buf[im_idx + im->stride] - im->buf[im_idx];

      int mag = sq(gx) + sq(gy);
      if (mag < td->min_mag) {
        // this is a bad node. Do not search for any edges
        // connected to this node, and flag it (by setting gx
        // below) so that other edges don't try to connect to
        // it either.
        stats[uf_idx].gx = INT32_MAX;
      }

      stats[uf_idx].gx = gx;
      stats[uf_idx].gy = gy;
      line_fit_init_nozero_update(&stats[uf_idx].lf, x, y, mag);
    }
  }
}

struct make_edge_task {
  april_tag_detector_t *td;
  int i0, i1;  // which rows of the image to process.
  image_u8_t *im;
  struct cluster_stat *stats;
  int edge0;  // where our first accepted edge should be written to. (INPUT)

  struct edge *edges;  // OUTPUT
  int nedges;          // number of edges that we wrote. (OUTPUT)

  int32_t max_score;  // maximum of all output edge scores. (OUTPUT)
};

// dy should be 0 or im->stride.
// Recall: edge scores are computed before any merges are performed.
static inline int32_t compute_edge_score(const struct cluster_stat *stats,
                                         int uf_idx0, int uf_idx1) {
  int32_t dot = stats[uf_idx0].gx * stats[uf_idx1].gx +
                stats[uf_idx0].gy * stats[uf_idx1].gy;

  if (dot < 0) return 0;

  return dot;
}

// sort descending score
/*
   static int edge_sort_fn(const void *_a, const void *_b)
   {
   struct edge *a = (struct edge*) _a;
   struct edge *b = (struct edge*) _b;
   return b->score - a->score;
   }
   */

#define EPS 0.00001
static float make_non_zero(float v) {
  if (fabs(v) < EPS) {
    if (v < 0) return -EPS;
    return EPS;
  }

  return v;
}

static void fit_lines_task(void *p) {
  struct fit_lines_task *task = (struct fit_lines_task *)p;
  unionfind_t *uf = task->uf;
  april_tag_detector_t *td = task->td;
  struct cluster_stat *stats = task->stats;
  zarray_t *segments = task->segments;
  image_u8_t *im_lines = task->im_lines;
  int width = task->im->width;

  for (int y = task->y0; y < task->y1; y++) {
    for (int x = 0; x < width - 1; x++) {
      uint32_t uf_idx = y * width + x;
      uint32_t rep = unionfind_get_representative(uf, uf_idx);

      // if it's not a parent node, skip it. (We've only need to
      // consider each cluster once.)
      if (rep != uf_idx) continue;

      if (unionfind_get_set_size(uf, rep) < td->min_segment_size) {
        continue;
      }

      struct cluster_stat *stat = &stats[uf_idx];
      float pmid[2];
      float u[2];

      line_fit_compute(&stat->lf, pmid, u);

      // compute end points
      float p0[2], p1[2];
      if (1) {
        // we describe the line as going through pmid along
        // direction u, such that any point on the line can be
        // written:
        //
        // p = pmid + lambda*u
        //
        // We want to find the range of lambdas that lie
        // within the bounding box.
        float lambda0 = -999999999, lambda1 = 99999999999;

        if (fabs(u[0]) > 0.05) {
          float la = (stat->lf.x0 - pmid[0]) / make_non_zero(u[0]);
          float lb = (stat->lf.x1 - pmid[0]) / make_non_zero(u[0]);

          if (la > lb) {
            // [ lb ----- la ]
            lambda0 = fmax(lambda0, lb);
            lambda1 = fmin(lambda1, la);
          } else {
            // [ la ----- lb ]
            lambda0 = fmax(lambda0, la);
            lambda1 = fmin(lambda1, lb);
          }
        }

        if (fabs(u[1]) > 0.05) {
          // Handle our minimum/maximum Y values.
          float la = (stat->lf.y0 - pmid[1]) / make_non_zero(u[1]);
          float lb = (stat->lf.y1 - pmid[1]) / make_non_zero(u[1]);

          if (la > lb) {
            lambda0 = fmax(lambda0, lb);
            lambda1 = fmin(lambda1, la);
          } else {
            lambda0 = fmax(lambda0, la);
            lambda1 = fmin(lambda1, lb);
          }
        }

        // compute the actual end points.
        p0[0] = pmid[0] + lambda0 * u[0];
        p0[1] = pmid[1] + lambda0 * u[1];

        p1[0] = pmid[0] + lambda1 * u[0];
        p1[1] = pmid[1] + lambda1 * u[1];
      }

      float segtheta = atan2(p1[1] - p0[1], p1[0] - p0[0]);
      float gradient_theta = atan2(stat->gy, stat->gx);

      // err *should* be +Math.PI/2 for the correct winding,
      // but if we've got the wrong winding, it'll be around
      // -Math.PI/2.
      float err = mod2pi(gradient_theta - segtheta + M_PI / 2);

      // Make sure segment is long enough
      float length2 = sq(p1[0] - p0[0]) + sq(p1[1] - p0[1]);
      if (length2 < sq(td->min_segment_length)) continue;

      // create a new segment
      struct segment seg;
      memset(&seg, 0, sizeof(struct segment));

      float pixel_center = 0.5;

      if (fabs(err) > M_PI / 2.0) {
        segtheta += M_PI;
        seg.p0[0] = p1[0] + pixel_center;
        seg.p0[1] = p1[1] + pixel_center;
        seg.p1[0] = p0[0] + pixel_center;
        seg.p1[1] = p0[1] + pixel_center;
      } else {
        seg.p0[0] = p0[0] + pixel_center;
        seg.p0[1] = p0[1] + pixel_center;
        seg.p1[0] = p1[0] + pixel_center;
        seg.p1[1] = p1[1] + pixel_center;
      }

      seg.theta = segtheta;

      pthread_mutex_lock(&td->mutex);
      zarray_add(segments, &seg);
      pthread_mutex_unlock(&td->mutex);

      // debug output
      if (im_lines != NULL) {
        const int bias = 30;
        int color = bias + (random() % (255 - bias));

        image_u8_draw_line(im_lines, p0[0], p0[1], p1[0], p1[1], color, 1);
        // draw notch. points to the left
        float cx = (p0[0] + p1[0]) / 2.0;
        float cy = (p0[1] + p1[1]) / 2.0;
        float nlength = 0;
        float nx = nlength * cos(segtheta + M_PI / 2);
        float ny = nlength * sin(segtheta + M_PI / 2);

        image_u8_draw_line(im_lines, cx, cy, cx + nx, cy + ny, color * .8, 1);
      }
    }
  }
}

static void merge_edge_task(void *p) {
  struct merge_edge_task *task = (struct merge_edge_task *)p;

  struct edge *edges = task->edges;
  struct cluster_stat *stats = task->stats;
  unionfind_t *uf = task->uf;
  april_tag_detector_t *td = task->td;

  for (int i = 0; i < task->nedges; i++) {

    uint32_t ida = edges[i].a;
    uint32_t idb = edges[i].b;

    ida = unionfind_get_representative(uf, ida);
    idb = unionfind_get_representative(uf, idb);

    if (ida == idb) continue;

    // a dot b = |a| |b| cos (theta)
    float dot =
        ((float)stats[ida].gx * stats[idb].gx) + stats[ida].gy * stats[idb].gy;
    float maga = sq(stats[ida].gx) + sq(stats[ida].gy);
    float magb = sq(stats[idb].gx) + sq(stats[idb].gy);

    // NB: cos(theta) = dot / sqrtf(maga * magb)
    if (dot < sqrtf(maga * magb) * td->costhresh0) continue;

    int idab = unionfind_connect(uf, ida, idb);

    stats[idab].gx = stats[ida].gx + stats[idb].gx;
    stats[idab].gy = stats[ida].gy + stats[idb].gy;

    int other = (idab == ida) ? idb : ida;
    line_fit_combine(&stats[idab].lf, &stats[other].lf);
  }
}

static void make_edge_task(void *p) {
  struct make_edge_task *task = (struct make_edge_task *)p;

  april_tag_detector_t *td = task->td;
  image_u8_t *im = task->im;
  struct cluster_stat *stats = task->stats;
  struct edge *edges = &task->edges[task->edge0];

  int min_edge_score = td->min_edge_score;

  int width = im->width, height = im->height;

  int nedges = 0;

  int32_t max_score = 0;

  for (int y = task->i0; y < task->i1; y++) {
    for (int x = 0; x < width - 1; x++) {

      int uf_idx = y * width + x;

      if (stats[uf_idx].gx == INT32_MAX) continue;

      // don't create edges to nodes for which we cannot
      // compute gradients.
      if (y < 1 || x < 1 || y >= (height - 1) || x >= (width - 1)) continue;

// always point to previously-initialized entries...

// dx should be {1, 0, -1}, dy should be {0,  -1}
#define DO_EDGE(dx, dy)                                            \
  {                                                                \
    uint32_t uf_idx1 = uf_idx + dx + dy * width;                   \
                                                                   \
    if (stats[uf_idx1].gx != INT32_MAX) {                          \
      int edge_score = compute_edge_score(stats, uf_idx, uf_idx1); \
      if (edge_score > min_edge_score) {                           \
        edges[nedges].a = uf_idx;                                  \
        edges[nedges].b = uf_idx1;                                 \
        edges[nedges].score = edge_score;                          \
        if (edge_score > max_score) max_score = edge_score;        \
        nedges++;                                                  \
      }                                                            \
    }                                                              \
  }

      // left-up
      DO_EDGE(-1, -1);

      // up
      DO_EDGE(0, -1);

      // right-up
      DO_EDGE(1, -1);

      // left
      DO_EDGE(-1, 0);
    }
  }

  task->nedges = nedges;
  task->max_score = max_score;
}

zarray_t *segment2(april_tag_detector_t *td, image_u8_t *im) {
  timeprofile_stamp(td->tp, "SEG:segment begin");

  int width = im->width, height = im->height;

  ///////////////////////////////////////////////////////////
  // Step three. Segment the edges, grouping pixels with similar
  // thetas together. This is a greedy algorithm: we start with
  // the most similar pixels.

  timeprofile_stamp(td->tp, "SEG:cluster begin");

  unionfind_t *uf = unionfind_create(width * height - 1);

  struct cluster_stat *stats = (struct cluster_stat *)calloc(
      width * height, sizeof(struct cluster_stat));

  // initialize the cluster_stats for each pixel. This used to be in
  // make_edges, but make_edges accesses neighboring pixels. When
  // make_edges is multi-threaded, these accesses can be to edges
  // that will be populated by other threads, creating a race
  // condition. Thus, we do all of the initialization FIRST, so that
  // make edges can actually proceed.
  if (1) {
    int chunksize =
        1 + height / (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);
    struct make_edge_init_task tasks[height / chunksize + 1];

    int ntasks = 0;
    for (int i = 0; i < height - 1; i += chunksize) {
      tasks[ntasks].td = td;
      tasks[ntasks].i0 = i;
      tasks[ntasks].i1 = imin(height - 1, i + chunksize);
      tasks[ntasks].im = im;
      tasks[ntasks].stats = stats;

      workerpool_add_task(td->wp, make_edge_init_task, &tasks[ntasks]);
      ntasks++;
    }

    workerpool_run(td->wp);
  }

  timeprofile_stamp(td->tp, "SEG:make edges (init)");

  if (1) {
    struct edge *edges =
        (struct edge *)calloc(width * height * 4, sizeof(struct edge));

    int chunksize =
        1 + height / (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);

    struct make_edge_task tasks[height / chunksize + 1];

    int ntasks = 0;
    for (int i = 0; i < height - 1; i += chunksize) {
      tasks[ntasks].td = td;
      tasks[ntasks].i0 = i;
      tasks[ntasks].i1 = imin(height - 1, i + chunksize);
      tasks[ntasks].im = im;
      tasks[ntasks].stats = stats;
      tasks[ntasks].edges = edges;
      tasks[ntasks].edge0 = i * width * 4;
      tasks[ntasks].nedges = 0;

      workerpool_add_task(td->wp, make_edge_task, &tasks[ntasks]);
      ntasks++;
    }

    workerpool_run(td->wp);

    timeprofile_stamp(td->tp, "SEG:make edges");

    int nedges = 0;  // total number of edges, computed below.

    // counting sort
    if (1) {
      int max_score = 0;
      for (int taskidx = 0; taskidx < ntasks; taskidx++) {
        if (tasks[taskidx].max_score > max_score)
          max_score = tasks[taskidx].max_score;

        nedges += tasks[taskidx].nedges;
      }

      td->nedges = nedges;

      timeprofile_stamp(td->tp, "SEG:sort (scan)");

      // for cost 'v', counts[v] will give the output position
      // for the next sample with cost v.
      int ncounts = max_score + 2;
      uint32_t counts[ncounts];
      memset(counts, 0, sizeof(counts));

      // how many of each value are there?
      for (int taskidx = 0; taskidx < ntasks; taskidx++) {
        int edge0 = tasks[taskidx].edge0;
        for (int i = 0; i < tasks[taskidx].nedges; i++) {
          int score = edges[edge0 + i].score;
          counts[score + 1]++;
        }
      }

      // build cumulative counts
      for (int i = 1; i < ncounts; i++) counts[i] += counts[i - 1];

      struct edge *new_edges =
          (struct edge *)calloc(nedges, sizeof(struct edge));

      for (int taskidx = 0; taskidx < ntasks; taskidx++) {
        int edge0 = tasks[taskidx].edge0;
        for (int i = 0; i < tasks[taskidx].nedges; i++) {
          int w = edges[edge0 + i].score;
          new_edges[counts[w]] = edges[edge0 + i];
          counts[w]++;
        }
      }

      // testing
      if (td->debug) {
        for (int i = 0; i + 1 < nedges; i++) {
          assert(new_edges[i].score <= new_edges[i + 1].score);
        }
      }

      free(edges);
      edges = new_edges;
    }

    timeprofile_stamp(td->tp, "SEG:sort");

    // process edges in order of DECREASING weight, merging
    // clusters if we can do so without exceeding the
    // thetaThresh.

    // this code slower than the single-threaded? (and not quite right ==>
    // disabled)
    // (this code is faster on ARM, slower on x86)
    if (0 && td->nthreads > 1) {
      // we attempt to process edges in parallel, taking
      // advantage of the general tendency for edges to connect
      // nearby nodes (and thus not require global
      // synchronization).  We divide each thread according to
      // which PIXELS it owns.
      int thread_pixels = width * height / td->nthreads;
      assert(td->nthreads * thread_pixels == width * height);

      struct merge_edge_task tasks[td->nthreads + 1];
      for (int i = 0; i < td->nthreads + 1; i++) {
        tasks[i].td = td;
        tasks[i].uf = uf;
        tasks[i].stats = stats;

        tasks[i].nedges = 0;

        if (i == td->nthreads) {
          tasks[i].edges = malloc(nedges * sizeof(struct edge));
        } else {
          tasks[i].edges =
              malloc(width * height * 4 / td->nthreads * sizeof(struct edge));
          workerpool_add_task(td->wp, merge_edge_task, &tasks[i]);
        }
      }

      uint8_t *flags = calloc(width * height, 1);

      // XXX this code not quite right?
      // Assume thread group size of 10. We process the following edges
      // (11, 12)
      // (12, 22) <-- mark both 12 and 22 in conflict. 11 should also be
      // implicated, but isn't.
      // (13, 11) <-- incorrectly allowed to proceed in parallel.

      for (int i = nedges - 1; i >= 0; i--) {
        uint32_t ida = edges[i].a;
        uint32_t idb = edges[i].b;

        // if one of the pixels is already dependent on a
        // cross-thread operation, then add this edge to the
        // "conflicts" queue. Note that if one of the pixels
        // was not previously dependent, it is now!
        if (flags[ida] || flags[idb]) {
          flags[idb] = 1;
          flags[ida] = 1;
          tasks[td->nthreads].edges[tasks[td->nthreads].nedges++] = edges[i];
          continue;
        }

        // are both of these pixels assigned to the same
        // thread? If so, hurray! We can process it in
        // parallel.
        int threada = ida / thread_pixels;
        int threadb = idb / thread_pixels;
        if (threada == threadb) {
          tasks[threada].edges[tasks[threada].nedges++] = edges[i];
          continue;
        }

        // These pixels are not in the same thread group. Mark
        // them as in conflict and add this edge to the
        // conflict queue.
        flags[ida] = 1;
        flags[idb] = 1;
        tasks[td->nthreads].edges[tasks[td->nthreads].nedges++] = edges[i];
      }

      timeprofile_stamp(td->tp, "SEG:parallelize");

      workerpool_run(td->wp);

      timeprofile_stamp(td->tp, "SEG:merge edges (parallel)");

      // Manually invoke the last segment (which handles conflicts).
      merge_edge_task(&tasks[td->nthreads]);

      free(flags);

      for (int i = 0; i < td->nthreads + 1; i++) {
        free(tasks[i].edges);
      }

      timeprofile_stamp(td->tp, "SEG:merge edges (conflicts)");

    } else {

      double costhresh02 = td->costhresh0 * td->costhresh0;

      // single threaded version
      for (int i = nedges - 1; i >= 0; i--) {
        uint32_t ida = edges[i].a;
        uint32_t idb = edges[i].b;

        ida = unionfind_get_representative(uf, ida);
        idb = unionfind_get_representative(uf, idb);

        if (ida == idb) continue;

        // a dot b = |a| |b| cos (theta)
        float dot = ((float)stats[ida].gx * stats[idb].gx) +
                    stats[ida].gy * stats[idb].gy;
        float dot2 = dot * dot;

        float maga = sq(stats[ida].gx) + sq(stats[ida].gy);
        float magb = sq(stats[idb].gx) + sq(stats[idb].gy);

        // NB: cos(theta) = dot / sqrtf(maga * magb)
        // XXX TODO eliminate sqrt by squaring the other side.
        if (dot2 < maga * magb * costhresh02) continue;

        int idab = unionfind_connect(uf, ida, idb);

        stats[idab].gx = stats[ida].gx + stats[idb].gx;
        stats[idab].gy = stats[ida].gy + stats[idb].gy;

        int other = (idab == ida) ? idb : ida;
        line_fit_combine(&stats[idab].lf, &stats[other].lf);
      }
    }

    free(edges);

    timeprofile_stamp(td->tp, "SEG:merge edges");
  }

  // make segmentation image.
  if (td->debug) {
    image_u8_t *d = image_u8_create(width, height);

    uint8_t *colors = (uint8_t *)calloc(width * height, 1);

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        uint32_t v = unionfind_get_representative(uf, y * width + x);
        uint32_t sz = unionfind_get_set_size(uf, y * width + x);
        if (sz < td->min_segment_size) continue;

        uint8_t color = colors[v];

        if (color == 0) {
          const int bias = 20;
          color = bias + (random() % (255 - bias));
          colors[v] = color;
        }
        d->buf[y * d->stride + x] = color;
      }
    }

    free(colors);

    image_u8_write_pgm(d, "debug_segmentation.pnm");
    image_u8_destroy(d);
  }

  image_u8_t *im_lines = td->debug ? image_u8_copy(im) : NULL;
  if (im_lines != NULL) image_u8_clear(im_lines);

  ///////////////////////////////////////////////////////////
  zarray_t *segments = zarray_create(sizeof(struct segment));

  if (1) {
    int chunksize =
        1 + height / (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);

    struct fit_lines_task tasks[height / chunksize + 1];
    int ntasks = 0;
    for (int i = 0; i < height - 1; i += chunksize) {
      tasks[ntasks].td = td;
      tasks[ntasks].uf = uf;
      tasks[ntasks].segments = segments;
      tasks[ntasks].stats = stats;
      tasks[ntasks].im = im;
      tasks[ntasks].im_lines = im_lines;
      tasks[ntasks].y0 = i;
      tasks[ntasks].y1 = imin(height - 1, i + chunksize);

      workerpool_add_task(td->wp, fit_lines_task, &tasks[ntasks]);
      ntasks++;
    }

    // the segment tasks call unionfind_get_representative, which
    // is not thread safe if updates actually occur. Thus we
    // resolve all the connectivity serially...
    if (1) {
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          unionfind_get_representative(uf, y * width + x);
        }
      }
    }

    workerpool_run(td->wp);
  }

  td->nsegments = zarray_size(segments);

  timeprofile_stamp(td->tp, "SEG:fit lines");

  if (im_lines != NULL) {
    image_u8_write_pgm(im_lines, "debug_lines.pnm");
    image_u8_destroy(im_lines);
  }

  unionfind_destroy(uf);

  free(stats);

  timeprofile_stamp(td->tp, "SEG:cleanup");

  return segments;
}
