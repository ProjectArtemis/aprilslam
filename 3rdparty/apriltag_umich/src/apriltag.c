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

#include "apriltag/apriltag.h"

#include <math.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/time.h>

#include "apriltag/image_f32.h"
#include "apriltag/image_u8.h"
#include "apriltag/image_u32.h"
#include "apriltag/unionfind.h"
#include "apriltag/zhash.h"
#include "apriltag/zarray.h"
#include "apriltag/matd.h"
#include "apriltag/homography.h"
#include "apriltag/graymodel.h"
#include "apriltag/timeprofile.h"
#include "apriltag/common.h"
#include "apriltag/g2d.h"

#include "apriltag/gridder.h"

#ifndef PI
#define PI 3.1415926535897932384626
#endif

extern zarray_t *segment2(april_tag_detector_t *td, image_u8_t *im);

/* Ideas:

   Use a better homography estimation method than DLT.

   Implement bluring

   In-place camera distortion handling

   Does union find work better with rank?

   Infer the position of the 4th edge of the quad purely from the
   other 3 edges.... should help with occlusions.

   Perform a refinement of the homography by sub-sampling each grid
   cell (perhaps 4 or 9 times) and looking for the alignment that
   produces the largest number of correct bits. (This is a binary
   version of what we proposed to pradeep.)

   Homography estimation comparison ideas:
 * DLT
 * DLT in which only two of the three cross product eqns are used in each case
 * Using ordinary SVD / Using inverse
 * Precision with float / double


 Make quad_search directly attempt to decode the quads. It could
 terminate a search if it finds a good quad. We could also order the
 neighbors in each segment so that angles close to 90 degrees get
 tried first. (This would only help speed things up when we actually
 found a decode).
 */

struct quad {
  struct segment *segments[4];
  float p[4][2];  // corners
};

// sort in ascending order of theta.
static inline int segment_compare_function(const void *_a, const void *_b) {
  struct segment *sega = ((struct segment *)_a);
  struct segment *segb = ((struct segment *)_b);

  if (sega->theta >= segb->theta) return 1;
  return -1;
}

static inline int detection_compare_function(const void *_a, const void *_b) {
  april_tag_detection_t *a = *(april_tag_detection_t **)_a;
  april_tag_detection_t *b = *(april_tag_detection_t **)_b;

  return a->id - b->id;
}
static int line_intersection(const float a0[2], const float a1[2],
                             const float b0[2], const float b1[2], float p[2]) {
  // solve for parameters L1 and L2 such that
  // a0 + (a1-a0)L1 = b0 + (b1-b0)L2
  //
  // rearrange:
  //
  // (a1-a0)L1 - (b1-b0)L2 = (b0 - a0)
  //
  float A00 = a1[0] - a0[0], A01 = -(b1[0] - b0[0]);
  float A10 = a1[1] - a0[1], A11 = -(b1[1] - b0[1]);

  float B0 = b0[0] - a0[0];
  float B1 = b0[1] - a0[1];

  float det = A00 * A11 - A10 * A01;
  // rank deficient?
  if (fabs(det) < 0.001) return -1;

  // inverse.
  float W00 = A11 / det, W01 = -A01 / det;
  //    float W10 = -A10 / det, W11 = A00 / det;

  // solve
  float L1 = W00 * B0 + W01 * B1;
  //    float L2 = W10*B0 + W11*B1;

  // compute intersection
  p[0] = a0[0] + (a1[0] - a0[0]) * L1;
  p[1] = a0[1] + (a1[1] - a0[1]) * L1;

  //    p[0] = b0[0] + (b1[0]-b0[0])*L2;
  //    p[1] = b0[1] + (b1[1]-b0[1])*L2;

  return 0;
}

/** if the bits in w were arranged in a d*d grid and that grid was
 * rotated, what would the new bits in w be?
 * The bits are organized like this (for d = 3):
 *
 *  8 7 6       2 5 8      0 1 2
 *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
 *  2 1 0       0 3 6      6 7 8
 **/
static uint64_t rotate90(uint64_t w, uint32_t d) {
  uint64_t wr = 0;

  for (int32_t r = d - 1; r >= 0; r--) {
    for (int32_t c = 0; c < d; c++) {
      int32_t b = r + d * c;

      wr = wr << 1;

      if ((w & (((uint64_t)1) << b)) != 0) wr |= 1;
    }
  }

  return wr;
}

/** How many bits are set in the long? **/
static uint32_t pop_count_slow(uint64_t w) {
  uint32_t cnt = 0;
  while (w != 0) {
    w &= (w - 1);
    cnt++;
  }
  return cnt;
}

// we use 12 bits as a compromise between table size and number of
// iterations. Note that 36 bits is the most common size of tag.
static uint8_t pop_count_table[4096];

static void pop_count_fast_setup() {
  for (int i = 0; i < 4096; i++) pop_count_table[i] = pop_count_slow(i);
}

// Not as fast as the table lookup on my 2012 rMBP
/*
   static uint32_t popcount64(uint64_t x)
   {
   x = (x & 0x5555555555555555ULL) + ((x >> 1) & 0x5555555555555555ULL);
   x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
   x = (x & 0x0F0F0F0F0F0F0F0FULL) + ((x >> 4) & 0x0F0F0F0F0F0F0F0FULL);
   return (x * 0x0101010101010101ULL) >> 56;
   }
   */

static uint32_t pop_count_fast(uint64_t w) {
#ifdef __SSE4_2__
  return __builtin_popcountll(w);
#endif

  uint32_t popcnt = 0;

  while (w != 0) {
    popcnt += pop_count_table[w & 0xfff];
    w >>= 12;
  }

  return popcnt;
}

static uint32_t rgb_scale(uint32_t rgb, float a) {
  int r = (rgb >> 16) & 0xff;
  int g = (rgb >> 8) & 0xff;
  int b = (rgb >> 0) & 0xff;

  r *= a;
  g *= a;
  b *= a;

  return (r << 16) | (g << 8) | b;
}

/** Compute the hamming distance between two longs. **/
static uint32_t hamming_distance(uint64_t a, uint64_t b) {
  return pop_count_fast(a ^ b);
}

/** Given an observed tag with code 'rcode', try to recover the
 * id. returns 1 if the tag was successfully decoded, though
 * many apps may wish to further restrict the hamming distance.
 *
 * rotatedcode: the true rotated code to which rcode was matched.
 *
 * guessid: if you have a guess for what the id is, pass that in. If
 * UINT32_MAX, no guess is assumed.
 **/
static int decode_tag(april_tag_family_t *tf, uint64_t rcode, uint32_t *id,
                      uint32_t *hamming, uint32_t *rotation,
                      uint64_t *rotatedcode, uint32_t guess_id) {
  uint32_t bestid = UINT32_MAX;
  uint32_t besthamming = tf->d * tf->d;
  uint32_t bestrotation = 0;

  uint64_t rcodes[4];

  rcodes[0] = rcode;
  rcodes[1] = rotate90(rcodes[0], tf->d);
  rcodes[2] = rotate90(rcodes[1], tf->d);
  rcodes[3] = rotate90(rcodes[2], tf->d);

  // if we find a tag with a hamming distance less than td->h / 2,
  // it must be the closest.
  if (guess_id < tf->ncodes) {
    for (int rot = 0; rot < 4; rot++) {
      int thishamming = hamming_distance(rcodes[rot], tf->codes[guess_id]);
      if (thishamming < besthamming) {
        besthamming = thishamming;
        bestrotation = rot;
        bestid = guess_id;

        if (besthamming < (tf->h / 2)) {
          goto done;
        }
      }
    }
  }

  for (int32_t id = 0; id < tf->ncodes; id++) {

    for (int rot = 0; rot < 4; rot++) {
      int thishamming = hamming_distance(rcodes[rot], tf->codes[id]);
      if (thishamming < besthamming) {
        besthamming = thishamming;
        bestrotation = rot;
        bestid = id;

        if (besthamming < (tf->h / 2)) goto done;
      }
    }
  }

done:

  if (besthamming < tf->h / 2) {
    *id = bestid;
    *rotation = bestrotation;
    *hamming = besthamming;
    *rotatedcode = tf->codes[bestid];
    for (int i = 0; i < 4 - bestrotation; i++)
      *rotatedcode = rotate90(*rotatedcode, tf->d);
    return 1;
  }

  return 0;
}

april_tag_detector_t *april_tag_detector_create(april_tag_family_t *fam) {
  pop_count_fast_setup();

  april_tag_detector_t *td =
      (april_tag_detector_t *)calloc(1, sizeof(april_tag_detector_t));

  td->nthreads = 1;

  td->seg_decimate = 0;
  td->seg_sigma = 0;

  // XXX is there a relationship between these two? i.e., should
  // they always be equal?
  td->min_mag = 20;                  // squared magnitude of gradient. (a'a)
  td->min_edge_score = td->min_mag;  // dot product. (a'b)

  td->min_white_black_diff = 5;

  td->costhresh0 = .9;

  td->min_segment_size = 4;

  // no real hope of detecting quads that provide less than one
  // pixel per payload
  td->min_segment_length = 4;  // 2+fam->d;
  td->max_segment_distance = 30;

  td->tag_families = zarray_create(sizeof(april_tag_family_t *));
  if (fam != NULL) zarray_add(td->tag_families, &fam);

  td->min_tag_size = 6;
  td->max_aspect_ratio = 6;
  td->critical_rad = 20 * M_PI / 180;
  pthread_mutex_init(&td->mutex, NULL);

  td->tp = timeprofile_create();

  // NB: defer initialization of td->wp so that the user can
  // override td->nthreads.

  return td;
}

void april_tag_detector_destroy(april_tag_detector_t *td) {
  timeprofile_destroy(td->tp);
  workerpool_destroy(td->wp);
  free(td);
}

// quads: all found quads are copied into here.
//
// pos (like "depth"): the index of the last valid segment. (0 means
// one valid segment) quad temporary storage used during depth-first
// search. (A copy is made when a quad is found).
//
// TODO: Allow two neighboring segments to be merged into a single
// segment if they are nearly colinear... allows better reconstruction
// across partially occluded edges. (Or should we do this on the raw
// line segments before calling search?)
static void quad_search(zarray_t *quads, april_tag_detector_t *td, int pos,
                        struct quad *quad) {
  struct segment *seg = quad->segments[pos];

  if (seg->neighbors == NULL) return;

  for (int i = 0; i < zarray_size(seg->neighbors); i++) {
    // neighbor is the next potential edge (at index pos+1).
    struct segment *neighbor;
    zarray_get(seg->neighbors, i, &neighbor);

    // only detect each quad one time by imposing an arbitrary order:
    // the first segment must have the right-most point 0.
    if (neighbor->p0[0] < quad->segments[0]->p0[0]) continue;

    // the last segment must form a loop with the first segment.
    if (pos == 3) {
      if (neighbor != quad->segments[0]) continue;
    } else {
      // otherwise, this segment must be different than all previous segments.
      int bad = 0;
      for (int j = 0; j <= pos; j++)
        if (quad->segments[j] == neighbor) {
          bad = 1;
          break;
        }
      if (bad) continue;
    }

    // Compute the corner between this segment and the previous
    // one. If they're too parallel, reject this edge.
    if (line_intersection(quad->segments[pos]->p0, quad->segments[pos]->p1,
                          neighbor->p0, neighbor->p1, quad->p[pos]))
      continue;

    // ensure left-hand winding
    if (0 && pos >= 2) {
      // we just wrote quad->p[pos]. This creates a new angle
      // with the two previous quad->ps.
      // XXX we're calling atan2f unnecessarily often.
      int a = pos & 3, b = pos - 1, c = pos - 2;

      double theta1 =
          atan2f(quad->p[a][1] - quad->p[b][1], quad->p[a][0] - quad->p[b][0]);

      double theta0 =
          atan2f(quad->p[b][1] - quad->p[c][1], quad->p[b][0] - quad->p[c][0]);

      double dtheta = mod2pi(theta1 - theta0);
      if (dtheta < 0) continue;
    }

    if (pos == 3) {
      // possibly produce a quad.

      // ensure left-hand winding. (Check the remaining corners).
      int bad = 0;
      for (int j = 0; j < 4; j++) {
        int a = (j + 1) & 3;
        int b = j;
        int c = (j + 3) & 3;
        float theta1 = atan2f(quad->p[a][1] - quad->p[b][1],
                              quad->p[a][0] - quad->p[b][0]);

        float theta0 = atan2f(quad->p[b][1] - quad->p[c][1],
                              quad->p[b][0] - quad->p[c][0]);

        float dtheta = mod2pi(theta1 - theta0);
        if (dtheta < 0) {
          //       printf("%f\n", dtheta); //assert(j==3);
          bad = 1;
          break;
        }
      }

      if (bad) continue;

      // Even with the critical_angle test, this is useful in
      // the event that one edge is dramatically longer than the
      // other.
      float mindist2 = 999999999;
      float maxdist2 = 0;

      // could eliminate sqrtf by redefining minTagSize and maxAspectRatio
      for (int a = 0; a <= 3; a++) {
        for (int b = a + 1; b <= 3; b++) {
          float dist2 = sq(quad->p[a][0] - quad->p[b][0]) +
                        sq(quad->p[a][1] - quad->p[b][1]);
          if (dist2 < mindist2) mindist2 = dist2;
          if (dist2 > maxdist2) maxdist2 = dist2;
        }
      }

      if (mindist2 < sq(td->min_tag_size)) continue;
      if (maxdist2 > mindist2 * sq(td->max_aspect_ratio)) continue;

      pthread_mutex_lock(&td->mutex);
      zarray_add(quads, quad);
      pthread_mutex_unlock(&td->mutex);

      // XXX Decode the quad here and now? Maybe get slightly
      // better multi-core utilization by avoiding two dispatch
      // cycles?
    } else {
      quad->segments[pos + 1] = neighbor;
      quad_search(quads, td, pos + 1, quad);
    }
  }
}

struct neighbor_search_task {
  gridder_t *gridder;
  zarray_t *segments;
  april_tag_detector_t *td;
  int i0, i1;
};

static void neighbor_search_task(void *_u) {
  struct neighbor_search_task *task = (struct neighbor_search_task *)_u;
  april_tag_detector_t *td = task->td;

  float min_segment_length2 = sq(td->min_segment_length);
  float max_segment_distance2 = sq(td->max_segment_distance);

  for (int i = task->i0; i < task->i1; i++) {
    struct segment *seg;

    zarray_get_volatile(task->segments, i, &seg);
    float length2 = sq(seg->p1[0] - seg->p0[0]) + sq(seg->p1[1] - seg->p0[1]);

    if (length2 < min_segment_length2) continue;

    float theta0 = atan2f(seg->p1[1] - seg->p0[1], seg->p1[0] - seg->p0[0]);

    seg->neighbors = zarray_create(sizeof(struct segment *));

    gridder_iterator_t git;
    gridder_iterator_init(task->gridder, &git, seg->p1[0], seg->p1[1],
                          task->td->max_segment_distance);

    float min_dist2 = 999999;

    while (1) {
      struct segment *seg2 = gridder_iterator_next(&git);
      if (seg2 == NULL) break;

      // XXX maybe superfluous? (only slightly stronger
      // constraint than what the gridder does for us.)
      float dist2 = sq(seg->p1[0] - seg2->p0[0]) + sq(seg->p1[1] - seg2->p0[1]);
      if (dist2 > max_segment_distance2) continue;

      if (dist2 < min_dist2) min_dist2 = dist2;

      // enforce winding order constraint (approximately! note
      // that because the quad is actually defined by the
      // intersections of the lines, which can happen anywhere
      // along the segments, this check isn't quite sufficient.
      // well double check for well-formed-ness in
      // quad_search. But doing this here reduces the search
      // space.
      float theta1 =
          atan2f(seg2->p1[1] - seg2->p0[1], seg2->p1[0] - seg2->p0[0]);

      float dtheta = mod2pi(theta1 - theta0);

      // prevent quads that have high aspect ratios by
      // eliminating "corners" that have either extremely sharp
      // angles or extremely flat angles.
      if (dtheta < td->critical_rad || dtheta > (M_PI - td->critical_rad))
        continue;

      float K1 = 1 / 1.157;  // reaches a value of 1 at 90 degrees.
      float K2 = 3;          // reaches a value of 1 at 10%
      float K3 = 0;
      float K4 = -2.5;

      float neighbor_cost = K1 * fabs(dtheta - M_PI / 2) +
                            K2 * sqrtf(dist2 / length2) + K3 * sqrtf(dist2) +
                            K4;

      if (neighbor_cost <= 0) zarray_add(seg->neighbors, &seg2);
    }

    /*
       for (int i = 0; i < zarray_size(seg->neighbors); i++) {
       struct segment *seg2;
       zarray_get(seg->neighbors, i, &seg2);

       float dist2 = sq(seg->p1[0]-seg2->p0[0]) + sq(seg->p1[1]-seg2->p0[1]);
       if (0 && dist2 > min_dist2 + 5) {
       zarray_remove_index(seg->neighbors, i, 1);
       i--;
       }
       }
       */
  }
}

struct quad_search_task {
  zarray_t *segments;
  zarray_t *quads;
  april_tag_detector_t *td;

  int i0, i1;
};

static void quad_search_task(void *_u) {
  struct quad_search_task *task = (struct quad_search_task *)_u;

  struct quad quad;
  memset(&quad, 0, sizeof(struct quad));

  for (int i = task->i0; i < task->i1; i++) {
    struct segment *seg;
    zarray_get_volatile(task->segments, i, &seg);

    quad.segments[0] = seg;

    quad_search(task->quads, task->td, 0, &quad);
  }
}

struct quad_decode_task {
  int i0, i1;
  zarray_t *quads;
  april_tag_detector_t *td;

  image_u8_t *im;
  zarray_t *detections;

  image_u8_t *im_gray_samples;
  image_u8_t *im_decision;
};

// compute an approximate "radius" (in pixels) of how big the tag was.
static inline double detection_radius(april_tag_detection_t *det) {
  double r = 0;
  for (int i = 0; i < 4; i++) {
    double ri =
        sqrt(sq(det->c[0] - det->p[i][0]) + sq(det->c[1] - det->p[i][1]));
    r += ri;
  }
  return r / 4.0;
}

// @param families is a list of april_tag_family, all of which must
// have the same bit size and border. (I.e., they are "compatible"
// geometrically; they just decode differently).
april_tag_detection_t *quad_decode_real(april_tag_detector_t *td,
                                        april_tag_family_t *family,
                                        image_u8_t *im, struct quad *quad,
                                        struct quad_decode_task *task,
                                        uint32_t guessid) {
  april_tag_detection_t *det = NULL;
  zarray_t *correspondences = zarray_create(sizeof(float[4]));
  int width = im->width, height = im->height;

  for (int i = 0; i < 4; i++) {
    float corr[4];

    corr[0] = (i == 0 || i == 3) ? -1 : 1;
    corr[1] = (i == 0 || i == 1) ? -1 : 1;
    corr[2] = quad->p[i][0];
    corr[3] = quad->p[i][1];

    zarray_add(correspondences, &corr);
  }

  matd_t *H = homography_compute(correspondences);

  if (0) {
    matd_t *x = matd_create(3, 1);

    // homography testing code
    for (int i = 0; i < zarray_size(correspondences); i++) {
      float *corr;

      zarray_get_volatile(correspondences, i, &corr);
      MAT_EL(x, 0, 0) = corr[0];
      MAT_EL(x, 1, 0) = corr[1];
      MAT_EL(x, 2, 0) = 1;

      matd_t *y = matd_op("M*M", H, x);

      for (int j = 0; j < 3; j++) MAT_EL(y, j, 0) /= MAT_EL(y, 2, 0);

      matd_destroy(y);
    }

    matd_destroy(x);
  }

  // fit gray models
  graymodel_t *black_model = graymodel_create();
  graymodel_t *white_model = graymodel_create();

  int dd = 2 * family->black_border + family->d;

  // we'll need this when we read off the bits.
  // This is declared here so that the "goto cleanup" does not
  // jump into scope.

  float softbits[family->d * family->d];

  for (int iy = -1; iy <= dd; iy++) {
    for (int ix = -1; ix <= dd; ix++) {
      float x = 2 * (ix + .5) / dd - 1;
      float y = 2 * (iy + .5) / dd - 1;

      double px, py;
      homography_project(H, x, y, &px, &py);
      int irx = (int)(px + .5);
      int iry = (int)(py + .5);

      if (irx < 0 || irx >= width || iry < 0 || iry >= height) continue;

      int v = im->buf[iry * im->stride + irx];

      if ((iy == -1 || iy == dd) || (ix == -1 || ix == dd)) {
        // part of the outer white border.
        graymodel_add_observation(white_model, x, y, v);

        if (task != NULL && task->im_gray_samples)
          task->im_gray_samples
              ->buf[iry * task->im_gray_samples->stride + irx] = 0;

      } else if ((iy == 0 || iy == (dd - 1)) || (ix == 0 || ix == (dd - 1))) {
        // part of the outer black border.
        graymodel_add_observation(black_model, x, y, v);

        if (task != NULL && task->im_gray_samples)
          task->im_gray_samples
              ->buf[iry * task->im_gray_samples->stride + irx] = 255;
      }
    }
  }

  graymodel_solve(white_model);
  graymodel_solve(black_model);

  float white_mean = MAT_EL(white_model->b, 3, 0) / white_model->n;
  float black_mean = MAT_EL(black_model->b, 3, 0) / black_model->n;

  if (white_mean < black_mean + td->min_white_black_diff) {
    //            printf("%15f %15f\n", black_mean, white_mean);
    goto cleanup;
  }

  // debugging output
  if (task != NULL && task->im_decision != NULL) {
    matd_t *Hinv = matd_inverse(H);

    double x0, y0, x1, y1, x2, y2, x3, y3;
    homography_project(H, -1, -1, &x0, &y0);
    homography_project(H, 1, -1, &x1, &y1);
    homography_project(H, -1, 1, &x2, &y2);
    homography_project(H, 1, 1, &x3, &y3);

    int ymin = imax(0, imin(imin(y0, y1), imin(y2, y3)) - 1);
    int ymax = imin(height - 1, imax(imax(y0, y1), imax(y2, y3)) + 1);

    int xmin = imax(0, imin(imin(x0, x1), imin(x2, x3)) - 1);
    int xmax = imin(width - 1, imax(imax(x0, x1), imax(x2, x3)) + 1);

    for (int y = ymin; y <= ymax; y++) {
      for (int x = xmin; x <= xmax; x++) {
        double px, py;
        homography_project(Hinv, x, y, &px, &py);

        if (px >= -1 && px <= 1 && py >= -1 && py <= 1) {

          double white = graymodel_interpolate(white_model, px, py);
          double black = graymodel_interpolate(black_model, px, py);

          int threshold = (int)(((white + black) / 2.0) + .5);

          if (threshold < 0) threshold = 0;
          if (threshold > 255) threshold = 255;
          task->im_decision->buf[y * task->im_decision->stride + x] = threshold;
        }
      }
    }

    matd_destroy(Hinv);
  }

  uint64_t tag_code = 0;

  // XXX Could add some soft decision logic here, so that we can
  // treat bad bits as erasures rather than get them wrong.
  // Basic idea: use white/black model to determine which
  // samples looked bad, and convert them to erasures.

  for (int iy = 0; iy < family->d; iy++) {
    for (int ix = 0; ix < family->d; ix++) {
      double y = 2 * (family->black_border + iy + .5) / dd - 1;
      double x = 2 * (family->black_border + ix + .5) / dd - 1;

      double px, py;
      homography_project(H, x, y, &px, &py);

      int irx = (int)(px + .5);
      int iry = (int)(py + .5);

      int bit = 0;

      if (irx >= 0 && irx < width && iry >= 0 && iry < height) {

        double white = graymodel_interpolate(white_model, x, y);
        double black = graymodel_interpolate(black_model, x, y);

        double threshold = ((white + black) / 2.0);

        int v = im->buf[iry * im->stride + irx];

        softbits[iy * family->d + ix] = v - threshold;
        bit = (v > threshold);
      } else {
        // Treat this as an erasure.
        softbits[iy * family->d + ix] = 0;
      }

      tag_code = (tag_code << 1) | bit;
    }
  }

  uint32_t id;
  uint32_t hamming;
  uint32_t rotation;
  uint64_t rotatedcode;

  // XXX update this so that it uses softbits.
  if (1)  // for (int tfidx = 0; tfidx < zarray_size(families); tfidx++) {
  {

    if (decode_tag(family, tag_code, &id, &hamming, &rotation, &rotatedcode,
                   guessid)) {

      float soft_goodness = 0;

      // score the softbits
      int nbits = family->d * family->d;

      for (int idx = nbits - 1; idx >= 0; idx--) {
        int bit = rotatedcode & 1;
        rotatedcode >>= 1;
        soft_goodness += softbits[idx] * (bit ? 1 : -1);
      }

      det = calloc(1, sizeof(april_tag_detection_t));
      det->family = family;
      det->id = id;
      det->hamming = hamming;
      det->goodness = soft_goodness / nbits;

      double theta = -rotation * PI / 2.0;
      double c = cos(theta), s = sin(theta);

      matd_t *R = matd_create(3, 3);
      MAT_EL(R, 0, 0) = c;
      MAT_EL(R, 0, 1) = -s;
      MAT_EL(R, 1, 0) = s;
      MAT_EL(R, 1, 1) = c;
      MAT_EL(R, 2, 2) = 1;

      det->H = matd_op("M*M", H, R);

      matd_destroy(R);

      homography_project(det->H, 0, 0, &det->c[0], &det->c[1]);

      // adjust the points in det->p so that they correspond to
      // counter-clockwise around the quad, starting at -1,-1.
      for (int i = 0; i < 4; i++) {
        int tcx = (i == 0 || i == 3) ? -1 : 1;
        int tcy = (i < 2) ? -1 : 1;

        double p[2];

        homography_project(det->H, tcx, tcy, &p[0], &p[1]);

        det->p[i][0] = p[0];
        det->p[i][1] = p[1];
      }
    }
  }

// clean up
cleanup:

  graymodel_destroy(white_model);
  graymodel_destroy(black_model);
  zarray_destroy(correspondences);
  matd_destroy(H);

  return det;
}

void quad_decode(april_tag_detector_t *td, april_tag_family_t *family,
                 image_u8_t *im, struct quad *quad,
                 struct quad_decode_task *task) {
  april_tag_detection_t *det =
      quad_decode_real(td, family, im, quad, task, UINT32_MAX);

  if (td->small_tag_refinement) {

    // XXX Restrict this processing to small edges.

    while ((det == NULL || det->hamming > 0)) {
      int improved = 0;

      for (int edge = 0; edge < 4; edge++) {

        int i0 = edge;
        int i1 = (edge + 1) & 3;

        // direction of the edge...
        float dx = quad->p[i1][0] - quad->p[i0][0];
        float dy = quad->p[i1][1] - quad->p[i0][1];

        float mag = sqrt(dy * dy + dx * dx);
        dy /= mag;
        dx /= mag;

        for (int stepdir = 0; stepdir < 2; stepdir++) {

          for (float stepsz = .25; stepsz <= 4; stepsz *= 2) {

            float step = stepsz * (stepdir == 0 ? -1 : 1);

            // not needed to do a deep copy of segments... we only use the
            // quads.
            struct quad qcopy;
            memcpy(&qcopy, quad, sizeof(struct quad));

            // step perpendicular to (dx,dy) direction... (-dy, dx)
            qcopy.p[i0][0] -= dy * step;
            qcopy.p[i0][1] += dx * step;
            qcopy.p[i1][0] -= dy * step;
            qcopy.p[i1][1] += dx * step;

            april_tag_detection_t *thisdet =
                quad_decode_real(td, family, im, &qcopy, task,
                                 det == NULL ? UINT32_MAX : det->id);
            if (thisdet == NULL) continue;

            // is this detection better than the last one?
            if (det == NULL || thisdet->hamming < det->hamming ||
                (thisdet->hamming == det->hamming &&
                 thisdet->goodness > det->goodness)) {

              improved = 1;

              // not needed to do a deep copy of segments... we only use the
              // quads.
              memcpy(quad, &qcopy, sizeof(struct quad));

              if (det) april_tag_detection_destroy(det);

              det = thisdet;

            } else {
              april_tag_detection_destroy(thisdet);
            }
          }
        }
      }

      if (!improved) break;
    }
  }

  if (det != NULL) {
    pthread_mutex_lock(&td->mutex);
    zarray_add(task->detections, &det);
    pthread_mutex_unlock(&td->mutex);
  }
}

static void quad_decode_task(void *_u) {
  struct quad_decode_task *task = (struct quad_decode_task *)_u;
  april_tag_detector_t *td = task->td;
  image_u8_t *im = task->im;

  for (int quadidx = task->i0; quadidx < task->i1; quadidx++) {
    struct quad *quad;
    zarray_get_volatile(task->quads, quadidx, &quad);

    for (int i = 0; i < zarray_size(td->tag_families); i++) {
      april_tag_family_t *family;
      zarray_get(td->tag_families, i, &family);

      struct quad quad_copy;
      memcpy(&quad_copy, quad, sizeof(struct quad));

      quad_decode(td, family, im, &quad_copy, task);
    }
  }
}

void april_tag_detection_destroy(april_tag_detection_t *det) {
  if (det == NULL) return;

  matd_destroy(det->H);
  free(det);
}

zarray_t *april_tag_detector_detect(april_tag_detector_t *td,
                                    image_u8_t *im_orig) {
  if (zarray_size(td->tag_families) == 0) {
    zarray_t *s = zarray_create(sizeof(april_tag_detection_t *));
    printf("apriltag.c: No tag families enabled.");
    return s;
  }

  if (td->wp == NULL || td->nthreads != workerpool_get_nthreads(td->wp)) {
    workerpool_destroy(td->wp);
    td->wp = workerpool_create(td->nthreads);
  }

  timeprofile_clear(td->tp);
  timeprofile_stamp(td->tp, "init");

  ///////////////////////////////////////////////////////////
  // Phase A. Segment the input image.
  zarray_t *segments;

  image_u8_t *im_seg = im_orig;

  if (td->seg_decimate > 1) {

    im_seg = image_u8_decimate(im_orig, td->seg_decimate);

    timeprofile_stamp(td->tp, "decimate");
  }

  if (td->seg_sigma != 0) {
    // compute a reasonable kernel width by figuring that the
    // kernel should go out 2 std devs.
    //
    // max sigma          ksz
    // 0.499              1  (disabled)
    // 0.999              3
    // 1.499              5
    // 1.999              7

    float sigma = fabs(td->seg_sigma);

    int ksz = 4 * sigma;  // 2 std devs in each direction
    if ((ksz & 1) == 0) ksz++;

    if (ksz > 1) {
      if (td->seg_sigma > 0) {
        // Apply a blur
        image_u8_gaussian_blur(im_seg, sigma, ksz);
      } else {
        // SHARPEN the image by subtracting the low frequency components.
        image_u8_t *orig = image_u8_copy(im_seg);
        image_u8_gaussian_blur(im_seg, sigma, ksz);

        for (int y = 0; y < orig->height; y++) {
          for (int x = 0; x < orig->width; x++) {
            int vorig = orig->buf[y * orig->stride + x];
            int vblur = im_seg->buf[y * im_seg->stride + x];

            int v = 2 * vorig - vblur;
            if (v < 0) v = 0;
            if (v > 255) v = 255;

            im_seg->buf[y * im_seg->stride + x] = (uint8_t)v;
          }
        }
        image_u8_destroy(orig);
      }
    }
  }
  timeprofile_stamp(td->tp, "blur");

  if (td->debug) image_u8_write_pgm(im_seg, "debug_blur.pnm");

  // segment the image. Lines should be fit assuming that 0,0 is
  // the lower-left corner of the first pixel; the center of the
  // pixel is at .5, .5.
  segments = segment2(td, im_seg);

  // adjust centers of pixels so that they correspond to the
  // original full-resolution image.
  if (td->seg_decimate > 1) {
    for (int i = 0; i < zarray_size(segments); i++) {
      struct segment *seg;
      zarray_get_volatile(segments, i, &seg);

      seg->p0[0] = seg->p0[0] * td->seg_decimate;
      seg->p0[1] = seg->p0[1] * td->seg_decimate;
      seg->p1[0] = seg->p1[0] * td->seg_decimate;
      seg->p1[1] = seg->p1[1] * td->seg_decimate;
    }
  }

  if (im_seg != im_orig) image_u8_destroy(im_seg);

  timeprofile_stamp(td->tp, "segment");

  ///////////////////////////////////////////////////////////
  // Step five. Loop over the clusters, fitting lines (which we
  // call Segments).
  zarray_t *detections = zarray_create(sizeof(april_tag_detection_t *));

  gridder_t *gridder = gridder_create(0, 0, im_orig->width, im_orig->height,
                                      (int)td->max_segment_distance);

  for (int i = 0; i < zarray_size(segments); i++) {
    struct segment *seg;

    zarray_get_volatile(segments, i, &seg);

    gridder_add(gridder, seg->p0[0], seg->p0[1], seg);
  }

  timeprofile_stamp(td->tp, "gridding");

  ///////////////////////////////////////////////////////////
  // Step six: populate the neighbor fields of each of our segments.
  if (1) {
    // want about ~10 tasks per thread.
    int chunksize = 1 + zarray_size(segments) /
                            (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);
    //       int chunksize = 10;

    struct neighbor_search_task tasks[zarray_size(segments) / chunksize + 1];
    int ntasks = 0;
    for (int i = 0; i < zarray_size(segments); i += chunksize) {
      tasks[ntasks].gridder = gridder;
      tasks[ntasks].segments = segments;
      tasks[ntasks].td = td;
      tasks[ntasks].i0 = i;
      tasks[ntasks].i1 = imin(zarray_size(segments), i + chunksize);

      workerpool_add_task(td->wp, neighbor_search_task, &tasks[ntasks]);
      ntasks++;
    }

    workerpool_run(td->wp);
  }

  gridder_destroy(gridder);

  timeprofile_stamp(td->tp, "neighbors");

  ////////////////////////////////////////////////////////////////
  // Step seven. Search all connected segments to see if any
  // form a loop of length 4. Add those to the quads list.
  zarray_t *quads = zarray_create(sizeof(struct quad));

  if (1) {
    int chunksize = 1 + zarray_size(segments) /
                            (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);
    //       int chunksize = 10;

    struct quad_search_task tasks[zarray_size(segments) / chunksize + 1];

    int ntasks = 0;
    for (int i = 0; i < zarray_size(segments); i += chunksize) {
      tasks[ntasks].segments = segments;
      tasks[ntasks].quads = quads;
      tasks[ntasks].td = td;
      tasks[ntasks].i0 = i;
      tasks[ntasks].i1 = imin(zarray_size(segments), i + chunksize);

      workerpool_add_task(td->wp, quad_search_task, &tasks[ntasks]);
      ntasks++;
    }

    workerpool_run(td->wp);

    //        printf("quads: %d tasks %.3f ms\n", ntasks, (b-a)/1000.0);
  }

  td->nquads = zarray_size(quads);

  timeprofile_stamp(td->tp, "quads");

  if (td->debug) {
    image_u8_t *im_quads = image_u8_copy(im_orig);
    image_u8_darken(im_quads);
    image_u8_darken(im_quads);

    if (im_quads != NULL) {
      for (int i = 0; i < zarray_size(quads); i++) {
        struct quad *quad;
        zarray_get_volatile(quads, i, &quad);

        const int bias = 100;
        int color = bias + (random() % (255 - bias));

        /*                for (int j = 0; j < 4; j++)
                          image_u8_draw_line(im_quads,
                          quad->segments[j]->p0[0], quad->segments[j]->p0[1],
                          quad->segments[j]->p1[0], quad->segments[j]->p1[1],
           200, 1);
                          */

        image_u8_draw_line(im_quads, quad->p[0][0], quad->p[0][1],
                           quad->p[1][0], quad->p[1][1], color, 1);
        image_u8_draw_line(im_quads, quad->p[1][0], quad->p[1][1],
                           quad->p[2][0], quad->p[2][1], color, 1);
        image_u8_draw_line(im_quads, quad->p[2][0], quad->p[2][1],
                           quad->p[3][0], quad->p[3][1], color, 1);
        image_u8_draw_line(im_quads, quad->p[3][0], quad->p[3][1],
                           quad->p[0][0], quad->p[0][1], color, 1);
      }

      image_u8_write_pgm(im_quads, "debug_quads.pnm");
      image_u8_destroy(im_quads);
    }
  }

  ////////////////////////////////////////////////////////////////
  // Step 7.5. Compute homographies for the quads so that we can
  // sample grid cells within the tag.
  if (1) {
    image_u8_t *im_gray_samples = td->debug ? image_u8_copy(im_orig) : NULL;

    // im_decision debugging output is slow.
    image_u8_t *im_decision = td->debug ? image_u8_copy(im_orig) : NULL;

    int chunksize = 1 + zarray_size(quads) /
                            (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);
    //       int chunksize = 5;

    struct quad_decode_task tasks[zarray_size(quads) / chunksize + 1];

    int ntasks = 0;
    for (int i = 0; i < zarray_size(quads); i += chunksize) {
      tasks[ntasks].i0 = i;
      tasks[ntasks].i1 = imin(zarray_size(quads), i + chunksize);
      tasks[ntasks].quads = quads;
      tasks[ntasks].td = td;
      tasks[ntasks].im = im_orig;
      tasks[ntasks].detections = detections;

      tasks[ntasks].im_gray_samples = im_gray_samples;
      tasks[ntasks].im_decision = im_decision;

      workerpool_add_task(td->wp, quad_decode_task, &tasks[ntasks]);
      ntasks++;
    }

    workerpool_run(td->wp);

    if (im_gray_samples != NULL) {
      image_u8_write_pgm(im_gray_samples, "debug_gray_samples.pnm");
      image_u8_destroy(im_gray_samples);
    }

    if (im_decision != NULL) {
      image_u8_write_pgm(im_decision, "debug_decision.pnm");
      image_u8_destroy(im_decision);
    }
  }

  timeprofile_stamp(td->tp, "homography/decode");

  ////////////////////////////////////////////////////////////////
  // Step 8. Reconcile detections--- don't report the same tag more
  // than once.
  if (1) {
    zarray_t *poly0 = g2d_polygon_create_data((double[4][2]) {}, 4);
    zarray_t *poly1 = g2d_polygon_create_data((double[4][2]) {}, 4);

    /*
       printf("\n");
       for (int i = 0; i < zarray_size(detections); i++) {

       april_tag_detection_t *det;
       zarray_get(detections, i, &det);

       printf("pre-detection %3d: tag%dh%d_%04d, hamming %d, goodness %15f\n",
       i, det->family->d*det->family->d, det->family->h, det->id, det->hamming,
       det->goodness);
       }
       */

    for (int i0 = 0; i0 < zarray_size(detections); i0++) {

      april_tag_detection_t *det0;
      zarray_get(detections, i0, &det0);

      for (int k = 0; k < 4; k++) zarray_set(poly0, k, det0->p[k], NULL);

      for (int i1 = i0 + 1; i1 < zarray_size(detections); i1++) {

        april_tag_detection_t *det1;
        zarray_get(detections, i1, &det1);

        if (det0->id != det1->id || det0->family != det1->family) continue;

        for (int k = 0; k < 4; k++) zarray_set(poly1, k, det1->p[k], NULL);

        /*
           printf(" %d %d (%d) %d %d\n", i0, i1, zarray_size(detections),
           g2d_polygon_overlaps_polygon(poly0, poly1),
           g2d_polygon_overlaps_polygon(poly1, poly0));
           */

        if (g2d_polygon_overlaps_polygon(poly0, poly1)) {
          // the tags overlap. Delete one, keep the other.

          if (det0->hamming < det1->hamming ||
              (det0->hamming == det1->hamming &&
               det0->goodness > det1->goodness)) {
            // keep det0, destroy det1
            april_tag_detection_destroy(det1);
            zarray_remove_index(detections, i1, 1);
            i1--;  // retry the same index
            goto retry1;
          } else {
            // keep det1, destroy det0
            april_tag_detection_destroy(det0);
            zarray_remove_index(detections, i0, 1);
            i0--;  // retry the same index.
            goto retry0;
          }
        }

      retry1:
        ;
      }

    retry0:
      ;
    }

    zarray_destroy(poly0);
    zarray_destroy(poly1);
  }

  timeprofile_stamp(td->tp, "reconcile");

  ////////////////////////////////////////////////////////////////
  // Produce final debug output
  if (td->debug) {
    image_u8_t *darker = image_u8_copy(im_orig);
    image_u8_darken(darker);
    image_u8_darken(darker);

    image_u32_t *out = image_u32_create_from_u8(darker);
    for (int detidx = 0; detidx < zarray_size(detections); detidx++) {
      april_tag_detection_t *det;
      zarray_get(detections, detidx, &det);

      if (det->hamming > 2) continue;

      // d |----| c
      //   |    |
      //   |    |
      // a |----| b

      double a[2], b[2], c[2], d[2];

      homography_project(det->H, -1, -1, &a[0], &a[1]);
      homography_project(det->H, 1, -1, &b[0], &b[1]);
      homography_project(det->H, 1, 1, &c[0], &c[1]);
      homography_project(det->H, -1, 1, &d[0], &d[1]);

      float scale =
          ((float[]) {1.0, 0.8, 0.6, 0.4, 0.4, 0.4, 0.4})[det->hamming];

      image_u32_draw_line(out, a[0], a[1], b[0], b[1],
                          rgb_scale(0xff0000, scale), 3);
      image_u32_draw_line(out, a[0], a[1], d[0], d[1],
                          rgb_scale(0x00ff00, scale), 3);
      image_u32_draw_line(out, b[0], b[1], c[0], c[1],
                          rgb_scale(0xff88ff, scale), 3);
      image_u32_draw_line(out, d[0], d[1], c[0], c[1],
                          rgb_scale(0x88ffff, scale), 3);
    }

    image_u32_write_pnm(out, "debug_output.pnm");

    image_u8_destroy(darker);
    image_u32_destroy(out);
  }

  // deallocate
  for (int i = 0; i < zarray_size(segments); i++) {
    struct segment *seg;

    zarray_get_volatile(segments, i, &seg);
    zarray_destroy(seg->neighbors);
  }

  zarray_destroy(segments);
  zarray_destroy(quads);

  zarray_sort(detections, detection_compare_function);
  timeprofile_stamp(td->tp, "cleanup");

  return detections;
}
