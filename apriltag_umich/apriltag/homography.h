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

#ifndef _HOMOGRAPHY_H
#define _HOMOGRAPHY_H

// correspondences is a list of float[4]s, consisting of the points x
// and y concatenated. We will compute a homography such that y = Hx
matd_t *homography_compute(zarray_t *correspondences);

void homography_project(const matd_t *H, double x, double y, double *ox, double *oy);

matd_t *homography_to_pose(const matd_t *H, double fx, double fy, double cx, double cy);
matd_t *homography_to_model_view(const matd_t *H, double F, double G, double A, double B, double C, double D);

/*
 static inline void homography_project01(const matd_t *H, double x, double y, double *ox, double *oy)
{
    homography_project(H, 2*x - 1, 2*y - 1, ox, oy);
}
*/
#endif
