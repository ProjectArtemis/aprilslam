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
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>

#include "apriltag.h"
#include "image_u8.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "zarray.h"

// Invoke:
//
// tagtest [options] input.pnm

int main(int argc, char *argv[])
{
    april_tag_family_t *tf = tag36h11_create();

    april_tag_detector_t *td = april_tag_detector_create(tf);
    td->small_tag_refinement = 0;

    int maxiters = 1;

    zarray_t *inputs = zarray_create(sizeof(char*));
    int waitsec = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-d"))
            td->debug = 1;
        else if (!strcmp(argv[i], "-t"))
            td->nthreads = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-f"))
            td->seg_decimate = (i+1 < argc && isdigit(argv[i+1][0])) ? atof(argv[++i]) : 2;
        else if (!strcmp(argv[i], "-i"))
            maxiters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-r"))
            td->small_tag_refinement = 1;
        else if (!strcmp(argv[i], "-w"))
            waitsec = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-b"))
            td->seg_sigma = atof(argv[++i]);
/*        else if (!strcmp(argv[i], "--family")) {
            char *fam = argv[++i];
            if (!strcmp(fam, "36h11"))
                td->tag_family = tag36h11_create();
            else if (!strcmp(fam, "36h10"))
                td->tag_family = tag36h10_create();
                } */
        else
            zarray_add(inputs, &argv[i]);
    }

    for (int iter = 0; iter < maxiters; iter++) {

        if (maxiters > 1)
            printf("iter %d / %d\n", iter + 1, maxiters);

        for (int input = 0; input < zarray_size(inputs); input++) {

            char *path;
            zarray_get(inputs, input, &path);
            printf("loading %s\n", path);
            image_u8_t *im = image_u8_create_from_pnm(path);
            if (im == NULL) {
                printf("couldn't find %s\n", path);
                continue;
            }

            zarray_t *detections = april_tag_detector_detect(td, im);

            for (int i = 0; i < zarray_size(detections); i++) {
                april_tag_detection_t *det;
                zarray_get(detections, i, &det);

                printf("detection %3d: id %4d, hamming %d, goodness %f\n", i, det->id, det->hamming, det->goodness);
                april_tag_detection_destroy(det);
            }

            zarray_destroy(detections);

            timeprofile_display(td->tp);
            printf("nedges: %d, nsegments: %d, nquads: %d\n", td->nedges, td->nsegments, td->nquads);

            image_u8_destroy(im);

            if (zarray_size(inputs) > 1 || iter > 0)
                sleep(waitsec);
        }
    }

    april_tag_detector_destroy(td);

    tag36h11_destroy(tf);
    return 0;
}
