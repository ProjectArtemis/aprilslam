extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/image_u8.h"
#include "apriltag/tag36h11.h"
#include "apriltag/zarray.h"
}
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// Detect April tags
april_tag_family_t *tf = tag36h11_create();
april_tag_detector_t *td = april_tag_detector_create(tf);

int main(int argc, char **argv) {
  cv::VideoCapture cap(0);  // open the default camera
  sleep(1);
  // cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  if (!cap.isOpened())  // check if we succeeded
    return -1;

  cv::namedWindow("video", 1);
  for (;;) {
    cv::Mat image;
    cv::Mat image_gray;
    cap.read(image);  // get a new frame from camera
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);  // convert rgb to gray

    image_u8_t *im = image_u8_create_from_gray(image_gray.cols, image_gray.rows,
                                               image_gray.data);

    zarray_t *detections = april_tag_detector_detect(td, im);

    for (int i = 0; i < zarray_size(detections); i++) {
      april_tag_detection_t *det;
      zarray_get(detections, i, &det);

      // Plot detection
      cv::line(image, cv::Point2f(det->p[0][0], det->p[0][1]),
               cv::Point2f(det->p[1][0], det->p[1][1]),
               cv::Scalar(255, 0, 0, 0));
      cv::line(image, cv::Point2f(det->p[1][0], det->p[1][1]),
               cv::Point2f(det->p[2][0], det->p[2][1]),
               cv::Scalar(0, 255, 0, 0));
      cv::line(image, cv::Point2f(det->p[2][0], det->p[2][1]),
               cv::Point2f(det->p[3][0], det->p[3][1]),
               cv::Scalar(0, 0, 255, 0));
      cv::line(image, cv::Point2f(det->p[3][0], det->p[3][1]),
               cv::Point2f(det->p[0][0], det->p[0][1]),
               cv::Scalar(255, 0, 255, 0));
      std::ostringstream str;
      str << det->id;
      cv::putText(image, str.str(), cv::Point2f(det->c[0] - 5, det->c[1] + 5),
                  cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2);
      april_tag_detection_destroy(det);
    }

    zarray_destroy(detections);
    image_u8_destroy(im);

    cv::imshow("video", image);  // display frame

    if (cv::waitKey(30) >= 0) break;
  }

  return 0;
}
