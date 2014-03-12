#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

int main(int argc, char** argv) {
  cv::VideoCapture cap(0); // open the default camera
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  if(!cap.isOpened())  // check if we succeeded
    return -1;


  cv::namedWindow("camera", 1);
  for(;;) {
    cv::Mat image;
    cv::Mat image_gray;
    cap.read(image); // get a new frame from camera
    cv::cvtColor(image, image_gray, CV_BGR2GRAY); // convert rgb to gray

    // detect April tags
    AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);
    vector<AprilTags::TagDetection> detections = tag_detector.extractTags(image_gray);
    //std::cout << detections.size() << "tags decteted: " << std::endl;

    for (int i = 0; i < detections.size(); ++i)
      detections[i].draw(image);
    cv::imshow("camera", image); // display frame

    if(cv::waitKey(10) >= 0)
      break;
  }

  return 0;
}
