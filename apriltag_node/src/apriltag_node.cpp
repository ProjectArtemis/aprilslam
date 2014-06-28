#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <apriltag_node/Tag.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
using std::vector;
#include <map>
#include <iostream>
using std::cout;
using std::endl;

#define BUILD_MIT

#if defined(BUILD_UMICH)
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/image_u8.h"
#include "apriltag/tag36h11.h"
#include "apriltag/zarray.h"
}
#elif defined(BUILD_MIT)
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#endif

typedef cv::Point2d Point2;
typedef cv::Point3d Point3;

typedef struct tag {
  Point2 p[4];
} tag_t;

static std::map<int, tag> tag_w;

const cv::Scalar colors[] = { cv::Scalar(255, 0, 0, 0),
                              cv::Scalar(0, 255, 0, 0),
                              cv::Scalar(0, 0, 255, 0),
                              cv::Scalar(255, 0, 255, 0) };

void cam_callback(const sensor_msgs::ImageConstPtr &image,
                  const sensor_msgs::CameraInfoConstPtr $cinfo) {
  // use cv_bridge and convert to grayscale image
  cv_bridge::CvImagePtr cv_ptr;
  // use toCvCopy because we will modify the image
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);

  cv::Mat image_rgb;
  cv::cvtColor(cv_ptr->image, image_rgb, CV_GRAY2RGB);

#if defined(BUILD_UMICH)
  // use apriltag_umich
  static april_tag_family_t *tf = tag36h11_create();
  static april_tag_detector_t *td = april_tag_detector_create(tf);

  image_u8_t *im = image_u8_create_from_gray(
      cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.data);
  zarray_t *detections = april_tag_detector_detect(td, im);

  ROS_INFO("Tags detected: %d", zarray_size(detections));

  for (size_t i = 0; i < zarray_size(detections); i++) {
    april_tag_detection_t *det;
    zarray_get(detections, i, &det);

    for (int j = 0; j < 4; j++) {
      const Point2 p = Point2(det->p[j][0], det->p[j][1]);
    }
    april_tag_detection_destroy(det);
  }

  zarray_destroy(detections);
  image_u8_destroy(im);

#elif defined(BUILD_MIT)
  // Use apriltag_mit
  static AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);
  std::vector<AprilTags::TagDetection> detections =
      tag_detector.extractTags(cv_ptr->image);

  // Check detection size
  if (detections.size()) {
    vector<Point2> pi; // Points in image
    vector<Point3> pw; // Points in world
    for (auto it = detections.begin(); it != detections.end(); it++) {
      for (int j = 0; j < 4; j++) {
        const int id = it->id;
        const Point2 p2 = Point2(it->p[j].first, it->p[j].second);
        pi.push_back(p2);
        Point3 p3(tag_w[id].p[j].x, tag_w[id].p[j].y, 0.0);
        pw.push_back(p3);
      }
    }

    cv::mat r;
    cv::Mat t;
    cv::Mat R;
  }
#endif

  cv::imshow("image", image_rgb);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::CameraSubscriber camera_sub =
      it.subscribeCamera("image_raw", 1, cam_callback);

  // output for OpenCV
  cv::namedWindow("image", 1);

  // Initialize simple test tag position
  double tag_size = 4.5 / 100;
  double tag_center[4][2] = { { tag_size, tag_size },
                              { tag_size, tag_size * 2 },
                              { tag_size * 2, tag_size },
                              { tag_size * 2, tag_size * 2 } };

  for (int i = 0; i < 4; ++i) {
    double x = tag_center[i][0];
    double y = tag_center[i][1];
    tag_w[i].p[0] = Point2(x - tag_size / 2, y - tag_size / 2);
    tag_w[i].p[1] = Point2(x + tag_size / 2, y - tag_size / 2);
    tag_w[i].p[2] = Point2(x + tag_size / 2, y + tag_size / 2);
    tag_w[i].p[3] = Point2(x - tag_size / 2, y + tag_size / 2);
  }

  ros::spin();

  return 0;
}
