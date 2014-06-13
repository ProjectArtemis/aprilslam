#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

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

const cv::Scalar colors[] = {cv::Scalar(255, 0, 0, 0), cv::Scalar(0, 255, 0, 0),
                             cv::Scalar(0, 0, 255, 0),
                             cv::Scalar(255, 0, 255, 0)};

class AprilTag {
 public:

 private:

};  // class AprilTag

void cam_callback(const sensor_msgs::ImageConstPtr &image,
                  const sensor_msgs::CameraInfoConstPtr $cinfo) {
  // use cv_bridge and convert to grayscale image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);

  std::vector<cv::Point2f> corners;

  cv::Mat image_rgb;
  cv::cvtColor(cv_ptr->image, image_rgb, CV_GRAY2RGB);

#if defined(BUILD_UMICH)
  // use apriltag_umich
  static april_tag_family_t *tf = tag36h11_create();
  static april_tag_detector_t *td = april_tag_detector_create(tf);

  image_u8_t *im = image_u8_create_from_gray(
      cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.data);
  zarray_t *detections = april_tag_detector_detect(td, im);

  for (size_t i = 0; i < zarray_size(detections); i++) {
    april_tag_detection_t *det;
    zarray_get(detections, i, &det);

    for (int j = 0; j < 4; j++) {
      const cv::Point2f p = cv::Point2f(det->p[j][0], det->p[j][1]);
      corners.push_back(p);
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

  for (std::vector<AprilTags::TagDetection>::iterator iter = detections.begin();
       iter != detections.end(); iter++) {
    for (int j = 0; j < 4; j++) {
      const cv::Point2f p = cv::Point2f(iter->p[j].first, iter->p[j].second);
      corners.push_back(p);
    }
  }

#endif

  cv::imshow("image", image_rgb);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // subscribe to camera
  image_transport::CameraSubscriber camera_sub =
      it.subscribeCamera("image_raw", 1, cam_callback);

  // output for OpenCV
  cv::namedWindow("image", 1);

  ros::spin();

  return 0;
}
