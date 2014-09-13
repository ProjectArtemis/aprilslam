#include "apriltag_ros/detector/detector_node.h"

#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <apriltag_ros/Apriltags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <AprilTags/Tag36h11.h>

namespace apriltag_ros {

DetectorNode::DetectorNode(const ros::NodeHandle &nh)
    : nh_(nh),
      it_(nh),
      sub_camera_(
          it_.subscribeCamera("image_raw", 1, &DetectorNode::CameraCb, this)),
      pub_apriltags_(nh_.advertise<apriltag_ros::Apriltags>("apriltags", 1)),
      tag_detector_(AprilTags::tagCodes36h11),
      cam_calibrated_(true) {
  /*
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&DetectorNode::ConnectCb, this);
  pub_apriltags_ = nh_.advertise<apriltag_ros::Apriltags>(
      "apriltags", 1, connect_cb, connect_cb);
  */
}

void DetectorNode::ConnectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (!pub_apriltags_.getNumSubscribers())
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
    sub_camera_ = it_.subscribeCamera("image_raw", 2, &DetectorNode::CameraCb,
                                      this, hints);
  }
}

void DetectorNode::CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                            const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  // Only show detection if camera is not calibrated
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->height == 0) {
    ROS_ERROR_THROTTLE(1, "%s: %s", nh_.getNamespace().c_str(),
                       "camera not calibrate");
    cam_calibrated_ = false;
  }

  // Retrieve camera info and image
  model_.fromCameraInfo(cinfo_msg);
  cv::Mat image = cv_bridge::toCvCopy(
                      image_msg, sensor_msgs::image_encodings::MONO8)->image;
  cv::Mat color;
  cv::cvtColor(image, color, CV_GRAY2BGR);

  // Detect tags
  std::vector<AprilTags::TagDetection> detections =
      tag_detector_.extractTags(image);

  // Process detection
  if (!detections.empty()) {
    ApriltagsPtr tags_msg(new Apriltags);
    tags_msg->header = image_msg->header;
    // Actual processing
    std::for_each(begin(detections), end(detections),
                  [&](const AprilTags::TagDetection &detection) {
      tags_msg->apriltags.push_back(DetectionToApriltagMsg(detection));
      detection.draw(color);
    });
    pub_apriltags_.publish(tags_msg);
  }

  // Display
  cv::imshow("image", color);
  cv::waitKey(1);
}

Apriltag DetectionToApriltagMsg(const AprilTags::TagDetection &detection) {
  Apriltag tag;
  tag.id = detection.id;
  tag.family = std::string("36h11");
  tag.hamming_distance = detection.hammingDistance;
  tag.center.x = detection.cxy.first;
  tag.center.y = detection.cxy.second;
  std::for_each(begin(detection.p), end(detection.p),
                [&](const std::pair<float, float> &corner) {
    geometry_msgs::Point point;
    point.x = corner.first;
    point.y = corner.second;
    tag.corners.push_back(point);
  });
  return tag;
}

}  // namespace apriltag_ros
