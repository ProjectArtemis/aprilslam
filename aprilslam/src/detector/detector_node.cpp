#include "aprilslam/detector_node.h"
#include "aprilslam/utils.h"

#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aprilslam/Apriltags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <AprilTags/Tag36h11.h>

namespace aprilslam {

DetectorNode::DetectorNode(const ros::NodeHandle &nh,
                           const ros::NodeHandle &pnh)
    : nh_(nh),
      tag_family_("36h11"),
      cam_calibrated_(true),
      it_(nh),
      sub_camera_(
          it_.subscribeCamera("image_rect", 1, &DetectorNode::CameraCb, this)),
      pub_tags_(nh_.advertise<aprilslam::Apriltags>("apriltags", 1)),
      pub_detections_(nh_.advertise<sensor_msgs::Image>("detections", 1)),
      tag_detector_(AprilTags::tagCodes36h11),
      tag_viz_(nh, "apriltags_marker") {

  if (!pnh.getParam("size", tag_size_)) {
    throw std::runtime_error("No tag size specified");
  }
  tag_viz_.SetColor(aprilslam::RED);
  tag_viz_.SetAlpha(0.75);
}

void DetectorNode::ConnectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (!pub_tags_.getNumSubscribers())
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
    sub_camera_ = it_.subscribeCamera("image_raw", 2, &DetectorNode::CameraCb,
                                      this, hints);
  }
}

void DetectorNode::CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                            const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  // Show only the detection if camera is uncalibrated
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->height == 0) {
    ROS_ERROR_THROTTLE(1, "%s: %s", nh_.getNamespace().c_str(),
                       "Camera not calibrated!");
    cam_calibrated_ = false;
  }
  // Retrieve camera info and image
  model_.fromCameraInfo(cinfo_msg);

	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat image;
	cv::cvtColor(cv_ptr->image, image, CV_BGR2GRAY);

  // Detect tags
  std::vector<AprilTags::TagDetection> detections =
      tag_detector_.extractTags(image);

  // Process detection
  if (!detections.empty()) {
    // Maybe use Ptr?
    Apriltags tags_c_msg;
    tags_c_msg.header = image_msg->header;
    // Actual processing
    std::for_each(begin(detections), end(detections),
                  [&](const AprilTags::TagDetection &detection) {
      tags_c_msg.apriltags.push_back(DetectionToApriltagMsg(detection));
      detection.draw(cv_ptr->image);
    });

    tag_viz_.PublishApriltagsMarker(tags_c_msg);
    pub_tags_.publish(tags_c_msg);
  }
	pub_detections_.publish(cv_ptr->toImageMsg());
}

Apriltag DetectorNode::DetectionToApriltagMsg(
    const AprilTags::TagDetection &detection) {
  Apriltag tag;
  // Gather basic information
  tag.id = detection.id;
  tag.family = tag_family_;
  tag.hamming_distance = detection.hammingDistance;
  tag.center.x = detection.cxy.first;
  tag.center.y = detection.cxy.second;
  tag.size = tag_size_;
  std::for_each(begin(detection.p), end(detection.p),   
                [&](const AprilTags::Pointf &corner) {
    geometry_msgs::Point point;
    point.x = corner.first;
    point.y = corner.second;
    tag.corners.push_back(point);
  });
  if (!cam_calibrated_) return tag;
  // Get rotation and translation of tag in camera frame, only if we have cinfo!
  Eigen::Quaterniond c_Q_b;
  Eigen::Vector3d c_T_b;
  /// @todo: Need to decide whether to undistort points here!
  detection.getRelativeQT(tag_size_, model_.fullIntrinsicMatrix(),
                          model_.distortionCoeffs(), c_Q_b, c_T_b);
  SetPose(&tag.pose, c_Q_b, c_T_b);
  return tag;
}

}  // namespace aprilslam
