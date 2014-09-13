#include "apriltag_ros/mapper_node.h"

namespace apriltag_ros {

void MapperNode::TagsCb(const apriltag_ros::ApriltagsConstPtr& tags_msg) {
  ROS_INFO_THROTTLE(2, "In tags cb");
}

void MapperNode::CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  if (model_.initialized()) {
    sub_cinfo_.shutdown();
    ROS_INFO("%s: %s", nh_.getNamespace().c_str(), "Camera initialized");
    return;
  }
  model_.fromCameraInfo(cinfo_msg);
}

}  // namespace apriltag_ros
