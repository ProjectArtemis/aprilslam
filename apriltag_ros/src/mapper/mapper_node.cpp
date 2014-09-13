#include "apriltag_ros/mapper_node.h"

namespace apriltag_ros {

void MapperNode::TagsCb(const apriltag_ros::ApriltagsConstPtr& tags_msg) {
  // Do nothing if no detection, this prevents checking in the following steps
  if (tags_msg->apriltags.empty()) {
    ROS_WARN_THROTTLE(1, "No tags detected.");
    return;
  }
  // Also do nothing if camera info not received
  if (!model_.initialized()) {
    ROS_WARN_THROTTLE(1, "No cinfo received");
    return;
  }
  // Set child frame id for tf to be the same as camera frame id
  if (pose_viz_.child_frame_id().empty()) {
    pose_viz_.set_child_frame_id(tags_msg->header.frame_id);
    ROS_INFO("Set child frame id to: %s.", pose_viz_.child_frame_id().c_str());
  }
  // Initialize map by adding the first tag that is not on the edge of the image
  if (!map_.init()) {
    if (!map_.AddFirstTag(*tags_msg, model_.cameraInfo().width,
                          model_.cameraInfo().height)) {
      return;
    }
    ROS_INFO("TagMap initialized.");
  }

  /*
  mapper_.AddPose();
  mapper_.AddLandmarks();
  mapper_.AddFactors();
  if (mapper_.init()) {
    mapper_.Update(1);
  } else {
    mapper_.FixScale();
  }
  */
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
