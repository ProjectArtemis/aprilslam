#include "apriltag_ros/mapper_node.h"
#include "apriltag_ros/utils.h"

namespace apriltag_ros {

void MapperNode::TagsCb(const apriltag_ros::ApriltagsConstPtr& tags_c_msg) {
  // Do nothing if no detection, this prevents checking in the following steps
  if (tags_c_msg->apriltags.empty()) {
    ROS_WARN_THROTTLE(1, "No tags detected.");
    return;
  }
  // Do nothing if camera info not received
  if (!model_.initialized()) {
    ROS_WARN_THROTTLE(1, "No cinfo received");
    return;
  }
  // Set child frame id for tf to be the same as camera frame id
  if (pose_viz_.child_frame_id().empty()) {
    pose_viz_.set_child_frame_id(tags_c_msg->header.frame_id);
    ROS_INFO("Set child frame id to: %s.", pose_viz_.child_frame_id().c_str());
  }
  // Do nothing if there are no good tags close to the center of the image
  std::vector<Apriltag> tags_c_good;
  if (!GetGoodTags(tags_c_msg->apriltags, &tags_c_good)) {
    ROS_WARN_THROTTLE(1, "No good tags detected.");
    return;
  }
  // Initialize map by adding the first tag that is not on the edge of the image
  if (!map_.init()) {
    map_.AddFirstTag(tags_c_good.front());
    ROS_INFO("TagMap initialized.");
  }
  // Do nothing if no pose can be estimated
  geometry_msgs::Pose pose;
  if (!map_.EstimatePose(tags_c_msg->apriltags, model_.fullIntrinsicMatrix(),
                         model_.distortionCoeffs(), &pose)) {
    ROS_WARN_THROTTLE(1, "No 2D-3D correspondence.");
    return;
  }
  // Now that with the initial pose calculated, we can do some mapping
  mapper_.AddPose(pose);
  mapper_.AddFactors(tags_c_good);
  if (mapper_.init()) {
    // This will only add new landmarks
    mapper_.AddLandmarks(tags_c_good);
    mapper_.Optimize();
    // Get latest estimates from mapper and put into map
    mapper_.Update(&map_, &pose);
    // Prepare for next iteration
    mapper_.Clear();
  } else {
    // This will add first landmark at origin and fix scale for first pose and
    // first landmark
    mapper_.Initialize(map_.first_tag());
  }
  // Publish updated pose and map
  pose_viz_.PublishPose(pose, frame_id_, tags_c_msg->header.stamp);
  tag_viz_.PublishApriltagsMarker(map_.tags_w(), frame_id_,
                                  tags_c_msg->header.stamp);
}

void MapperNode::CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  if (model_.initialized()) {
    sub_cinfo_.shutdown();
    ROS_INFO("%s: %s", nh_.getNamespace().c_str(), "Camera initialized");
    return;
  }
  model_.fromCameraInfo(cinfo_msg);
}

bool MapperNode::GetGoodTags(const std::vector<Apriltag> tags_c,
                             std::vector<Apriltag>* tags_c_good) {
  for (const Apriltag& tag_c : tags_c) {
    if (IsInsideImageCenter(tag_c.center.x, tag_c.center.y,
                            model_.cameraInfo().width,
                            model_.cameraInfo().height, 5)) {
      tags_c_good->push_back(tag_c);
    }
  }
  return !tags_c_good->empty();
}

}  // namespace apriltag_ros
