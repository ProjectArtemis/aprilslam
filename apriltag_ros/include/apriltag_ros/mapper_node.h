#ifndef APRILTAG_ROS_MAPPER_NODE_H_
#define APRILTAG_ROS_MAPPER_NODE_H_

#include <ros/ros.h>
#include <apriltag_ros/Apriltags.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include "apriltag_ros/mapper.h"

namespace apriltag_ros {

class Tagmap {};

class MapperNode {
 public:
  MapperNode(const ros::NodeHandle& nh)
      : nh_(nh),
        sub_tags_(nh_.subscribe("apriltags", 1, &MapperNode::TagsCb, this)),
        sub_cinfo_(nh_.subscribe("camera_info", 1, &MapperNode::CinfoCb, this)),
        mapper_(0.04, 1) {}

 private:
  void TagsCb(const apriltag_ros::ApriltagsConstPtr& tags_msg);
  void CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_tags_;
  ros::Subscriber sub_cinfo_;
  image_geometry::PinholeCameraModel model_;
  apriltag_ros::Mapper mapper_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_MAPPER_NODE_H_
