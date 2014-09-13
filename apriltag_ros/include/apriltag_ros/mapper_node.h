#ifndef APRILTAG_ROS_MAPPER_NODE_H_
#define APRILTAG_ROS_MAPPER_NODE_H_

#include <ros/ros.h>
#include <apriltag_ros/Apriltags.h>

namespace apriltag_ros {

class Tagmap {

};

class MapperNode {
 public:
  MapperNode(const ros::NodeHandle& nh)
      : nh_(nh),
        sub_tags_(nh_.subscribe("apriltags", 1, &MapperNode::TagsCb, this)) {}

 private:
  void TagsCb(const apriltag_ros::ApriltagsConstPtr& tags_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_tags_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_MAPPER_NODE_H_
