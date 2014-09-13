#ifndef APRILTAG_ROS_VISUALIZER_H_
#define APRILTAG_ROS_VISUALIZER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <apriltag_ros/Apriltag.h>
#include <apriltag_ros/Apriltags.h>

namespace apriltag_ros {

class ApriltagVisualizer {
 public:
  ApriltagVisualizer(const ros::NodeHandle& nh)
      : nh_(nh),
        pub_markers_(nh_.advertise<visualization_msgs::MarkerArray>(
            "apriltags_marker", 1)) {}

  void PublishApriltagsMarker(const apriltag_ros::Apriltags& apriltags);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_markers_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_VISUALIZER_H_
