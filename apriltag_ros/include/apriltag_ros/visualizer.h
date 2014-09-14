#ifndef APRILTAG_ROS_VISUALIZER_H_
#define APRILTAG_ROS_VISUALIZER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz/helpers/color.h>
#include <rviz_helper/colors.h>
#include <apriltag_ros/Apriltag.h>
#include <apriltag_ros/Apriltags.h>

namespace apriltag_ros {

class ApriltagVisualizer {
 public:
  ApriltagVisualizer(const ros::NodeHandle& nh, const std::string& topic)
      : nh_(nh),
        pub_markers_(nh_.advertise<visualization_msgs::MarkerArray>(
            topic, 1)) {}

  void set_color(const rviz::Color& color) {
    color_.r = color.r_;
    color_.g = color.g_;
    color_.b = color.b_;
  }
  void set_alpha(double alpha) { color_.a = alpha; }

  void PublishApriltagsMarker(const apriltag_ros::Apriltags& apriltags);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_markers_;
  std_msgs::ColorRGBA color_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_VISUALIZER_H_
