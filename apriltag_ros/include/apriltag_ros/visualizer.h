#ifndef APRILTAG_ROS_VISUALIZER_H_
#define APRILTAG_ROS_VISUALIZER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <apriltag_ros/Apriltag.h>
#include <apriltag_ros/Apriltags.h>

namespace apriltag_ros {

namespace colors {
const std::vector<double> RED = {1, 0, 0};
const std::vector<double> GREEN = {0, 1, 0};
const std::vector<double> BLUE = {0, 0, 1};
const std::vector<double> CYAN = {0, 1, 1};
const std::vector<double> MAGENTA = {1, 0, 1};
const std::vector<double> YELLOW = {1, 1, 0};
}

class ApriltagVisualizer {
 public:
  ApriltagVisualizer(const ros::NodeHandle& nh)
      : nh_(nh),
        pub_markers_(nh_.advertise<visualization_msgs::MarkerArray>(
            "apriltags_marker", 1)) {
    set_alpha(1);
  }

  void set_colorRGB(const std::vector<double>& color) {
    color_.r = color[0];
    color_.g = color[1];
    color_.b = color[2];
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
