#ifndef APRILSLAM_VISUALIZER_H_
#define APRILSLAM_VISUALIZER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz/helpers/color.h>
#include <aprilslam/Apriltag.h>
#include <aprilslam/Apriltags.h>

namespace aprilslam {

const rviz::Color RED = {1, 0, 0};
const rviz::Color GREEN = {0, 1, 0};
const rviz::Color BLUE = {0, 0, 1};
const rviz::Color CYAN = {0, 1, 1};
const rviz::Color MAGENTA = {1, 0, 1};
const rviz::Color YELLOW = {1, 1, 0};

class ApriltagVisualizer {
 public:
  ApriltagVisualizer(const ros::NodeHandle& nh, const std::string& topic)
      : nh_(nh),
        pub_markers_(nh_.advertise<visualization_msgs::MarkerArray>(topic, 1)) {
  }

  void SetColor(const rviz::Color& color) {
    color_.r = color.r_;
    color_.g = color.g_;
    color_.b = color.b_;
  }
  void SetAlpha(double alpha) { color_.a = alpha; }

  void PublishApriltagsMarker(const aprilslam::Apriltags& apriltags);
  void PublishApriltagsMarker(const std::vector<aprilslam::Apriltag>& tags,
                              const std::string& frame_id,
                              const ros::Time& stamp);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_markers_;
  std_msgs::ColorRGBA color_;
};

}  // namespace aprilslam

#endif  // APRILSLAM_VISUALIZER_H_
