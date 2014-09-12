#include "apriltag_ros/detector/detector_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "detector");
  ros::NodeHandle nh("~");

  try {
    apriltag_ros::DetectorNode detector_node(nh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
