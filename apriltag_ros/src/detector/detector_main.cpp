#include "apriltag_ros/detector_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    // Hard code tag size now
    apriltag_ros::DetectorNode detector_node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
