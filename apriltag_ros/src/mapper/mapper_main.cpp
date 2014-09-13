#include "apriltag_ros/mapper_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;

  try {
    // Hard code tag size now
    apriltag_ros::MapperNode mapper_node(nh, "world");
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
