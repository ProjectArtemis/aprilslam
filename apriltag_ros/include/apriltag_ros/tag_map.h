#ifndef APRILTAG_ROS_TAG_MAP_H_
#define APRILTAG_ROS_TAG_MAP_H_

#include <apriltag_ros/Apriltags.h>
#include <opencv2/core/core.hpp>

namespace apriltag_ros {

class TagMap {
 public:
  TagMap(const std::string& frame_id) : frame_id_(frame_id), first_tag_id_(-1) {
    tags_.header.frame_id = frame_id_;
  }

  bool AddFirstTag(const Apriltags& tags, int width, int height);

  bool init() const { return first_tag_id_ >= 0; }
  int first_tag_id() const { return first_tag_id_; }
  void set_first_tag_id(int id) { first_tag_id_ = id; }
  const apriltag_ros::Apriltags& ToMsg() const { return tags_; }

 private:
  std::string frame_id_;
  int first_tag_id_;
  apriltag_ros::Apriltags tags_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_TAG_MAP_H_
