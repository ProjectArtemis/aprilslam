#ifndef APRILTAG_ROS_TAG_MAP_H_
#define APRILTAG_ROS_TAG_MAP_H_

#include <apriltag_ros/Apriltags.h>
#include <opencv2/core/core.hpp>
#include <set>

namespace apriltag_ros {

class TagMap {
 public:
  TagMap(const std::string& frame_id) { tags_msg_.header.frame_id = frame_id; }

  void AddOrUpdate(int id, double size, const geometry_msgs::Pose& pose);
  void AddTag(const Apriltag& tag_c, const geometry_msgs::Pose& pose);
  bool AddFirstTag(const Apriltags& tags_c, int width, int height);
  bool EstimatePose(const std::vector<Apriltag>& tags_c, const cv::Matx33d& K,
                    const cv::Mat_<double>& D, geometry_msgs::Pose* pose) const;

  bool init() const { return !tags_w().empty(); }
  const apriltag_ros::Apriltag& first_tag() const {
    assert(init());
    return tags_w().front();
  }
  const apriltag_ros::Apriltags& ToMsg() const { return tags_msg_; }
  const std::vector<apriltag_ros::Apriltag>& tags_w() const {
    return tags_msg_.apriltags;
  }

 private:
  apriltag_ros::Apriltags tags_msg_;
};

std::vector<apriltag_ros::Apriltag>::const_iterator FindById(
    int id, const std::vector<apriltag_ros::Apriltag>& tags);

std::vector<apriltag_ros::Apriltag>::iterator FindById(
    int id, std::vector<apriltag_ros::Apriltag>& tags);
}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_TAG_MAP_H_
