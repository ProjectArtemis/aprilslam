#include "apriltag_ros/tag_map.h"
#include "apriltag_ros/utils.h"

#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>
namespace apriltag_ros {

void TagMap::AddOrUpdate(int id, double size, const geometry_msgs::Pose &pose) {
  auto it = FindById(id, tags_msg_.apriltags);
  if (it == tags_w().end()) {
    // Not in map, add to map
    ROS_INFO_STREAM(id << " not in map");
  } else {
    // Already in map update
    ROS_INFO_STREAM(id << "  in map");
  }
}

void TagMap::AddTag(const Apriltag &tag_c, const geometry_msgs::Pose &pose) {
  Apriltag tag_w = tag_c;
  tag_w.pose = pose;
  SetCorners(&tag_w.corners, tag_w.pose, tag_w.size);
  tag_w.center = tag_w.pose.position;
  tags_msg_.apriltags.push_back(tag_w);
  ROS_INFO("tag %d added to map", tag_c.id);
}

bool TagMap::AddFirstTag(const Apriltags &tags_c, int width, int height) {
  for (const Apriltag &tag_c : tags_c.apriltags) {
    if (IsInsideImageCenter(tag_c.center.x, tag_c.center.y, width, height, 3)) {
      // This magic number 3 means the border is 1/3 of the size
      tags_msg_.header.stamp = tags_c.header.stamp;
      // Creat tag in world frame and set to origin
      // Set the first tag to origin
      geometry_msgs::Pose pose;
      SetPose(&pose);
      AddTag(tag_c, pose);
      ROS_INFO("tag %d: %f, %f", tag_c.id, tag_c.center.x, tag_c.center.y);
      return true;
    }
  }
  // No good tag detected around the center of image
  return false;
}

bool TagMap::EstimatePose(const std::vector<Apriltag> &tags_c,
                          const cv::Matx33d &K, const cv::Mat_<double> &D,
                          geometry_msgs::Pose *pose) const {
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;

  for (const Apriltag &tag_c : tags_c) {
    // Find 2D-3D correspondences
    auto it = FindById(tag_c.id, tags_w());
    if (it != tags_w().cend()) {
      const Apriltag &tag_w = *it;
      std::for_each(tag_w.corners.begin(), tag_w.corners.end(),
                    [&obj_pts](const geometry_msgs::Point &p_w) {
        obj_pts.emplace_back(p_w.x, p_w.y, p_w.z);
      });
      std::for_each(tag_c.corners.begin(), tag_c.corners.end(),
                    [&img_pts](const geometry_msgs::Point &p_c) {
        img_pts.emplace_back(p_c.x, p_c.y);
      });
    }
  }

  ROS_ASSERT_MSG(img_pts.size() == obj_pts.size(), "size mismatch!");
  if (img_pts.empty()) return false;
  // Actual pose estimation work here
  cv::Mat c_r_w, c_t_w, c_R_w;
  cv::solvePnP(obj_pts, img_pts, K, D, c_r_w, c_t_w);
  cv::Rodrigues(c_r_w, c_R_w);
  cv::Mat c_T_w(c_t_w);
  cv::Mat w_R_c(c_R_w.t());
  cv::Mat w_T_c = -w_R_c * c_T_w;

  double *pt = w_T_c.ptr<double>();
  SetPosition(&pose->position, pt[0], pt[1], pt[2]);

  Eigen::Quaterniond w_Q_c = RodriguesToQuat(c_r_w).inverse();
  SetOrientation(&pose->orientation, w_Q_c);
  return true;
}

std::vector<Apriltag>::const_iterator FindById(
    int id, const std::vector<Apriltag> &tags) {
  return std::find_if(tags.cbegin(), tags.cend(),
                      [&id](const Apriltag &tag) { return id == tag.id; });
}

std::vector<Apriltag>::iterator FindById(int id, std::vector<Apriltag> &tags) {
  return std::find_if(tags.begin(), tags.end(),
                      [&id](const Apriltag &tag) { return id == tag.id; });
}

}  // namespace apriltag_ros
