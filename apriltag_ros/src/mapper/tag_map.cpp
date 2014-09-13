#include "apriltag_ros/tag_map.h"
#include "apriltag_ros/utils.h"

#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>
namespace apriltag_ros {

bool TagMap::AddFirstTag(const Apriltags &tags, int width, int height) {
  for (const Apriltag &tag : tags.apriltags) {
    if (IsInsideImageCenter(tag.center.x, tag.center.y, width, height, 3)) {
      // This magic number 3 means the border is 1/3 of the size
      set_first_tag_id(tag.id);
      // Creat tag in world frame
      Apriltag tag_world = tag;
      tags_.header.stamp = tags.header.stamp;
      // Set the first tag to origin
      SetPosition(&tag_world.pose.position);
      SetOrientation(&tag_world.pose.orientation);
      SetCorners(&tag_world.corners, tag_world.pose, tag.size);
      tag_world.center = tag_world.pose.position;
      // Add this tag to map
      tags_.apriltags.push_back(tag_world);
      ROS_INFO("tag %d: %f, %f", tag.id, tag.center.x, tag.center.y);
      return true;
    }
  }
  // No good tag detected around the center of image
  return false;
}

bool TagMap::EstimatePose(const Apriltags &tags, const cv::Matx33d &K,
                          const cv::Mat_<double> &D,
                          geometry_msgs::Pose *pose) const {
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;

  for (const Apriltag &tag_c : tags.apriltags) {
    auto it = std::find_if(
        tags_.apriltags.begin(), tags_.apriltags.end(),
        [&tag_c](const Apriltag &tag_w) { return tag_c.id == tag_w.id; });
    // Find 2D-3D correspondences
    if (it != tags_.apriltags.end()) {
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

}  // namespace apriltag_ros
