#include "apriltag_ros/utils.h"
#include <sophus/se3.hpp>

namespace apriltag_ros {

bool IsInsideImageCenter(double x, double y, int w, int h, double k) {
  return (x > w / k) && (y > h / k) && (x < (w - w / k)) && (y < (h - h / k));
}

void SetPosition(geometry_msgs::Point* pos, double x, double y, double z) {
  pos->x = x;
  pos->y = y;
  pos->z = z;
}

void SetOrientation(geometry_msgs::Quaternion* quat, double w, double x,
                    double y, double z) {
  quat->w = w;
  quat->x = x;
  quat->y = y;
  quat->z = z;
}

void SetCorners(std::vector<geometry_msgs::Point>* corners,
                const geometry_msgs::Pose& pose, double tag_size) {
  std::vector<geometry_msgs::Point>& w_corners = *corners;
  double a = tag_size / 2;
  Sophus::SE3d w_H_b(
      Sophus::Quaterniond(pose.orientation.w, pose.orientation.x,
                          pose.orientation.y, pose.orientation.z),
      Sophus::SE3d::Point(pose.position.x, pose.position.y, pose.position.z));
  std::vector<Sophus::SE3d::Point> b_cs = {
      {-a, -a, 0}, {a, -a, 0}, {a, a, 0}, {-a, a, 0}};
  // Transform corners in body frame to world frame
  std::transform(b_cs.begin(), b_cs.end(), w_corners.begin(),
                 [&](Sophus::SE3d::Point& b_c) {
    const Sophus::SE3d::Point w_c = w_H_b * b_c;
    geometry_msgs::Point w_corner;
    w_corner.x = w_c(0);
    w_corner.y = w_c(1);
    w_corner.z = w_c(2);
    return w_corner;
  });
  // Nuke this raw loop version
  /*
  for (size_t i = 0; i < w_corners.size(); ++i) {
    const Sophus::SE3d::Point w_c = w_H_b * b_cs[i];
    w_corners[i].x = w_c(0);
    w_corners[i].y = w_c(1);
    w_corners[i].z = w_c(2);
  }
  */
}

}  // namespace apriltag_ros
