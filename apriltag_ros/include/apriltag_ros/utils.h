#ifndef APRILTAG_ROS_UTILS_H_
#define APRILTAG_ROS_UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

namespace apriltag_ros {

bool IsInsideImageCenter(double x, double y, int w, int h, double k);
void SetPosition(geometry_msgs::Point* pos, double x = 0, double y = 0,
                 double z = 0);
void SetOrientation(geometry_msgs::Quaternion* quat, double w = 1, double x = 0,
                    double y = 0, double z = 0);
void SetCorners(std::vector<geometry_msgs::Point>* corners,
                const geometry_msgs::Pose& pose, double tag_size);

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_UTILS_H_
