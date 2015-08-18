#ifndef APRILSLAM_MAPPER_NODE_H_
#define APRILSLAM_MAPPER_NODE_H_

#include <ros/ros.h>
#include <aprilslam/Apriltags.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include "aprilslam/visualizer.h"
#include <tf2_ros/transform_broadcaster.h>

#include "aprilslam/mapper.h"
#include "aprilslam/tag_map.h"

namespace aprilslam {

class MapperNode {
 public:
  MapperNode(const ros::NodeHandle& nh, const std::string& frame_id)
      : nh_(nh),
        sub_tags_(nh_.subscribe("apriltags", 1, &MapperNode::TagsCb, this)),
        sub_cinfo_(nh_.subscribe("camera_info", 1, &MapperNode::CinfoCb, this)),
        frame_id_(frame_id),
        mapper_(0.04, 1),
        tag_viz_(nh, "apriltags_map") {
    		tag_viz_.SetColor(aprilslam::GREEN);
    		tag_viz_.SetAlpha(0.75);
  	}

  bool GetGoodTags(const std::vector<aprilslam::Apriltag> tags_c,
                   std::vector<aprilslam::Apriltag>* tags_c_good);

 private:
  void TagsCb(const aprilslam::ApriltagsConstPtr& tags_c_msg);
  void CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_tags_;
  ros::Subscriber sub_cinfo_;
  std::string frame_id_;
  aprilslam::TagMap map_;
  aprilslam::Mapper mapper_;
  aprilslam::ApriltagVisualizer tag_viz_;
  image_geometry::PinholeCameraModel model_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

};

}  // namespace aprilslam

#endif  // APRILSLAM_MAPPER_NODE_H_
