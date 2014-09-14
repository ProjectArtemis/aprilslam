#ifndef APRILTAG_ROS_MAPPER_H_
#define APRILTAG_ROS_MAPPER_H_

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <geometry_msgs/Pose.h>
#include <apriltag_ros/Apriltags.h>

namespace apriltag_ros {

// Feel like using iSAM2?
class Mapper {
 public:
  static int pose_cnt;

  Mapper(double relinearize_thresh, int relinearize_skip);

  bool init() const { return init_; }
  void Update(int num_iterations = 1);
  void AddPose(const geometry_msgs::Pose& pose);
  void AddFactors(const apriltag_ros::Apriltags& tags);
  void AddLandmarks(const apriltag_ros::Apriltags& tags);
  void Initialize(int landmark_id);
  void Clear();

 private:
  void AddLandmark(int id, const gtsam::Pose3& pose);
  void AddFirstLandmark(int id);
  void AddPrior(int landmark_id);

  bool init_;
  gtsam::ISAM2Params params_;
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  gtsam::Pose3 pose_;
  gtsam::noiseModel::Diagonal::shared_ptr tag_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr small_noise_;
  std::set<int> all_ids_;
};

gtsam::Pose3 FromGeometryPose(const geometry_msgs::Pose& pose);
}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_MAPPER_H_
