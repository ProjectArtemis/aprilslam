#include "apriltag_ros/mapper.h"
#include "apriltag_ros/utils.h"
#include <gtsam/slam/PriorFactor.h>
#include <ros/ros.h>

namespace apriltag_ros {

using namespace gtsam;

int Mapper::pose_cnt = 0;

Mapper::Mapper(double relinearize_thresh, int relinearize_skip)
    : init_(false),
      params_(ISAM2GaussNewtonParams(), relinearize_thresh, relinearize_skip),
      isam2_(params_),
      tag_noise_(noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.05), Vector3::Constant(0.01)))),
      small_noise_(noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.05), Vector3::Constant(0.01)))) {}

void Mapper::AddPose(const geometry_msgs::Pose &pose) {
  pose_cnt++;
  pose_ = FromGeometryPose(pose);
  initial_estimates_.insert(Symbol('x', pose_cnt), pose_);
}

void Mapper::Initialize(const Apriltag &tag_w) {
  ROS_ASSERT_MSG(pose_cnt == 1, "Incorrect initial pose");
  AddFirstLandmark(tag_w.id, tag_w.size);
  AddPrior(tag_w.id);
  init_ = true;
}

void Mapper::AddPrior(int landmark_id) {
  // A very strong prior on first pose and landmark
  graph_.push_back(
      PriorFactor<Pose3>(Symbol('x', pose_cnt), pose_, small_noise_));
  graph_.push_back(
      PriorFactor<Pose3>(Symbol('l', landmark_id), Pose3(), small_noise_));
}

void Mapper::AddFirstLandmark(int id, double size) {
  AddLandmark(id, size, Pose3());
}

void Mapper::AddLandmark(int id, double size, const Pose3 &pose) {
  initial_estimates_.insert(Symbol('l', id), pose);
  all_ids_.insert(id);
  all_sizes_[id] = size;
}

void Mapper::AddLandmarks(const std::vector<Apriltag> &tags_c) {
  for (const Apriltag &tag_c : tags_c) {
    // Only add landmark if it's not already added
    if (all_ids_.find(tag_c.id) == all_ids_.end()) {
      const Pose3 &w_T_c = pose_;
      const Pose3 c_T_t = FromGeometryPose(tag_c.pose);
      const Pose3 w_T_t = w_T_c.compose(c_T_t);
      AddLandmark(tag_c.id, tag_c.size, w_T_t);
    }
  }
}

void Mapper::AddFactors(const std::vector<Apriltag> &tags_c) {
  Symbol x_i('x', pose_cnt);
  for (const Apriltag &tag_c : tags_c) {
    graph_.push_back(BetweenFactor<Pose3>(
        x_i, Symbol('l', tag_c.id), FromGeometryPose(tag_c.pose), tag_noise_));
  }
}

void Mapper::Optimize(int num_iterations) {
  isam2_.update(graph_, initial_estimates_);
  if (num_iterations > 1) {
    for (int i = 1; i < num_iterations; ++i) {
      isam2_.update();
    }
  }
}

void Mapper::Update(TagMap *map, geometry_msgs::Pose *pose) const {
  ROS_ASSERT_MSG(all_ids_.size() == all_sizes_.size(), "id and size mismatch");
  Values results = isam2_.calculateEstimate();
  // Update the current pose
  const Pose3 &cam_pose = results.at<Pose3>(Symbol('x', pose_cnt));
  SetPosition(&pose->position, cam_pose.x(), cam_pose.y(), cam_pose.z());
  SetOrientation(&pose->orientation, cam_pose.rotation().toQuaternion());
  // Update the current map
  for (const int tag_id : all_ids_) {
    const Pose3 &tag_pose3 = results.at<Pose3>(Symbol('l', tag_id));
    geometry_msgs::Pose tag_pose;
    SetPosition(&tag_pose.position, tag_pose3.x(), tag_pose3.y(),
                tag_pose3.z());
    SetOrientation(&tag_pose.orientation, tag_pose3.rotation().toQuaternion());
    // This should not change the size of all_sizes_ because all_sizes_ and
    // all_ids_ should have the same size
    auto it = all_sizes_.find(tag_id);
    map->AddOrUpdate(tag_id, (*it).second, tag_pose);
  }
}

void Mapper::Clear() {
  graph_.resize(0);
  initial_estimates_.clear();
}

Pose3 FromGeometryPose(const geometry_msgs::Pose &pose) {
  Point3 t(pose.position.x, pose.position.y, pose.position.z);
  Rot3 R(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                            pose.orientation.y, pose.orientation.z));
  return Pose3(R, t);
}

}  // namespace apriltag_ros
