#ifndef APRILTAG_ROS_MAPPER_H_
#define APRILTAG_ROS_MAPPER_H_

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

namespace apriltag_ros {

// Feel like using iSAM2?
class Mapper {
 public:
  Mapper(double relinearize_thresh, int relinearize_skip)
      : init_(false),
        params_(gtsam::ISAM2GaussNewtonParams(), relinearize_thresh,
                relinearize_skip),
        isam2_(params_) {}

  bool init() const { return init_; }
  void Update(int num_iterations);

 private:
  bool init_;
  gtsam::ISAM2Params params_;
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_MAPPER_H_
