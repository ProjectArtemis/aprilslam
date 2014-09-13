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
      : params_(gtsam::ISAM2GaussNewtonParams(), relinearize_thresh,
                relinearize_skip),
        isam2_(params_) {}

 private:
  gtsam::ISAM2Params params_;
  gtsam::ISAM2 isam2_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_MAPPER_H_
