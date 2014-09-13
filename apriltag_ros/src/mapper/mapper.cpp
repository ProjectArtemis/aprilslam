#include "apriltag_ros/mapper.h"

namespace apriltag_ros {

void Mapper::Update(int num_iterations) {
  isam2_.update(graph_, initial_estimates_);
  if (num_iterations > 1) {
    for (int i = 1; i < num_iterations; ++i) {
      isam2_.update();
    }
  }
}

}  // namespace apriltag_ros
