#ifndef APRILSLAM_TAG_MAP_H_
#define APRILSLAM_TAG_MAP_H_

#include <aprilslam/Apriltags.h>
#include <opencv2/core/core.hpp>
#include <set>

namespace aprilslam {

class TagMap {
 public:
  TagMap() = default;

  void AddOrUpdate(const Apriltag& tag_w, const geometry_msgs::Pose& pose);
  void UpdateTag(Apriltag* tag_w, const geometry_msgs::Pose& pose);
  void AddFirstTag(const Apriltag& tag_c);
  bool EstimatePose(const std::vector<Apriltag>& tags_c, const cv::Matx33d& K,
                    const cv::Mat_<double>& D, geometry_msgs::Pose* pose) const;

  bool init() const { return !tags_w().empty(); }
  const aprilslam::Apriltag& first_tag() const { return tags_w().front(); }
  const std::vector<aprilslam::Apriltag>& tags_w() const { return tags_w_; }

 private:
  void AddTag(const Apriltag& tag, const geometry_msgs::Pose& pose);

  std::vector<aprilslam::Apriltag> tags_w_;
};

std::vector<aprilslam::Apriltag>::const_iterator FindById(
    int id, const std::vector<aprilslam::Apriltag>& tags);

std::vector<aprilslam::Apriltag>::iterator FindById(
    int id, std::vector<aprilslam::Apriltag>& tags);
}  // namespace aprilslam

#endif  // APRILSLAM_TAG_MAP_H_
