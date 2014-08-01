//  AprilTag YAML encoding.
//  Added 21/07/2014 by gareth.

#ifndef TAG_YAML_HPP
#define TAG_YAML_HPP

#include <opencv2/core/core.hpp>
#include <yaml-cpp/yaml.h>

namespace apriltag_ros {

struct Tag {
  int id;            /// Tag ID
  cv::Point3d p[4];  /// Corners
};
} // namespace apriltag_ros

//  Encoding/decoding functionality
namespace YAML {

template <> struct convert<apriltag_ros::Tag> {
  static Node encode(const apriltag_ros::Tag &rhs);
  static bool decode(const Node &node, apriltag_ros::Tag &rhs);
};
} // namespace YAML

YAML::Emitter& operator << (YAML::Emitter& out, const apriltag_ros::Tag& tag);

#endif // TAG_YAML_HPP
