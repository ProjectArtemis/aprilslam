//  AprilTag YAML encoding.
//  Added 21/07/2014 by gareth.

#include "tag_yaml.hpp"

YAML::Node YAML::convert<apriltag_ros::Tag>::encode(
    const apriltag_ros::Tag &rhs) {
  YAML::Node node;
  node["id"] = rhs.id;

  //  convert to vector of coordinates
  for (int i = 0; i < 4; i++) {
    YAML::Node pointNode;
    pointNode["x"] = rhs.p[i].x;
    pointNode["y"] = rhs.p[i].y;
    pointNode["z"] = rhs.p[i].z;

    node["corners"].push_back(pointNode);
  }
  return node;
}

bool YAML::convert<apriltag_ros::Tag>::decode(const YAML::Node &node,
                                              apriltag_ros::Tag &rhs) {

  if (!node.IsMap()) {
    return false;
  }

  rhs.id = node["id"].as<int>();

  YAML::Node pointSeq = node["corners"];
  if (!pointSeq.IsSequence()) {
    return false;
  }

  //  this will trigger an exception if any points are invalid...
  for (int i = 0; i < 4; i++) {
    rhs.p[i].x = pointSeq[i]["x"].as<double>();
    rhs.p[i].y = pointSeq[i]["y"].as<double>();
    rhs.p[i].z = pointSeq[i]["z"].as<double>();
  }

  return true;
}

YAML::Emitter &operator<<(YAML::Emitter &out, const apriltag_ros::Tag &tag) {
  out << YAML::Block;
  out << YAML::BeginMap;
  out << YAML::Key << "id" << YAML::Value << tag.id;
  out << YAML::Key << "corners" << YAML::Value;
  out << YAML::BeginSeq;
  for (int i = 0; i < 4; i++) {
    out << YAML::Flow;
    out << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << tag.p[i].x;
    out << YAML::Key << "y" << YAML::Value << tag.p[i].y;
    out << YAML::Key << "z" << YAML::Value << tag.p[i].z;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}
