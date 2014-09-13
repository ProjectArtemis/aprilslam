#include "apriltag_ros/tag_map.h"
#include "apriltag_ros/utils.h"

#include <ros/ros.h>
namespace apriltag_ros {

bool TagMap::AddFirstTag(const Apriltags& tags, int width, int height) {
  for (const Apriltag& tag : tags.apriltags) {
    if (IsInsideImageCenter(tag.center.x, tag.center.y, width, height, 3)) {
      // This magic number 3 means the boarder is 1/3 of the size
      set_first_tag_id(tag.id);
      Apriltag tag_world = tag;
      tags_.header.stamp = tags.header.stamp;
      // Set the first tag to origin
      SetPosition(&tag_world.pose.position);
      SetOrientation(&tag_world.pose.orientation);
      SetCorners(&tag_world.corners, tag_world.pose, tag.size);
      tag_world.center = tag_world.pose.position;
      ROS_INFO("tag %d: %f, %f", tag.id, tag.center.x, tag.center.y);
      return true;
    }
  }
  // No good tag detected around the center of image
  return false;
}

}  // namespace apriltag_ros
