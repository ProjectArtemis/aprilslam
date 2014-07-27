#include "tag_yaml.hpp"
#include <vector>

using namespace apriltag_ros;

int main(int arg, char ** argv) {

  double x = 0, y = 0;
  int tag_id = 0;

  YAML::Node node;
  Tag tag;

  for (int i=0; i < 9; i++) {
    x = 0.0;

    for (int j=0; j < 12; j++) {
      tag.id = tag_id++;
      tag.p[0] = cv::Point3d(x+0.152,y,0);
      tag.p[1] = cv::Point3d(x+0.152,y+0.152,0);
      tag.p[2] = cv::Point3d(x,y+0.152,0);
      tag.p[3] = cv::Point3d(x,y,0);

      node.push_back(tag);

      x += 0.152*2.0;
    }

    if (i==2 || i==5) {
      y += 0.178 + 0.152;
    } else {
      y += 0.152 * 2;
    }
  }

  return 0;
}
