#include <vector>
using std::vector;
#include <opencv2/core/core.hpp>

namespace kr {

typedef cv::Point2f Point2;
typedef cv::Point3f Point3;

struct Apriltag {
  unsigned id;
  Point2 c;
  vector<Point2> p;

  // Default constructor
  Apriltag() : id(0) {}
}; // class Apriltag

class ApriltagMap {
public:
  ApriltagMap() {}
  ~ApriltagMap() {}
  void getPose() {}
  void toMsg() {}

private:
  vector<Point3> pw_;
  vector<Point2> pi_;
  vector<Apriltag> tag_;
}; // class ApriltagMap

} // namespace kr
