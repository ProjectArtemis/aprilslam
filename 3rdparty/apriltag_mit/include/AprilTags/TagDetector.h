#ifndef APRILTAGS_TAGDETECTOR_H_
#define APRILTAGS_TAGDETECTOR_H_

#include <vector>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/FloatImage.h"

namespace AprilTags {

class TagDetector {
 public:
  const TagFamily thisTagFamily;

  //! Constructor
  // note: TagFamily is instantiated here from TagCodes
  TagDetector(const TagCodes& tagCodes) : thisTagFamily(tagCodes) {}

  std::vector<TagDetection> extractTags(const cv::Mat& image);
};

}  // namespace AprilTags

#endif  // APRILTAGS_TAGDETECTOR_H_
