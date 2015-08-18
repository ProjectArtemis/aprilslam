cmake_minimum_required(VERSION 2.8.3)
project(apriltag_mit)

set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(catkin REQUIRED COMPONENTS
)

find_package(cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltag_mit
  CATKIN_DEPENDS
  DEPENDS Eigen OpenCV
)

include_directories(include include/AprilTags)
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(apriltag_mit
  src/Edge.cc src/FloatImage.cc src/GLine2D.cc src/GLineSegment2D.cc src/Gaussian.cc src/GrayModel.cc src/Homography33.cc src/MathUtil.cc src/Quad.cc src/Segment.cc src/TagDetection.cc src/TagDetector.cc src/TagFamily.cc src/UnionFindSimple.cc
)
target_link_libraries(apriltag_mit
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
