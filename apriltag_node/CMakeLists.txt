cmake_minimum_required(VERSION 2.8.3)
project(apriltag_node)

find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs image_transport cv_bridge)
find_package(OpenCV)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(UMICH_DIR ../apriltag_umich)
set(MIT_DIR ../apriltag_mit)

include_directories(${UMICH_DIR})
link_directories(${UMICH_DIR}/lib)

include_directories(${MIT_DIR})
link_directories(${MIT_DIR}/lib)

add_executable(${PROJECT_NAME} src/apriltag_node.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    apriltag_mit apriltag_umich)
