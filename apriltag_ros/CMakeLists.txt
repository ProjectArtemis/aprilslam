cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport cv_bridge image_geometry
    sensor_msgs geometry_msgs message_generation
    apriltag_mit std_msgs
    )

add_message_files(DIRECTORY msg FILES Apriltag.msg Apriltags.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

# apriltag_ros library
add_library(${PROJECT_NAME}
    src/visualizer/visualizer.cpp
    src/detector/detector_node.cpp
    src/mapper/mapper_node.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

# detector node
add_executable(detector_node
    src/detector/detector_main.cpp
    )
target_link_libraries(detector_node
    ${PROJECT_NAME}
    )

add_executable(mapper_node
    src/mapper/mapper_main.cpp
    )
target_link_libraries(mapper_node
    ${PROJECT_NAME}
    )

# dependencies
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )

# Map generator
#add_executable(generate_map
#src/generate_map.cpp
#src/tag_yaml.cpp
#)

#target_link_libraries(generate_map
#${catkin_LIBRARIES}
#${OpenCV_LIBRARIES}
#yaml-cpp
#)
