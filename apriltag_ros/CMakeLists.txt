cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport cv_bridge camera_info_manager
    sensor_msgs geometry_msgs message_generation
    apriltag_mit apriltag_umich)
find_package(OpenCV)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

# Apriltag node
add_executable(${PROJECT_NAME}_node
    src/apriltag_ros_node.cpp
    src/tag_yaml.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
    )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

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
