cmake_minimum_required(VERSION 2.8)
project(apriltag_umich)

set(USE_ROS TRUE)

add_definitions(-std=gnu99 -fPIC)

find_package (Threads)
include_directories(include)

if(USE_ROS)
    find_package(catkin REQUIRED)
    catkin_package(
        CATKIN_DEPENDS
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )
else(USE_ROS)
    message(STATUS "Source: " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary: " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
endif(USE_ROS)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB C_FILES ${PROJECT_SOURCE_DIR}/src/*.c)
list(REMOVE_ITEM C_FILES ${PROJECT_SOURCE_DIR}/src/tagtest.c)
add_library(${PROJECT_NAME} SHARED ${C_FILES})
target_link_libraries(${PROJECT_NAME}
    ${CMAKE_THREAD_LIBS_INIT}
    )

#apriltag umich
add_executable(test_apriltag_umich src/test/test_apriltag_umich.cpp)
target_link_libraries(test_apriltag_umich
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )
