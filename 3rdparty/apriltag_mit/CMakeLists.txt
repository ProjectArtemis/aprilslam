cmake_minimum_required(VERSION 2.8.3)
project(apriltag_mit)

set(USE_ROS TRUE)

add_definitions(-std=c++11 -Wall -fPIC)

find_package(cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    )

if(USE_ROS)
    find_package(catkin REQUIRED)
    catkin_package(
        DEPENDS Eigen OpenCV
        CATKIN_DEPENDS
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )
else()
    message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
endif()

file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${CC_FILES})
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${Eigen_LIBRARIES}
    )

add_executable(test_apriltag_mit
    src/test/test_apriltag_mit.cpp)
target_link_libraries(test_apriltag_mit
    ${OpenCV_LIBRARIES}
    ${PROJECT_NAME}
    )

if(USE_ROS)
    add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
endif()
