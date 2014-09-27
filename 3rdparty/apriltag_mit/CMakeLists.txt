cmake_minimum_required(VERSION 2.8.3)
project(apriltag_mit)

add_definitions(-Wall -fPIC)

find_package(OpenCV REQUIRED)

include_directories(include)

if(USE_ROS)
    message(STATUS "Building w/ ROS")
    find_package(cmake_modules)
    find_package(Eigen REQUIRED)
    find_package(catkin REQUIRED)
    catkin_package(
        DEPENDS Eigen OpenCV
        CATKIN_DEPENDS
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )

    add_dependencies(${PROJECT_NAME}
      ${catkin_EXPORTED_TARGETS}
    )
    include_directories(${Eigen_INCLUDE_DIRS})
    set(LIBRARY_TYPE SHARED)
else()
    message(STATUS "Not building w/ ROS")
    # use FindEigen in this repo instead of ROS FindEigen
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
    find_package(Eigen)
    message(STATUS "Eigen found: ${EIGEN_FOUND}")
    message(STATUS "Eigen path: ${EIGEN_INCLUDE_DIRS}")
    include_directories(${EIGEN_INCLUDE_DIRS})
    message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
    # build static library outside of ROS
    set(LIBRARY_TYPE STATIC)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} ${LIBRARY_TYPE} ${CC_FILES})
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
