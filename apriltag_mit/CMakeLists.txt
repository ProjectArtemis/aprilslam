cmake_minimum_required(VERSION 2.8)
project(apriltag_mit)

message(STATUS "Project: " ${PROJECT_NAME})
message(STATUS "Source: " ${PROJECT_SOURCE_DIR})
message(STATUS "Binary: " ${PROJECT_BINARY_DIR})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#link libraries
link_directories(${PROJECT_SOURCE_DIR}/lib)

include_directories(AprilTags .)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
message(STATUS "Eigen3: " ${Eigen3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV: " ${OpenCV_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fPIC")
file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${CC_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBS})

add_executable(test_apriltag_mit src/test_apriltag_mit.cpp)
target_link_libraries(test_apriltag_mit ${OpenCV_LIBS})
target_link_libraries(test_apriltag_mit ${PROJECT_NAME})
