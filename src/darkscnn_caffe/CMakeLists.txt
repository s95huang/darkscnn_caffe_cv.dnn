cmake_minimum_required(VERSION 3.0.2)
project(darkscnn_caffe)

## Compile as C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# force to use OpenCV with CUDA
find_package(OpenCV REQUIRED
NO_MODULE # should be optional, tells CMake to use config mode
PATHS /usr/local # look here
NO_DEFAULT_PATH) # and don't look anywhere else

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(darkscnn_caffe_node src/node.cpp)
target_link_libraries(darkscnn_caffe_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(DIRECTORY data/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)