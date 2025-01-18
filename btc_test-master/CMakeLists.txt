cmake_minimum_required(VERSION 3.0.2)
project(btc_loop)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_PREFIX_PATH "/opt/ros/noetic")
set(CMAKE_MODULE_PATH "${CMAKE_PREFIX_PATH}/share/catkin/cmake")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(GSL REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
# include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(load_cloud_show src/load_cloud_rgb.cpp src/std_ba.cpp  src/std.cpp)
target_link_libraries(load_cloud_show
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        tbb
        GSL::gsl
        gtsam)

add_executable(rgb_multi_mapping src/rgb_multi_mapping.cpp src/std_ba.cpp src/std.cpp )
target_link_libraries(rgb_multi_mapping
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        tbb
        dw
        GSL::gsl
        gtsam)