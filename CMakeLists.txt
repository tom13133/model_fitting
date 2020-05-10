
cmake_minimum_required(VERSION 2.8.3)
project(model_fitting)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED COMPONENTS )
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES target_processing
#  CATKIN_DEPENDS cv_bridge geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visual_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  
  ${SOPHUS_INCLUDE_DIRS}
)


## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/kalman_filter.cpp
   src/LidarProcessor.cpp
   src/lidar_processing.cpp
   src/model.cpp
   src/ransac.cpp
 )

add_executable(LidarProcessor_node script/LidarProcessor_node.cpp)
add_dependencies(LidarProcessor_node ${main_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LidarProcessor_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${SOPHUS_LIBRARIES})


