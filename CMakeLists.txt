cmake_minimum_required(VERSION 3.0.2)
project(air_bumper)

SET(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  tf
  mavros
  geometry_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
add_message_files(
  FILES
  CostMap.msg
  activity_pose.msg
  confirmation.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
add_executable(imu_detection src/imu_detection.cpp)
target_link_libraries(imu_detection  ${catkin_LIBRARIES})

add_executable(add_point_cloud src/add_point_cloud_node.cpp)
target_link_libraries(add_point_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(grad_recover src/grad_recover.cpp)
target_link_libraries(grad_recover ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(grad_recover_node src/grad_recover_node.cpp)
target_link_libraries(grad_recover_node ${catkin_LIBRARIES})
target_link_libraries(grad_recover_node grad_recover)

add_library(air_bumper src/air_bumper.cpp)
target_link_libraries(air_bumper ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(air_bumper_node src/air_bumper_node.cpp)
target_link_libraries(air_bumper_node ${catkin_LIBRARIES})
target_link_libraries(air_bumper_node air_bumper)
target_link_libraries(air_bumper_node grad_recover)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

