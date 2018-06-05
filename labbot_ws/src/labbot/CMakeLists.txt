cmake_minimum_required(VERSION 2.8.3)
project(labbot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  tf
  std_msgs
  message_generation
  nodelet
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  msgFromLabbot.msg
  msgToLabbot.msg  
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES labbot nodelet
  CATKIN_DEPENDS message_runtime nodelet
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(labbotTeleoperator src/labbot_teleoperation_gamepad.cpp)
#target_link_libraries(labbotTeleoperator ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(labbot_teleoperation_twist_node src/labbot_teleoperation_twist.cpp)
add_executable(labbot_odometry_node src/labbot_odometry.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(labbot_teleoperation_twist_node labbot_teleoperation_gamepad_cpp)
add_dependencies(labbot_odometry_node labbot_generate_messages_cpp)

## Declare a cpp library
add_library(labbot_teleoperation_twist_nodelet src/labbot_teleoperation_twist_nodelet.cpp)

target_link_libraries(labbot_teleoperation_twist_node ${catkin_LIBRARIES})
target_link_libraries(labbot_odometry_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS labbot_teleoperation_twist_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
