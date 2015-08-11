cmake_minimum_required(VERSION 2.8.3)
project(test_odometry)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nodelet
  ecl_threads
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
  LIBRARIES test_odometry_nodelet
  CATKIN_DEPENDS geometry_msgs joy nav_msgs roscpp rospy sensor_msgs nodelet std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(test_odometry_nodelet src/test_odometry_nodelet.cpp)

## Declare a cpp executable
add_executable(test_odometry_node src/test_odometry_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(test_odometry_node test_odometry_node_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_odometry_nodelet ${catkin_LIBRARIES})
target_link_libraries(test_odometry_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS test_odometry_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
