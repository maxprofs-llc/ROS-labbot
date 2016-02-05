cmake_minimum_required(VERSION 2.8.3)
project(safe_drive)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
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
  LIBRARIES safe_drive_nodelet
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nodelet
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(safe_drive_nodelet src/safe_drive_nodelet.cpp)

## Declare a cpp executable
add_executable(safe_drive_node src/safe_drive_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(safe_drive_node safe_drive_node_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(safe_drive_nodelet ${catkin_LIBRARIES})
target_link_libraries(safe_drive_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS safe_drive_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   nodelet_plugins.xml safe_drive_nodelet.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
