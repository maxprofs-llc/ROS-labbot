cmake_minimum_required(VERSION 2.8.3)
project(command_mux)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
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
  LIBRARIES command_mux_nodelet
  CATKIN_DEPENDS geometry_msgs roscpp rospy nodelet
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(command_mux_nodelet src/command_mux_nodelet.cpp)

## Declare a cpp executable
add_executable(command_mux_node src/command_mux_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(command_mux_node command_mux_node_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(command_mux_nodelet ${catkin_LIBRARIES})
target_link_libraries(command_mux_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS command_mux_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
