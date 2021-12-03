cmake_minimum_required(VERSION 2.8.3)
project(NNvisit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mps_driver
   CATKIN_DEPENDS 
   rospy
   roscpp
   message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ executable

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(mps_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


