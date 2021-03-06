cmake_minimum_required(VERSION 2.8.3)
project(enif_iuc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  sensor_msgs
  serial
  message_generation
  mavros_msgs
  nav_msgs
  mps_driver  
  particle_filter
  )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Signal.msg
  Waypoint.msg
  WaypointTask.msg
  AgentTakeoff.msg
  AgentBox.msg
  AgentState.msg
  AgentMPS.msg
  AgentWaypointTask.msg
  AgentGlobalPosition.msg
  AgentHeight.msg
  AgentBatteryState.msg
  AgentHome.msg
  AgentLocal.msg
  AgentCheck.msg
  AgentSource.msg
  AgentEstimatedGaussian.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs mps_driver sensor_msgs mavros_msgs geometry_msgs nav_msgs particle_filter
  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS #other_catkin_pkg
  rospy 
  roscpp
  message_runtime
  std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)




## Declare a cpp executable
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_executable(enif_iuc_quad src/enif_iuc_quad.cpp)
#add_executable(enif_iuc_ground src/enif_iuc_ground.cpp)
add_executable(enif_iuc_publisher src/signal_subscriber.cpp)

add_dependencies(enif_iuc_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(enif_iuc_quad ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(enif_iuc_quad
  ${catkin_LIBRARIES}
)

#target_link_libraries(enif_iuc_ground
#  ${catkin_LIBRARIES}
#  serial
#)

target_link_libraries(enif_iuc_publisher
  ${catkin_LIBRARIES}
)







