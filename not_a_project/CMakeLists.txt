cmake_minimum_required(VERSION 3.0.2)
project(not_a_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  actionlib
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   filter.srv
   stopping_distance.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   approach_wall.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES not_a_project
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# --- Part 1, Service timer ---
add_executable(timer_srv_node src/timer.cpp)
target_link_libraries(timer_srv_node  ${catkin_LIBRARIES} )
add_dependencies(timer_srv_node not_a_project_gencpp)

add_executable(filter_node src/filter_node.cpp)
target_link_libraries(filter_node  ${catkin_LIBRARIES} )
add_dependencies(filter_node not_a_project_gencpp)


# --- Part 5, Homework 1 in cpp ---
add_executable(wall_aproach_node src/wall_approach.cpp)
target_link_libraries(wall_aproach_node  ${catkin_LIBRARIES} )
add_dependencies(
  wall_aproach_node
  ${wall_aproach_node_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(laser_front_filter src/front_filter.cpp)
target_link_libraries(laser_front_filter  ${catkin_LIBRARIES} )
add_dependencies(laser_front_filter not_a_project_gencpp)

add_executable(Wall_Aproach_action_client src/aproachWall_actionClient.cpp)
target_link_libraries(Wall_Aproach_action_client  ${catkin_LIBRARIES} )
add_dependencies(
  Wall_Aproach_action_client
  ${Wall_Aproach_action_client_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
