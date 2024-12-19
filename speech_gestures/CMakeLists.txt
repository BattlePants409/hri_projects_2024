cmake_minimum_required(VERSION 3.0.2)
project(spatial_interaction)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# Uncomment and add your message files if needed
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# Uncomment and add your service files if needed
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# Uncomment if using custom messages or services
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts (e.g., nodes)
catkin_install_python(PROGRAMS
  scripts/adjust_head_node.py
  scripts/class_node.py
  scripts/figure_eight_movement.py
  scripts/gesture_toward_hand.py
  scripts/join_group.py
  scripts/joint_demo.py
  scripts/listener.py
  scripts/look_in_hand_direction.py
  scripts/move_straight.py
  scripts/move_straight_odom.py
  scripts/multiple_joints.py
  scripts/obstacle_avoidance_advanced.py
  scripts/obstacle_avoidance_basic.py
  scripts/obstacle_avoidance_move_toward_goal.py
  scripts/sensor_reader.py
  scripts/square_movement.py
  scripts/talker.py
  scripts/tf_look_at_hand.py
  scripts/triangle_movement.py
  scripts/turn_circle.py
  scripts/turn_odom.py
  scripts/wiggler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# Uncomment if you have tests
# catkin_add_nosetests(test)

