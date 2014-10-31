cmake_minimum_required(VERSION 2.8.3)
project(robotino_leds)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GoFromTo.srv
  SinalizeEnd.srv
  StopTransportation.srv
  TransportProduct.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotino_leds
#  CATKIN_DEPENDS roscpp std_msg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robotino_leds_node 
	src/robotino_leds_node.cpp
	src/RobotinoLeds.cpp
	)

target_link_libraries(robotino_leds_node
	${catkin_LIBRARIES}
	)
